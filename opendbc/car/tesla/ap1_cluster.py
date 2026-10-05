"""AP1 instrument-cluster frames. Pure logic, no CANPacker, no messaging.

Ported from BogPilot/openpilot tag ap1-driving-milestone-2 (92e84996)
selfdrive/car/tesla/cluster.py. sunnypilot differences: the DBC is opendbc's
own opendbc/dbc/tesla_can.dbc (0x399 / 0x389 / 0x239 are byte-identical to
BogPilot's), and the model path comes in as plain x / y lists
(CarControlSP.modelPathX / modelPathY, filled by controlsd_ext from
modelV2.position for AP1 only) instead of a SubMaster read in the controller.

Behavior reference: Tinkla earlytesla-openpilot (tesla_unity_dev,
501c7de91b59c70510e9dc1585acfde9b7102c93) selfdrive/car/tesla/HUD_module.py
and teslacan.py create_das_status / create_das_status2 / create_lane_message,
plus earlytesla-panda (f7751e4) board/safety/safety_tesla.h TESLA_AP_FWD_MODDED
and tesla_fwd_hook. This file is BogPilot's own code. Nothing is copied from
Tesla firmware.

What Tinkla does on an AP1 car: openpilot sends its cluster values to the
panda. The panda takes each stock Mobileye frame from bus 2, keeps the stock
bits named by a per-address mask (counter, speed limits, blind spot, ...),
writes openpilot's bits over the rest, fixes the checksum, and forwards it to
bus 0. So the cluster sees stock timing and stock data, except for the fields
that make it look like Autosteer is steering.

BogPilot's panda forward hook only gets (bus, addr). It cannot edit a frame.
So the same result is built here instead: each new stock frame read from bus 2
is rebuilt from its decoded DBC signals, openpilot's fields are written over
it, and openpilot sends it on bus 0. The panda drops the stock copy only while
openpilot has recently sent that address (tesla_fwd_hook). openpilot sends
nothing for an address until it has seen a stock frame for it, so no frame is
ever made up without a stock source.

Counter: openpilot sends stock counter + 1. While substituting, the stock frame
that would have carried that counter is the one the panda drops, so the
counter seen on bus 0 stays continuous at the start and end of substitution.

Frames (opendbc/tesla_can.dbc, the DBC this tree loads):

  0x399 AutopilotStatus  (Tinkla's DBC calls it DAS_status), 2 Hz, checksum
  0x389 DAS_status2      2 Hz, checksum
  0x239 DAS_lanes        10 Hz, no checksum

Checksum, checked against Tinkla tesla_compute_checksum and AP1 rlogs:
(addr & 0xFF) + (addr >> 8) + sum(bytes 0..6), & 0xFF, in byte 7.

Not done here: 0x3e9 DAS_bodyControls (turn, hazard, headlight and wiper
requests; real stock frames carry bits no DBC signal covers), DAS_object,
DAS_telemetry, the warning matrices, pre-AP 0x659. See docs/tesla/DIVERGENCES.md.

Not a product, no warranty, driver remains responsible, comply with local law.
This does not make the car safe to drive.
"""

import os
import re
from dataclasses import dataclass

import numpy as np

AUTOPILOT_STATUS = 0x399
DAS_STATUS2 = 0x389
DAS_LANES = 0x239
CLUSTER_ADDRS = (AUTOPILOT_STATUS, DAS_STATUS2, DAS_LANES)
CLUSTER_BUS = 0

MSG_NAMES = {
  AUTOPILOT_STATUS: "AutopilotStatus",
  DAS_STATUS2: "DAS_status2",
  DAS_LANES: "DAS_lanes",
}
COUNTER_SIGNALS = {
  AUTOPILOT_STATUS: "DAS_statusCounter",
  DAS_STATUS2: "DAS_status2Counter",
  DAS_LANES: "DAS_lanesCounter",
}
CHECKSUM_SIGNALS = {
  AUTOPILOT_STATUS: "DAS_statusChecksum",
  DAS_STATUS2: "DAS_status2Checksum",
}

# Tinkla earlytesla-panda TIME_TO_HIDE_ERRORS = 4000000 us. Its comment says
# "less than 3 seconds"; the constant is 4 s and that is what ran.
POST_DISENGAGE_NS = 4_000_000_000

# tesla_can.dbc VAL_ 921 autopilotStatus / DAS_autopilotState.
AP_STATE_UNAVAILABLE = 1
AP_STATE_AVAILABLE = 2
AP_STATE_ACTIVE_MIN = 3
# Tinkla HUD_module: DAS_op_status = 5 if enabled.
AP_STATE_OPENPILOT = 5

# VAL_ 921 DAS_autopilotHandsOnState. Tinkla: 2 by default while enabled,
# 3 for a quiet steer-required or for human steering, 5 with a chime.
HANDS_ON_NOT_DETECTED = 2
HANDS_ON_VISUAL = 3
HANDS_ON_CHIME_2 = 5

# VAL_ 921 DAS_autoLaneChangeState.
ALC_NO_LANES = 1
ALC_ONLY_L = 6
ALC_ONLY_R = 7
ALC_BOTH = 8
ALC_IN_PROGRESS_L = 9
ALC_IN_PROGRESS_R = 10

# VAL_ 921 DAS_laneDepartureWarning.
LDW_LEFT = 1
LDW_RIGHT = 2

# DAS_status2 DAS_csaState. Not in this tree's tesla_can.dbc: BO_ 905 there has
# DAS_lssState 31|3. Tinkla's tesla_can.dbc (BogGyver/opendbc 9c0b6fe) has
# DAS_relaxCruiseLimits 31|1 and DAS_csaState 32|2 (VAL 2 ENABLE, 1 AVAILABLE).
# AP1 rlogs only ever set bits 32-33 of this frame (to 0 or 1), never bit 31,
# which fits Tinkla's layout. Tinkla sends 2 while enabled.
CSA_SIGNAL = ("DAS_csaState", 32, 2)
CSA_ENABLE = 2
# VAL_ 905 DAS_longCollisionWarning 1 FCM_LONG_COLLISION_WARNING_VEHICLE_UNKNOWN.
LONG_FCW_VEHICLE = 1

# Tinkla HUD_module: IC_LANE_SCALE = 0.5, so path coefficients are scaled by
# f = 2 (C_n * f^n). Fallback view range 50 m when modelV2 path is unusable.
# LineUsage 2 is FUSED. DBC DAS_virtualLaneViewRange is [0|160] m.
IC_LANE_SCALE = 0.5
LANE_VIEW_RANGE_M = 50
VIEW_RANGE_MIN_M = 0
VIEW_RANGE_MAX_M = 160
# Minimum usable model path length before falling back to actuator curvature.
MODEL_PATH_MIN_M = 5.0
# Tinkla HUD_module fits the path out to 100 m (max_distance).
MODEL_FIT_MAX_M = 100.0
# Curvature fit window. Through-origin fit over 50 m matched stock Mobileye
# C2 sign ~98% on real curves in the 979dbf8 drive replay.
MODEL_C2_FIT_M = 50.0
LINE_USAGE_FUSED = 2
LINE_USAGE_REJECTED = 0

# DBC DAS_virtualLaneC0..C3 clip ranges (opendbc/tesla_can.dbc).
C0_RANGE = (-3.5, 3.5)
C1_RANGE = (-0.2, 0.2)
C2_RANGE = (-0.0025, 0.0025)
C3_RANGE = (-3e-5, 3e-5)


@dataclass(frozen=True)
class Signal:
  name: str
  start: int
  size: int
  factor: float
  offset: float


def _dbc_path():
  from opendbc import DBC_PATH
  return os.path.join(DBC_PATH, "tesla_can.dbc")


_SG = re.compile(r"^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)")


def load_signals(path=None, addrs=CLUSTER_ADDRS):
  """{addr: {name: Signal}} from the DBC this tree ships.

  Only little-endian unsigned signals are expected in these frames. Anything
  else raises, so a changed DBC cannot be packed wrong silently.
  """
  path = path or _dbc_path()
  out = {a: {} for a in addrs}
  cur = None
  with open(path, encoding="utf-8", errors="replace") as f:
    for line in f:
      if line.startswith("BO_ "):
        addr = int(line.split()[1])
        cur = addr if addr in out else None
      elif cur is not None:
        m = _SG.match(line)
        if m is None:
          if line.strip() == "":
            cur = None
          continue
        name, start, size, le, sign, factor, offset = m.groups()
        if le != "1" or sign != "+":
          raise ValueError(f"0x{cur:x} {name}: only little-endian unsigned is supported")
        out[cur][name] = Signal(name, int(start), int(size), float(factor), float(offset))
  for a in addrs:
    if not out[a]:
      raise ValueError(f"0x{a:x} not found in {path}")
  return out


def _raw(sig, value):
  raw = int(round((float(value) - sig.offset) / sig.factor))
  return max(0, min(raw, (1 << sig.size) - 1))


def set_raw(word, start, size, raw):
  mask = ((1 << size) - 1) << start
  return (word & ~mask) | ((int(raw) << start) & mask)


def get_raw(word, start, size):
  return (word >> start) & ((1 << size) - 1)


def tesla_checksum(addr, dat):
  return ((addr & 0xFF) + ((addr >> 8) & 0xFF) + sum(dat[:7])) & 0xFF


def pack(addr, sigs, values):
  """Rebuild 8 bytes from decoded values. Unknown names are an error.

  Signals missing from values are 0. On AP1 rlogs, every bit the stock
  0x399 / 0x389 / 0x239 frames set lies inside a DBC signal of this tree, so
  rebuilding a stock frame from its decoded values gives the same bytes.
  """
  word = 0
  for name, value in values.items():
    word = set_raw(word, sigs[name].start, sigs[name].size, _raw(sigs[name], value))
  return word


def finish(addr, word, counter):
  """Set counter and, where the frame has one, the checksum. Returns bytes."""
  sigs = SIGNALS[addr]
  c = sigs[COUNTER_SIGNALS[addr]]
  word = set_raw(word, c.start, c.size, counter & ((1 << c.size) - 1))
  dat = bytearray(word.to_bytes(8, "little"))
  if addr in CHECKSUM_SIGNALS:
    k = sigs[CHECKSUM_SIGNALS[addr]]
    assert k.start == 56 and k.size == 8
    dat[7] = tesla_checksum(addr, dat)
  return bytes(dat)


def unpack(addr, dat):
  word = int.from_bytes(bytes(dat), "little")
  return {n: get_raw(word, s.start, s.size) * s.factor + s.offset for n, s in SIGNALS[addr].items()}


SIGNALS = load_signals()


@dataclass(frozen=True)
class ModelPath:
  """Fitted DAS_lanes poly from modelV2.position (ego frame, y left+)."""
  c0: float
  c1: float
  c2: float
  c3: float
  view_range_m: float


@dataclass(frozen=True)
class HudInputs:
  """Everything the cluster frames read. Built in CarController from CC/CS."""
  enabled: bool
  fcw: bool
  steer_required: bool
  audible: bool
  human_steering: bool
  left_lane_depart: bool
  right_lane_depart: bool
  left_blinker: bool
  right_blinker: bool
  curvature: float
  # When False, ClusterController sends nothing (panda forwards stock).
  ic_integration: bool = True
  # Engaged model path. None => actuator curvature + 50 m fallback.
  model_path: ModelPath | None = None


def hands_on_state(h):
  """Tinkla HUD_module DAS_hands_on_state while enabled."""
  state = HANDS_ON_NOT_DETECTED
  if h.steer_required:
    state = HANDS_ON_CHIME_2 if h.audible else HANDS_ON_VISUAL
  if h.human_steering:
    state = HANDS_ON_VISUAL
  return state


def alc_state(h, left_lane, right_lane):
  """Tinkla DAS_alca_state. Lane presence is the stock Mobileye DAS_lanes bit.

  Tinkla used openpilot lane-line probabilities. Lane presence still comes
  from the stock camera's left/right lane bits (modelV2 is only for the path
  poly / view range, not lane existence or lead cars).
  A lane change in progress is openpilot's (CC blinkers), 9 left, 10 right.
  """
  if h.left_blinker:
    return ALC_IN_PROGRESS_L
  if h.right_blinker:
    return ALC_IN_PROGRESS_R
  if left_lane and right_lane:
    return ALC_BOTH
  if left_lane:
    return ALC_ONLY_L
  if right_lane:
    return ALC_ONLY_R
  return ALC_NO_LANES


def _clip(value, lo_hi):
  lo, hi = lo_hi
  return max(lo, min(hi, float(value)))


def lane_c2(curvature):
  """Tinkla scales the path x^2 coefficient by (1/IC_LANE_SCALE)^2.

  For a path of curvature k, y = k/2 * x^2. Left positive, as in modelV2.
  Clipped to DBC DAS_virtualLaneC2 range.
  """
  f = 1.0 / IC_LANE_SCALE
  return _clip((curvature / 2.0) * f * f, C2_RANGE)


def clamp_view_range_m(meters):
  """DBC DAS_virtualLaneViewRange is integer meters in [0|160]."""
  return int(round(_clip(meters, (VIEW_RANGE_MIN_M, VIEW_RANGE_MAX_M))))


def path_from_model_v2(model_v2) -> ModelPath | None:
  """Fit C2 from modelV2.position x/y (ego frame). C0, C1, C3 stay 0.

  Uses the planned-path position polynomial openpilot already publishes.
  Coefficients are scaled by (1/IC_LANE_SCALE)^n like Tinkla, then clipped to
  the DBC ranges. View range is the last valid x in meters, clamped to
  [0|160]. Returns None when the path is missing or too short so the caller
  can fall back to actuator curvature + 50 m.

  Sign: y left-positive matches modelV2 and the actuator-curvature path
  already shipped (on a logged AP1 drive stock Mobileye c2 agrees with this sign more
  often than the negated fit; stock c0 is a different reference and is not
  matched). Lead cars / DAS_object / DAS_telemetry are not read here.
  """
  try:
    pos = model_v2.position
    return path_from_xy(pos.x, pos.y)
  except Exception:
    return None


def path_from_xy(x, y) -> ModelPath | None:
  """path_from_model_v2 on plain lists (modelV2.position.x / .y, ego frame).

  sunnypilot: controlsd_ext copies those two lists into CarControlSP for AP1
  only. Empty lists (stale/absent model, or another platform) return None.
  """
  try:
    xs = np.asarray(list(x), dtype=float)
    ys = np.asarray(list(y), dtype=float)
  except Exception:
    return None
  if xs.size < 4 or ys.size < 4 or xs.size != ys.size:
    return None
  mask = (xs > 0.5) & np.isfinite(xs) & np.isfinite(ys)
  xs = xs[mask]
  ys = ys[mask]
  if xs.size < 4:
    return None
  # View range: how far ahead the model path is valid, capped at Tinkla's
  # 100 m max_distance (then clamped to the DBC range when packed).
  view = min(float(xs[-1]), MODEL_FIT_MAX_M)
  if view < MODEL_PATH_MIN_M:
    return None
  # Fit y = c2 * x^2 through the origin over the near path. The model path
  # starts at the car, so C0 (offset) and C1 (heading) are 0 by construction.
  # A free fit traded C0 against C1 (corr -0.98 on the 979dbf8 drive) and
  # drew the line off to one side and across the car; Tinkla also forces
  # C1 = 0 (suppress_x_coord) and has C0 = 0 ("always center") as an option.
  near = xs <= MODEL_C2_FIT_M
  if int(near.sum()) < 4:
    return None
  xn = xs[near]
  yn = ys[near]
  denom = float(np.sum(xn ** 4))
  if not np.isfinite(denom) or denom <= 0.0:
    return None
  c2 = float(np.sum(xn * xn * yn) / denom)
  if not np.isfinite(c2):
    return None
  f = 1.0 / IC_LANE_SCALE
  return ModelPath(
    c0=0.0,
    c1=0.0,
    c2=_clip(c2 * (f * f), C2_RANGE),
    c3=0.0,
    view_range_m=float(clamp_view_range_m(view)),
  )


def lanes_path_values(h: HudInputs):
  """C0..C3 and view range for DAS_lanes. Model path or curvature fallback."""
  if h.model_path is not None:
    p = h.model_path
    # C0 and C1 stay 0 (see path_from_model_v2).
    return 0.0, 0.0, p.c2, p.c3, clamp_view_range_m(p.view_range_m)
  return 0.0, 0.0, lane_c2(h.curvature), 0.0, LANE_VIEW_RANGE_M


def build_autopilot_status(stock, h, stock_lanes, mode):
  """stock: decoded AutopilotStatus values. mode 'engaged' or 'post'."""
  v = dict(stock)
  stock_state = int(round(stock.get("autopilotStatus", 0)))
  if mode == "engaged":
    # Tinkla: 5 while enabled. Stock already active (3..5) is left alone.
    if stock_state < AP_STATE_ACTIVE_MIN:
      v["autopilotStatus"] = AP_STATE_OPENPILOT
    v["DAS_autopilotHandsOnState"] = hands_on_state(h)
    left = bool(stock_lanes and stock_lanes.get("DAS_leftLaneExists", 0))
    right = bool(stock_lanes and stock_lanes.get("DAS_rightLaneExists", 0))
    v["DAS_autoLaneChangeState"] = alc_state(h, left, right)
    # Warnings: openpilot's only add to stock. A stock warning is never cleared.
    if h.fcw:
      v["DAS_forwardCollisionWarning"] = 1
    if h.left_lane_depart:
      v["DAS_laneDepartureWarning"] = LDW_LEFT
    elif h.right_lane_depart:
      v["DAS_laneDepartureWarning"] = LDW_RIGHT
  elif mode == "post":
    # Tinkla tesla_fwd_hook: for 4 s after disengage, autopilot state 2.
    # Here only when stock says UNAVAILABLE (the state openpilot caused).
    if stock_state == AP_STATE_UNAVAILABLE:
      v["autopilotStatus"] = AP_STATE_AVAILABLE
  else:
    raise ValueError(mode)
  return pack(AUTOPILOT_STATUS, SIGNALS[AUTOPILOT_STATUS], v)


def build_das_status2(stock, h, mode):
  v = dict(stock)
  # Tinkla: activationFailureStatus is openpilot's (0) while enabled, and is
  # zeroed in the 4 s window after disengage.
  v["DAS_activationFailureStatus"] = 0
  if mode == "engaged":
    v["DAS_driverInteractionLevel"] = 0
    if h.fcw:
      v["DAS_longCollisionWarning"] = LONG_FCW_VEHICLE
  elif mode != "post":
    raise ValueError(mode)
  word = pack(DAS_STATUS2, SIGNALS[DAS_STATUS2], v)
  if mode == "engaged":
    name, start, size = CSA_SIGNAL
    word = set_raw(word, start, size, CSA_ENABLE)
  return word


def build_das_lanes(stock, h):
  """Rewrite path coefficients only. Keep stock lane presence and usage.

  AP1 rlogs: Mobileye often leaves DAS_*LaneExists at 0 while DAS_*LineUsage
  is 2 (FUSED). The IC draws from LineUsage. Overwriting usage from the
  Exists bits cleared both sides and hid the lines while engaged (stock
  still had usage 2,2 on bus 2). Tinkla set Exists/Usage from model probs;
  lane presence stays stock here. Path C0..C2 / view range come from
  modelV2.position when HudInputs.model_path is set, else actuator
  curvature + 50 m. C3 stays 0 (DBC has the signal; stock cubic terms are
  tiny and a deg-3 fit is unstable at IC precision).
  """
  v = dict(stock)
  c0, c1, c2, c3, view = lanes_path_values(h)
  v["DAS_virtualLaneC0"] = c0
  v["DAS_virtualLaneC1"] = c1
  v["DAS_virtualLaneC2"] = c2
  v["DAS_virtualLaneC3"] = c3
  v["DAS_virtualLaneViewRange"] = view
  return pack(DAS_LANES, SIGNALS[DAS_LANES], v)


class ClusterController:
  """Decides, per control step, which cluster frames openpilot sends.

  update() takes the newest decoded stock frame per address that arrived this
  step (or none). Nothing is sent for an address without a new stock frame.
  While enabled and HudInputs.ic_integration: all three. For POST_DISENGAGE_NS
  after enabled drops: AutopilotStatus and DAS_status2 with Tinkla's
  post-disengage rewrite only. When ic_integration is False, send nothing
  immediately (panda forwards stock). Otherwise nothing, and stock flows
  through the panda. Missing modelV2 never stops substitution by itself.
  """

  def __init__(self):
    self.prev_enabled = False
    self.disengaged_ns = None
    self.last_lanes = None

  def mode(self, enabled, now_nanos):
    if enabled:
      self.disengaged_ns = None
      mode = "engaged"
    else:
      if self.prev_enabled:
        self.disengaged_ns = now_nanos
      if self.disengaged_ns is not None and (now_nanos - self.disengaged_ns) < POST_DISENGAGE_NS:
        mode = "post"
      else:
        self.disengaged_ns = None
        mode = None
    self.prev_enabled = enabled
    return mode

  def update(self, h, new_stock, now_nanos):
    """new_stock: {addr: decoded values} for stock frames received this step.

    Returns [(addr, bytes)] to send on CLUSTER_BUS.
    """
    if DAS_LANES in new_stock:
      self.last_lanes = new_stock[DAS_LANES]
    if not h.ic_integration:
      # Toggle off: stop substitution immediately so stock frames forward.
      self.prev_enabled = False
      self.disengaged_ns = None
      return []
    mode = self.mode(h.enabled, now_nanos)
    if mode is None:
      return []

    out = []
    for addr in CLUSTER_ADDRS:
      stock = new_stock.get(addr)
      if stock is None:
        continue
      if addr == AUTOPILOT_STATUS:
        word = build_autopilot_status(stock, h, self.last_lanes, mode)
      elif addr == DAS_STATUS2:
        word = build_das_status2(stock, h, mode)
      else:
        if mode != "engaged":
          continue
        word = build_das_lanes(stock, h)
      counter = int(round(stock.get(COUNTER_SIGNALS[addr], 0))) + 1
      out.append((addr, finish(addr, word, counter)))
    return out
