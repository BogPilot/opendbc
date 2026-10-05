"""AP1 cluster frames (ap1_cluster.py): pure logic.

Ported from BogPilot/openpilot tag ap1-driving-milestone-2 (92e84996)
selfdrive/car/tesla/tests/test_ap1_cluster.py (the pure ClusterController /
DBC / model-path tests). The CarState / CarController wiring is tested
against the real opendbc parser and packer in test_tesla_ap1_milestone2.py.
Real stock payloads come from fixtures/ap1_cluster_frames.py.

Not a driving validation.
"""

from types import SimpleNamespace

import pytest

from opendbc.car.tesla import ap1_cluster as c
from opendbc.car.tesla.tests.fixtures.ap1_cluster_frames import (
  AUTOPILOT_STATUS_HEX,
  DAS_LANES_HEX,
  DAS_STATUS2_HEX,
  DAS_BODY_CONTROLS_UNCOVERED_BITS_SEEN,
)

STOCK = {
  c.AUTOPILOT_STATUS: [bytes.fromhex(h) for h in AUTOPILOT_STATUS_HEX],
  c.DAS_STATUS2: [bytes.fromhex(h) for h in DAS_STATUS2_HEX],
  c.DAS_LANES: [bytes.fromhex(h) for h in DAS_LANES_HEX],
}
NS = 1_000_000_000


def hud(**kw):
  args = dict(enabled=True, fcw=False, steer_required=False, audible=False, human_steering=False,
              left_lane_depart=False, right_lane_depart=False, left_blinker=False, right_blinker=False,
              curvature=0.0, ic_integration=True, model_path=None)
  args.update(kw)
  return c.HudInputs(**args)


def stock(addr, i=0):
  return c.unpack(addr, STOCK[addr][i])


def all_stock(i=0):
  return {a: stock(a, i) for a in c.CLUSTER_ADDRS}


# --- DBC, checksum, rebuild -------------------------------------------------

def test_signals_come_from_this_trees_dbc():
  assert c.MSG_NAMES == {0x399: "AutopilotStatus", 0x389: "DAS_status2", 0x239: "DAS_lanes"}
  assert "autopilotStatus" in c.SIGNALS[0x399]

  def geom(addr, name):
    sig = c.SIGNALS[addr][name]
    return (sig.start, sig.size)
  assert geom(0x399, "DAS_autopilotHandsOnState") == (42, 4)
  assert geom(0x399, "DAS_autoLaneChangeState") == (46, 5)
  assert geom(0x389, "DAS_activationFailureStatus") == (14, 2)
  c2 = c.SIGNALS[0x239]["DAS_virtualLaneC2"]
  assert (c2.start, c2.size, c2.factor, c2.offset) == (32, 8, 2e-05, -0.0025)
  for addr, name in c.COUNTER_SIGNALS.items():
    assert geom(addr, name) == (60 if addr == c.DAS_LANES else 52, 4)
  for addr, name in c.CHECKSUM_SIGNALS.items():
    assert geom(addr, name) == (56, 8)
  assert c.DAS_LANES not in c.CHECKSUM_SIGNALS


def test_checksum_matches_real_stock_frames():
  for addr in (c.AUTOPILOT_STATUS, c.DAS_STATUS2):
    for dat in STOCK[addr]:
      assert c.tesla_checksum(addr, dat) == dat[7], (hex(addr), dat.hex())


def test_rebuild_from_decoded_values_is_lossless_on_real_frames():
  for addr, frames in STOCK.items():
    for dat in frames:
      v = c.unpack(addr, dat)
      word = c.pack(addr, c.SIGNALS[addr], v)
      assert word.to_bytes(8, "little") == dat
      assert c.finish(addr, word, int(v[c.COUNTER_SIGNALS[addr]])) == dat


def test_body_controls_not_rebuildable_from_dbc():
  sigs = c.load_signals(addrs=(0x3e9,))[0x3e9]
  covered = 0
  for s in sigs.values():
    covered |= ((1 << s.size) - 1) << s.start
  assert DAS_BODY_CONTROLS_UNCOVERED_BITS_SEEN & ~covered == DAS_BODY_CONTROLS_UNCOVERED_BITS_SEEN
  assert 0x3e9 not in c.CLUSTER_ADDRS


def test_unknown_signal_name_is_an_error():
  with pytest.raises(KeyError):
    c.pack(c.DAS_LANES, c.SIGNALS[c.DAS_LANES], {"DAS_invented": 1})


# --- engaged ----------------------------------------------------------------

def _send(ctrl, h, new, t):
  return dict(ctrl.update(h, new, t))


def test_engaged_frames_counter_checksum_and_stock_fields():
  ctrl = c.ClusterController()
  out = _send(ctrl, hud(), all_stock(), 10 * NS)
  assert set(out) == set(c.CLUSTER_ADDRS)
  for addr, dat in out.items():
    s = stock(addr)
    v = c.unpack(addr, dat)
    ctr = c.COUNTER_SIGNALS[addr]
    assert int(v[ctr]) == (int(s[ctr]) + 1) % 16
    if addr in c.CHECKSUM_SIGNALS:
      assert dat[7] == c.tesla_checksum(addr, dat)
  ap = c.unpack(0x399, out[0x399])
  s = stock(0x399)
  for keep in ("DAS_blindSpotRearLeft", "DAS_blindSpotRearRight", "DAS_fusedSpeedLimit",
               "DAS_visionOnlySpeedLimit", "DAS_suppressSpeedWarning", "DAS_fleetSpeedState"):
    assert ap[keep] == s[keep], keep
  st2 = c.unpack(0x389, out[0x389])
  s2 = stock(0x389)
  for keep in ("DAS_accSpeedLimit", "DAS_ACC_report", "DAS_robState", "DAS_ppOffsetDesiredRamp",
               "DAS_pmmSysFaultReason", "DAS_radarTelemetry"):
    assert st2[keep] == s2[keep], keep


def test_counter_wraps():
  s = stock(0x399)
  s["DAS_statusCounter"] = 15
  out = _send(c.ClusterController(), hud(), {0x399: s}, NS)
  assert int(c.unpack(0x399, out[0x399])["DAS_statusCounter"]) == 0


def test_engaged_autopilot_state_and_hands_on():
  ctrl = c.ClusterController()
  ap = c.unpack(0x399, _send(ctrl, hud(), all_stock(), NS)[0x399])
  assert ap["autopilotStatus"] == 5
  assert ap["DAS_autopilotHandsOnState"] == 2
  cases = [
    (dict(steer_required=True), 3),
    (dict(steer_required=True, audible=True), 5),
    (dict(human_steering=True), 3),
    (dict(steer_required=True, audible=True, human_steering=True), 3),
  ]
  for kw, want in cases:
    ap = c.unpack(0x399, _send(ctrl, hud(**kw), all_stock(), NS)[0x399])
    assert ap["DAS_autopilotHandsOnState"] == want, kw


def test_stock_active_state_is_left_alone():
  s = stock(0x399)
  s["autopilotStatus"] = 3
  ap = c.unpack(0x399, _send(c.ClusterController(), hud(), {0x399: s}, NS)[0x399])
  assert ap["autopilotStatus"] == 3


def test_alc_state_from_stock_lane_bits_and_op_lane_change():
  def alc(left, right, **kw):
    lanes = stock(0x239)
    lanes["DAS_leftLaneExists"], lanes["DAS_rightLaneExists"] = left, right
    out = _send(c.ClusterController(), hud(**kw), {0x239: lanes, 0x399: stock(0x399)}, NS)
    return c.unpack(0x399, out[0x399])["DAS_autoLaneChangeState"]
  assert alc(1, 1) == 8
  assert alc(1, 0) == 6
  assert alc(0, 1) == 7
  assert alc(0, 0) == 1
  assert alc(1, 1, left_blinker=True) == 9
  assert alc(1, 1, right_blinker=True) == 10


def test_alc_uses_last_lanes_frame_when_none_new_this_step():
  ctrl = c.ClusterController()
  lanes = stock(0x239)
  lanes["DAS_leftLaneExists"], lanes["DAS_rightLaneExists"] = 1, 0
  _send(ctrl, hud(), {0x239: lanes}, NS)
  out = _send(ctrl, hud(), {0x399: stock(0x399)}, NS + 10_000_000)
  assert c.unpack(0x399, out[0x399])["DAS_autoLaneChangeState"] == 6


def test_op_warnings_add_and_never_clear_stock_warnings():
  s = stock(0x399)
  s["DAS_forwardCollisionWarning"] = 1
  s["DAS_laneDepartureWarning"] = 3
  s2 = stock(0x389)
  s2["DAS_longCollisionWarning"] = 6
  out = _send(c.ClusterController(), hud(), {0x399: s, 0x389: s2}, NS)
  ap, st2 = c.unpack(0x399, out[0x399]), c.unpack(0x389, out[0x389])
  assert ap["DAS_forwardCollisionWarning"] == 1
  assert ap["DAS_laneDepartureWarning"] == 3
  assert st2["DAS_longCollisionWarning"] == 6

  out = _send(c.ClusterController(), hud(fcw=True, right_lane_depart=True), all_stock(), NS)
  ap, st2 = c.unpack(0x399, out[0x399]), c.unpack(0x389, out[0x389])
  assert ap["DAS_forwardCollisionWarning"] == 1
  assert ap["DAS_laneDepartureWarning"] == 2
  assert st2["DAS_longCollisionWarning"] == 1
  ap = c.unpack(0x399, _send(c.ClusterController(), hud(left_lane_depart=True), all_stock(), NS)[0x399])
  assert ap["DAS_laneDepartureWarning"] == 1


def test_engaged_status2_fields_and_csa_bits():
  s2 = stock(0x389)
  s2["DAS_activationFailureStatus"] = 2
  s2["DAS_driverInteractionLevel"] = 1
  dat = _send(c.ClusterController(), hud(), {0x389: s2}, NS)[0x389]
  v = c.unpack(0x389, dat)
  assert v["DAS_activationFailureStatus"] == 0
  assert v["DAS_driverInteractionLevel"] == 0
  word = int.from_bytes(dat, "little")
  assert c.get_raw(word, 32, 2) == c.CSA_ENABLE
  # Bit 31 (Tinkla DAS_relaxCruiseLimits) and bits 34+ stay stock.
  stock_word = int.from_bytes(STOCK[0x389][0], "little")
  assert c.get_raw(word, 31, 1) == c.get_raw(stock_word, 31, 1)
  assert c.get_raw(word, 34, 14) == c.get_raw(stock_word, 34, 14)


def test_lanes_path_from_curvature():
  ctrl = c.ClusterController()
  for k, want in ((0.0, 0.0), (0.0002, 0.0004), (-0.0002, -0.0004), (0.01, 0.0025), (-0.01, -0.0025)):
    v = c.unpack(0x239, _send(ctrl, hud(curvature=k), {0x239: stock(0x239)}, NS)[0x239])
    assert v["DAS_virtualLaneC2"] == pytest.approx(want, abs=2e-05), k
    assert v["DAS_virtualLaneC0"] == pytest.approx(0.0, abs=0.035)
    assert v["DAS_virtualLaneC1"] == pytest.approx(0.0, abs=0.0016)
    assert v["DAS_virtualLaneC3"] == pytest.approx(0.0, abs=2.4e-07)
    assert v["DAS_virtualLaneViewRange"] == 50
  lanes = stock(0x239)
  lanes["DAS_leftLaneExists"], lanes["DAS_rightLaneExists"] = 1, 0
  lanes["DAS_leftLineUsage"], lanes["DAS_rightLineUsage"] = 2, 0
  v = c.unpack(0x239, _send(ctrl, hud(), {0x239: lanes}, NS)[0x239])
  assert (v["DAS_leftLineUsage"], v["DAS_rightLineUsage"]) == (2, 0)
  assert (v["DAS_leftLaneExists"], v["DAS_rightLaneExists"]) == (1, 0)
  assert v["DAS_virtualLaneWidth"] == lanes["DAS_virtualLaneWidth"]
  assert v["DAS_leftFork"] == lanes["DAS_leftFork"]


def test_lanes_preserves_stock_usage_when_exists_zero():
  """AP1 stock: Exists often 0 while LineUsage is 2. Do not wipe usage."""
  lanes = stock(0x239)
  lanes["DAS_leftLaneExists"], lanes["DAS_rightLaneExists"] = 0, 0
  lanes["DAS_leftLineUsage"], lanes["DAS_rightLineUsage"] = 2, 2
  v = c.unpack(0x239, _send(c.ClusterController(), hud(), {0x239: lanes}, NS)[0x239])
  assert (v["DAS_leftLaneExists"], v["DAS_rightLaneExists"]) == (0, 0)
  assert (v["DAS_leftLineUsage"], v["DAS_rightLineUsage"]) == (2, 2)
  assert v["DAS_virtualLaneViewRange"] == 50


# --- when nothing is sent ---------------------------------------------------

def test_never_engaged_sends_nothing():
  ctrl = c.ClusterController()
  for i in range(5):
    assert ctrl.update(hud(enabled=False), all_stock(i), i * NS) == []


def test_no_stock_frame_no_send():
  ctrl = c.ClusterController()
  assert ctrl.update(hud(), {}, NS) == []
  out = _send(ctrl, hud(), {0x389: stock(0x389)}, 2 * NS)
  assert set(out) == {0x389}


def test_post_disengage_window():
  ctrl = c.ClusterController()
  _send(ctrl, hud(), all_stock(), 0)
  t0 = 1 * NS
  s = stock(0x399)
  s["autopilotStatus"] = 1
  s2 = stock(0x389)
  s2["DAS_activationFailureStatus"] = 1
  new = {0x399: s, 0x389: s2, 0x239: stock(0x239)}
  out = _send(ctrl, hud(enabled=False), new, t0)
  assert set(out) == {0x399, 0x389}  # lanes are engaged only
  ap = c.unpack(0x399, out[0x399])
  assert ap["autopilotStatus"] == 2
  # Post rewrite touches only the state; everything else is stock.
  assert ap["DAS_autopilotHandsOnState"] == s["DAS_autopilotHandsOnState"]
  assert ap["DAS_autoLaneChangeState"] == s["DAS_autoLaneChangeState"]
  st2 = c.unpack(0x389, out[0x389])
  assert st2["DAS_activationFailureStatus"] == 0
  stock_word = int.from_bytes(c.finish(0x389, c.pack(0x389, c.SIGNALS[0x389], s2), 0), "little")
  assert c.get_raw(int.from_bytes(out[0x389], "little"), 32, 2) == c.get_raw(stock_word, 32, 2)

  # Stock states other than UNAVAILABLE are not rewritten.
  for state in (0, 2, 3, 15):
    s3 = dict(s, autopilotStatus=state)
    ap = c.unpack(0x399, _send(ctrl, hud(enabled=False), {0x399: s3}, t0 + NS)[0x399])
    assert ap["autopilotStatus"] == state

  assert _send(ctrl, hud(enabled=False), new, t0 + c.POST_DISENGAGE_NS - 1)
  assert ctrl.update(hud(enabled=False), new, t0 + c.POST_DISENGAGE_NS) == []
  assert ctrl.update(hud(enabled=False), new, t0 + 10 * NS) == []


def test_reengage_inside_window_is_engaged():
  ctrl = c.ClusterController()
  _send(ctrl, hud(), all_stock(), 0)
  _send(ctrl, hud(enabled=False), all_stock(), NS)
  out = _send(ctrl, hud(), all_stock(), 2 * NS)
  assert set(out) == set(c.CLUSTER_ADDRS)
  assert c.unpack(0x399, out[0x399])["autopilotStatus"] == 5


def test_post_window_never_shows_active_state():
  ctrl = c.ClusterController()
  _send(ctrl, hud(), all_stock(), 0)
  for i, dat in enumerate(STOCK[0x399]):
    v = c.unpack(0x399, dat)
    if int(v["autopilotStatus"]) in (3, 4, 5):
      continue
    out = _send(ctrl, hud(enabled=False), {0x399: v}, NS + i)
    assert int(c.unpack(0x399, out[0x399])["autopilotStatus"]) not in (3, 4, 5)


def test_toggle_off_sends_nothing_even_when_engaged():
  ctrl = c.ClusterController()
  assert ctrl.update(hud(ic_integration=False), all_stock(), NS) == []
  # Re-enable works after off.
  out = _send(ctrl, hud(ic_integration=True), all_stock(), 2 * NS)
  assert set(out) == set(c.CLUSTER_ADDRS)


def test_toggle_off_clears_post_disengage_window():
  ctrl = c.ClusterController()
  _send(ctrl, hud(), all_stock(), 0)
  # Enter post window
  _send(ctrl, hud(enabled=False), all_stock(), NS)
  # Toggle off mid-window: nothing, and state resets.
  assert ctrl.update(hud(enabled=False, ic_integration=False), all_stock(), 2 * NS) == []
  # Still disabled with toggle back on but never re-engaged: no post rewrite
  # because toggle-off cleared disengaged_ns.
  assert ctrl.update(hud(enabled=False, ic_integration=True), all_stock(), 3 * NS) == []


def test_lanes_use_model_path_when_present():
  path = c.ModelPath(c0=0.35, c1=-0.05, c2=0.0012, c3=0.0, view_range_m=72)
  v = c.unpack(0x239, _send(c.ClusterController(), hud(model_path=path, curvature=0.01),
                            {0x239: stock(0x239)}, NS)[0x239])
  assert v["DAS_virtualLaneC0"] == pytest.approx(0.0, abs=0.035)  # path starts at the car
  assert v["DAS_virtualLaneC1"] == pytest.approx(0.0, abs=0.0016)  # Tinkla: C1 suppressed
  assert v["DAS_virtualLaneC2"] == pytest.approx(0.0012, abs=2e-05)
  assert v["DAS_virtualLaneC3"] == pytest.approx(0.0, abs=2.4e-07)
  assert v["DAS_virtualLaneViewRange"] == 72


def test_view_range_clamped_to_dbc():
  for raw, want in ((-5, 0), (0, 0), (50, 50), (160, 160), (200, 160), (72.4, 72)):
    assert c.clamp_view_range_m(raw) == want
  path = c.ModelPath(c0=0, c1=0, c2=0, c3=0, view_range_m=999)
  v = c.unpack(0x239, _send(c.ClusterController(), hud(model_path=path),
                            {0x239: stock(0x239)}, NS)[0x239])
  assert v["DAS_virtualLaneViewRange"] == 160


def test_model_path_missing_falls_back_to_curvature_and_50m():
  v = c.unpack(0x239, _send(c.ClusterController(), hud(model_path=None, curvature=0.0002),
                            {0x239: stock(0x239)}, NS)[0x239])
  assert v["DAS_virtualLaneC0"] == pytest.approx(0.0, abs=0.035)
  assert v["DAS_virtualLaneC1"] == pytest.approx(0.0, abs=0.0016)
  assert v["DAS_virtualLaneC2"] == pytest.approx(0.0004, abs=2e-05)
  assert v["DAS_virtualLaneViewRange"] == 50


def test_path_from_model_v2_fits_poly_and_scales():
  # Curved path from the car: y = 0.0001 x^2, out to 60 m.
  xs = [float(i) for i in range(1, 61)]
  ys = [0.0001 * x * x for x in xs]
  model = SimpleNamespace(position=SimpleNamespace(x=xs, y=ys))
  path = c.path_from_model_v2(model)
  assert path is not None
  f = 1.0 / c.IC_LANE_SCALE
  assert path.c0 == 0.0
  assert path.c1 == 0.0  # Tinkla suppress_x_coord
  assert path.c2 == pytest.approx(0.0001 * f * f, abs=2e-5)
  assert path.c3 == 0.0
  assert path.view_range_m == 60


def test_path_from_model_v2_heading_does_not_tilt_or_offset_line():
  # A slanted model path (979dbf8 regression) must not produce C0/C1.
  xs = [float(i) for i in range(1, 101)]
  ys = [0.3 + 0.05 * x for x in xs]
  path = c.path_from_model_v2(SimpleNamespace(position=SimpleNamespace(x=xs, y=ys)))
  assert path is not None
  assert (path.c0, path.c1, path.c3) == (0.0, 0.0, 0.0)
  assert path.view_range_m == 100  # Tinkla max_distance cap
  path = c.ModelPath(c0=1.5, c1=0.2, c2=0.0, c3=0.0, view_range_m=60)
  v = c.unpack(0x239, _send(c.ClusterController(), hud(model_path=path),
                            {0x239: stock(0x239)}, NS)[0x239])
  assert v["DAS_virtualLaneC0"] == pytest.approx(0.0, abs=0.035)
  assert v["DAS_virtualLaneC1"] == pytest.approx(0.0, abs=0.0016)


def test_path_from_model_v2_rejects_short_or_empty():
  assert c.path_from_model_v2(SimpleNamespace(position=SimpleNamespace(x=[], y=[]))) is None
  assert c.path_from_model_v2(SimpleNamespace(position=SimpleNamespace(x=[1, 2], y=[0, 0]))) is None
  # xmax < MODEL_PATH_MIN_M
  xs = [0.5, 1.0, 2.0, 3.0]
  ys = [0.0, 0.0, 0.0, 0.0]
  assert c.path_from_model_v2(SimpleNamespace(position=SimpleNamespace(x=xs, y=ys))) is None


def test_stock_line_usage_preserved_with_model_path():
  lanes = stock(0x239)
  lanes["DAS_leftLaneExists"], lanes["DAS_rightLaneExists"] = 0, 0
  lanes["DAS_leftLineUsage"], lanes["DAS_rightLineUsage"] = 2, 2
  path = c.ModelPath(c0=0.1, c1=0.0, c2=0.0005, c3=0.0, view_range_m=40)
  v = c.unpack(0x239, _send(c.ClusterController(), hud(model_path=path), {0x239: lanes}, NS)[0x239])
  assert (v["DAS_leftLineUsage"], v["DAS_rightLineUsage"]) == (2, 2)
  assert (v["DAS_leftLaneExists"], v["DAS_rightLaneExists"]) == (0, 0)


def test_path_from_xy_matches_model_v2_reader():
  xs = [float(i) for i in range(1, 61)]
  ys = [0.0001 * x * x for x in xs]
  assert c.path_from_xy(xs, ys) == c.path_from_model_v2(SimpleNamespace(position=SimpleNamespace(x=xs, y=ys)))
  assert c.path_from_xy([], []) is None
  assert c.path_from_xy(None, None) is None
