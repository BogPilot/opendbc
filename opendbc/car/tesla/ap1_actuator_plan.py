"""AP1 actuator decision for TESLA_AP1_MODELS.

Angle steps use the Tinkla AP1 lookup, not Model 3/Y limits.

Actuator decision for Tesla CarController.

CarController.update packs exactly the messages this plan names. No CAN
packing happens here. While disengaged, the plan has no steering frame so
stock Mobileye 0x488 can keep flowing (AP1 interceptor). While AP1 cruise is
enabled but path lateral is not, 0x488 is still sent: the measured angle and
control type NONE. That is Tinkla's human-control and non-idle EPAS path,
except EAC_ERROR_HIGH_ANGLE_REQ (6) and a latched EAC_ERROR_HANDS_ON (3).
Those two are not path blocks once hands are below the HSO threshold. While
enabled, hands are below that threshold, and EPAS is not EAC_FAULT, 0x488 is
ANGLE: the planned angle when lat is active, otherwise the measured angle,
rate limited and clipped to the measured wheel +/- 20 deg. DAS_control is planned
only when the car params allow openpilot
longitudinal control and CarControl.enabled and CarControl.longActive are
both true. Otherwise the plan has no longitudinal command.
"""

from dataclasses import dataclass

from opendbc.car.tesla.ap1_hso import AP1_HANDS_ON_LEVEL
from opendbc.car.tesla.ap1_steer_fault import AP1_LATCHED_ANGLE_ERRORS

# Tinkla earlytesla-panda TESLA_LOOKUP_ANGLE_RATE_UP / _DOWN.
# Not sunnypilot Model 3/Y AngleSteeringLimitsVM (5 deg / 20 ms frame).
# Not BogPilot's Python CarControllerParams shared table (0/5/15 m/s).
# Below the first speed breakpoint the first rate is used.
_AP1_RATE_UP_BP = (2.0, 7.0, 17.0)
_AP1_RATE_UP_V = (8.0, 4.0, 2.5)
_AP1_RATE_DOWN_BP = (2.0, 7.0, 17.0)
_AP1_RATE_DOWN_V = (9.0, 5.0, 4.5)
_ACCEL_TO_SPEED_MULTIPLIER = 3


def _clip(val, lo, hi):
  return lo if val < lo else hi if val > hi else val


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(len(xp) - 1):
    if xp[i] <= x <= xp[i + 1]:
      span = xp[i + 1] - xp[i]
      t = (x - xp[i]) / span
      return fp[i] + t * (fp[i + 1] - fp[i])
  return fp[-1]


def _ap1_rate_limit(apply_angle, apply_angle_last, v_ego):
  steer_up = apply_angle_last * apply_angle >= 0. and abs(apply_angle) > abs(apply_angle_last)
  bp = _AP1_RATE_UP_BP if steer_up else _AP1_RATE_DOWN_BP
  rates = _AP1_RATE_UP_V if steer_up else _AP1_RATE_DOWN_V
  step = _interp(v_ego, bp, rates)
  return _clip(apply_angle, apply_angle_last - step, apply_angle_last + step)

# safety_tesla.h tesla_tx_hook: type 0 is NONE and type 3 is DISABLED.
# Neither one sets steer_control_enabled.
STEERING_CONTROL_NONE = 0
STEERING_CONTROL_ANGLE = 1

# 11-bit DAS_control. Chassis is 0x2b9 (tesla_can.dbc). Powertrain is 0x2bf
# (tesla_powertrain.dbc). AP1 plans the chassis id only. tesla_tx_hook uses
# 0x2b9 when TESLA_FLAG_POWERTRAIN is unset.
DAS_CONTROL_CHASSIS = 0x2B9
DAS_CONTROL_POWERTRAIN = 0x2BF

# tesla_can.dbc VAL_ 697 DAS_accState. Tinkla create_ap1_long_control uses
# 4 (ACC_ON) in drive and 3 (ACC_HOLD) only for static cruise.
ACC_HOLD = 3
ACC_ON = 4


@dataclass(frozen=True)
class SteerCommand:
  angle_deg: float
  enabled: bool
  counter: int

  @property
  def control_type(self) -> int:
    if self.enabled:
      return STEERING_CONTROL_ANGLE
    return STEERING_CONTROL_NONE


@dataclass(frozen=True)
class LongCommand:
  acc_state: int
  target_speed: float
  min_accel: float
  max_accel: float
  counter: int


@dataclass(frozen=True)
class ActuatorPlan:
  steer: SteerCommand | None
  longitudinal: tuple[LongCommand, ...]
  cancel: bool
  apply_angle_last: float
  # Empty unless a DAS_control command is planned. AP1 is (0x2b9,) only.
  longitudinal_addrs: tuple[int, ...] = ()


def steering_control_type(enabled: bool) -> int:
  if enabled:
    return STEERING_CONTROL_ANGLE
  return STEERING_CONTROL_NONE


def ap1_long_acc_state(camera_acc_state, chassis_das_only):
  """ACC state openpilot may put in chassis DAS_control.

  AP1 openpilot longitudinal is adaptive cruise in drive, not Tinkla static
  cruise. Tinkla selfdrive/car/tesla/teslacan.py create_ap1_long_control sets
  DAS_accState to 4 while in drive, and to 3 only when static_cruise and
  cruise are both on. Copying the camera ACC_HOLD (3) made the car wait for
  the accelerator at a stop. Non-AP1 still passes the camera state through.
  """
  if chassis_das_only and camera_acc_state == ACC_HOLD:
    return ACC_ON
  return camera_acc_state


def ap1_should_send_hold_clear(ap1, long_allowed, camera_acc_state, frame):
  """When to TX the all-zero 0x349 clear.

  Tinkla HUD_module sends warning matrix 3 at 1 Hz with DAS_gas_to_resume
  forced to 0. Also send while the camera is still reporting ACC_HOLD so the
  clear is on the bus for that stop, not only on the 1 Hz tick.
  Disengaged longitudinal does not send it.
  """
  if not ap1 or not long_allowed:
    return False
  return camera_acc_state == ACC_HOLD or (frame % 100) == 0


def longitudinal_command_allowed(openpilot_longitudinal_control, enabled, long_active):
  """True only when a DAS_control command may be planned.

  CarControl.enabled must be true for actuator commands. longActive is the
  longitudinal engage bit from controlsd. The openpilot-long car param is
  required as well. Any false input means no DAS_control frame.
  """
  return bool(openpilot_longitudinal_control) and bool(enabled) and bool(long_active)


def _ap1_limited_angle(requested_angle_deg, last_angle_deg, measured_angle_deg, v_ego):
  """Rate limit, then Tinkla's +/- 20 deg clip around the measured wheel.

  The clip is the "to not fault the EPS" bound. It is applied after the
  speed-based rate limit. A requested angle inside that window is unchanged
  aside from the rate step.
  """
  apply_angle = _ap1_rate_limit(requested_angle_deg, last_angle_deg, v_ego)
  return _clip(apply_angle, measured_angle_deg - 20, measured_angle_deg + 20)


def build_actuator_plan(frame, lat_active, hands_on_fault, openpilot_longitudinal_control,
                        enabled, long_active,
                        measured_angle_deg, requested_angle_deg, last_angle_deg, v_ego, accel,
                        acc_state, das_counters, pcm_cancel, chassis_das_only=False,
                        epas_error=None, eac_fault=False, hands_on_level=0):
  """Same steering branches CarController.update had before this function existed.

  Disengaged lateral: no steering frame. Sending type-NONE 0x488 while
  disengaged replaced stock Mobileye commands on AP1. AP1 with cruise enabled
  and lat inactive still sends 0x488 at the measured angle with type NONE,
  except error code 6 and a latched hands-on code 3. Those do not force NONE
  once hands are below 2. While enabled, hands are below 2, and EPAS is not
  EAC_FAULT, code 6 and code 3 send ANGLE: the planned angle when lat is
  active, otherwise the measured angle, rate limited and clipped to measured
  +/- 20 deg. Other non-idle codes still take the NONE branch. Hands at or
  above 2 also take NONE and do not cancel, including a live code 3.
  apply_angle_last
  tracks the commanded angle so the next path command starts from the wheel.
  Longitudinal messages are built only when longitudinal_command_allowed is
  true. An inactive long plan is empty.
  """

  lkas_enabled = lat_active and not hands_on_fault
  apply_angle_last = last_angle_deg
  steer = None
  # AP1 only. Hands pause and EAC_FAULT still win over a latched code 6 or 3.
  # Code 3 takes this path only after hands drop; at or above 2, hands_pause
  # keeps type NONE even though code 3 is not itself a temporary fault.
  hands_pause = bool(chassis_das_only) and hands_on_level >= AP1_HANDS_ON_LEVEL
  latched_angle = (
    bool(chassis_das_only)
    and bool(enabled)
    and not hands_on_fault
    and not hands_pause
    and not eac_fault
    and epas_error in AP1_LATCHED_ANGLE_ERRORS
  )
  if chassis_das_only and eac_fault:
    lkas_enabled = False

  if frame % 2 == 0:
    if lkas_enabled and not hands_pause:
      # Angular rate limit based on speed, then the EPS clip.
      apply_angle = _ap1_limited_angle(requested_angle_deg, last_angle_deg, measured_angle_deg, v_ego)
      apply_angle_last = apply_angle
      steer = SteerCommand(apply_angle, True, (frame // 2) % 16)
    elif latched_angle:
      # Planner angle is not active. Command the measured wheel as ANGLE
      # so EPAS can leave a latched code 6 or code 3. Still rate limited
      # and clipped.
      apply_angle = _ap1_limited_angle(measured_angle_deg, last_angle_deg, measured_angle_deg, v_ego)
      apply_angle_last = apply_angle
      steer = SteerCommand(apply_angle, True, (frame // 2) % 16)
    elif chassis_das_only and enabled and not hands_on_fault:
      # Cruise stays up. Do not send the planned path angle.
      apply_angle_last = measured_angle_deg
      steer = SteerCommand(measured_angle_deg, False, (frame // 2) % 16)
    else:
      # Interceptor: do not TX 0x488 while disengaged; stock DAS must pass.
      apply_angle_last = measured_angle_deg

  longitudinal = []
  longitudinal_addrs = ()
  if longitudinal_command_allowed(openpilot_longitudinal_control, enabled, long_active):
    target_accel = accel
    target_speed = max(v_ego + (target_accel * _ACCEL_TO_SPEED_MULTIPLIER), 0)
    max_accel = 0 if target_accel < 0 else target_accel
    min_accel = 0 if target_accel > 0 else target_accel

    acc_state = ap1_long_acc_state(acc_state, chassis_das_only)
    while len(das_counters) > 0:
      longitudinal.append(LongCommand(acc_state, target_speed, min_accel, max_accel, das_counters.popleft()))
    if longitudinal:
      if chassis_das_only:
        longitudinal_addrs = (DAS_CONTROL_CHASSIS,)
      else:
        longitudinal_addrs = (DAS_CONTROL_CHASSIS, DAS_CONTROL_POWERTRAIN)

  # Cancel on user steering override, since there is no steering torque blending
  if hands_on_fault:
    pcm_cancel = True

  cancel = frame % 10 == 0 and pcm_cancel
  return ActuatorPlan(steer, tuple(longitudinal), bool(cancel), apply_angle_last, longitudinal_addrs)
