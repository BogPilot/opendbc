"""AP1 behavior from BogPilot ap1-driving-milestone-2 (92e84996), sunnypilot layout.

Ports the decision tests of BogPilot selfdrive/car/tesla/tests/test_ap1_resume_hold.py,
test_ap1_driver_override.py and test_ap1_hso.py, and runs the AP1 CarInterface with the
real opendbc CANParser / CANPacker for the wiring (steeringPressed level, resume hold,
soft-start, EPAS inhibit warning, stalk personality, cluster substitution).

Unit tests only. Not a drive test, not road-tested.
"""

from collections import deque

import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla import ap1_cluster as c
from opendbc.car.tesla.ap1_actuator_plan import (
  ACC_ON,
  AP1_ENGAGE_SOFT_START_FRAMES,
  EAC_ACTIVE,
  STEERING_CONTROL_ANGLE,
  STEERING_CONTROL_NONE,
  ap1_hold_measured_angle,
  build_actuator_plan,
)
from opendbc.car.tesla.ap1_hso import (
  AP1_CONTROL_HZ,
  AP1_DRIVER_INPUT_LEVEL,
  AP1_HANDS_ON_LEVEL,
  AP1_INHIBIT_ALERT_FRAMES,
  AP1_RESUME_HOLD_FRAMES,
  AP1_RESUME_HOLD_S,
  Ap1DriverYield,
  Ap1EpasInhibitAlert,
  ap1_driver_input,
  ap1_lat_active,
  ap1_steering_pressed,
)
from opendbc.car.tesla.ap1_stalk_follow import ap1_stalk_personality, map_stalk_follow
from opendbc.car.tesla.tests.fixtures.ap1_cluster_frames import AUTOPILOT_STATUS_HEX, DAS_LANES_HEX, DAS_STATUS2_HEX
from opendbc.car.tesla.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

DT_NS = 10_000_000  # 100 Hz
EAC = {"EAC_INHIBITED": 0, "EAC_AVAILABLE": 1, "EAC_ACTIVE": 2, "EAC_FAULT": 3}
EPAS_ERR = {"EAC_ERROR_IDLE": 0, "EAC_ERROR_HANDS_ON": 3, "EAC_ERROR_HIGH_ANGLE_REQ": 6, "EAC_ERROR_TMP_FAULT": 4}


# --- constants and pure decisions -------------------------------------------

def test_thresholds_and_hold_constant():
  assert AP1_HANDS_ON_LEVEL == 2
  assert AP1_DRIVER_INPUT_LEVEL == 1
  assert AP1_RESUME_HOLD_S == 0.5
  assert AP1_CONTROL_HZ == 100
  assert AP1_RESUME_HOLD_FRAMES == 50
  assert AP1_ENGAGE_SOFT_START_FRAMES == 30
  assert AP1_INHIBIT_ALERT_FRAMES == 100


def test_level_1_is_driver_input_but_not_a_pause():
  assert [ap1_driver_input(lv) for lv in range(4)] == [False, True, True, True]
  assert [ap1_steering_pressed(lv) for lv in range(4)] == [False, False, True, True]
  assert ap1_lat_active(True, 1) is True
  assert ap1_lat_active(True, 2) is False
  assert ap1_lat_active(False, 0) is False


def _run_yield(y, enabled, level, n):
  return [y.update(enabled, level) for _ in range(n)]


def test_yield_hold_not_elapsed_stays_yielded():
  y = Ap1DriverYield()
  assert _run_yield(y, True, 3, 5) == [True] * 5
  assert all(_run_yield(y, True, 1, 300))  # easing off at level 1: no countdown
  assert all(_run_yield(y, True, 0, AP1_RESUME_HOLD_FRAMES - 1))
  assert y.resumed is False


def test_yield_re_press_restarts_timer():
  for re_press in (1, 3):
    y = Ap1DriverYield()
    _run_yield(y, True, 3, 3)
    _run_yield(y, True, 0, AP1_RESUME_HOLD_FRAMES - 1)
    assert y.update(True, re_press) is True
    assert all(_run_yield(y, True, 0, AP1_RESUME_HOLD_FRAMES - 1))
    assert y.update(True, 0) is False
    assert y.resumed is True


def test_yield_elapsed_resumes_once():
  y = Ap1DriverYield()
  _run_yield(y, True, 3, 3)
  states = _run_yield(y, True, 0, AP1_RESUME_HOLD_FRAMES)
  assert states[:-1] == [True] * (AP1_RESUME_HOLD_FRAMES - 1)
  assert states[-1] is False and y.resumed is True
  assert y.update(True, 0) is False and y.resumed is False


def test_yield_level_1_alone_never_starts_a_hold():
  assert not any(_run_yield(Ap1DriverYield(), True, 1, 200))


def test_yield_disengaged_never_and_disengage_clears_at_once():
  y = Ap1DriverYield()
  assert not any(_run_yield(y, False, 3, 50))
  _run_yield(y, True, 3, 3)
  assert y.update(False, 0) is False
  assert y.update(True, 0) is False  # re-engage with hands off: no leftover hold


def _plan(**kwargs):
  args = dict(
    frame=0, lat_active=True, hands_on_fault=False, openpilot_longitudinal_control=True,
    enabled=True, long_active=True, measured_angle_deg=6.0, requested_angle_deg=30.0,
    last_angle_deg=6.0, v_ego=15.0, accel=0.2, acc_state=ACC_ON, das_counters=deque([3]),
    pcm_cancel=False, chassis_das_only=True, eac_status=EAC_ACTIVE, hands_on_level=0,
  )
  args.update(kwargs)
  return build_actuator_plan(**args)


def test_plan_active_epas_uses_planner_angle():
  plan = _plan()
  assert plan.steer.control_type == STEERING_CONTROL_ANGLE
  assert plan.steer.angle_deg > 6.0


def test_plan_measured_angle_while_not_active_soft_start_or_latched():
  for kw in (dict(eac_status="EAC_INHIBITED"), dict(eac_status="EAC_AVAILABLE"), dict(soft_start=True),
             dict(epas_error="EAC_ERROR_HIGH_ANGLE_REQ"), dict(epas_error="EAC_ERROR_HANDS_ON")):
    plan = _plan(**kw)
    assert plan.steer.control_type == STEERING_CONTROL_ANGLE, kw
    assert plan.steer.angle_deg == 6.0, kw
  # Also when path lateral is off (the inhibit recovery does not need latActive)
  plan = _plan(lat_active=False, eac_status="EAC_INHIBITED")
  assert plan.steer.control_type == STEERING_CONTROL_ANGLE and plan.steer.angle_deg == 6.0


def test_plan_hands_pause_and_yield_win_over_recovery():
  for kw in (dict(hands_on_level=2), dict(hands_on_level=3), dict(driver_yield=True)):
    for extra in ({}, dict(eac_status="EAC_AVAILABLE"), dict(soft_start=True), dict(epas_error="EAC_ERROR_HANDS_ON")):
      plan = _plan(**kw, **extra)
      assert plan.steer.control_type == STEERING_CONTROL_NONE, (kw, extra)
      assert plan.steer.angle_deg == 6.0
      assert plan.cancel is False
      assert len(plan.longitudinal) == 1  # cruise stays up


def test_plan_eac_fault_never_angle():
  plan = _plan(eac_fault=True, soft_start=True, eac_status="EAC_FAULT")
  assert plan.steer.control_type == STEERING_CONTROL_NONE


def test_plan_disengaged_sends_no_steering_frame():
  for kw in ({}, dict(driver_yield=True), dict(soft_start=True), dict(eac_status="EAC_INHIBITED")):
    plan = _plan(enabled=False, long_active=False, lat_active=False, **kw)
    assert plan.steer is None, kw
    assert plan.longitudinal == ()


def test_plan_lat_enabled_mads_lateral_only():
  # MADS lateral-only: CarControl.enabled False, lateral engaged. Steering gets the full AP1 handling,
  # longitudinal does not run.
  plan = _plan(enabled=False, long_active=False, lat_enabled=True, soft_start=True)
  assert plan.steer.control_type == STEERING_CONTROL_ANGLE and plan.steer.angle_deg == 6.0
  assert plan.longitudinal == ()
  plan = _plan(enabled=False, long_active=False, lat_enabled=True, driver_yield=True)
  assert plan.steer.control_type == STEERING_CONTROL_NONE
  plan = _plan(enabled=True, lat_active=False, lat_enabled=False, eac_status="EAC_INHIBITED")
  assert plan.steer is None


def test_hold_measured_gate():
  assert not ap1_hold_measured_angle(False, True, False, False, "EAC_INHIBITED", None, True)
  assert not ap1_hold_measured_angle(True, False, False, False, "EAC_INHIBITED", None, True)
  assert not ap1_hold_measured_angle(True, True, True, False, "EAC_INHIBITED", None, True)
  assert not ap1_hold_measured_angle(True, True, False, True, "EAC_INHIBITED", None, True)
  assert not ap1_hold_measured_angle(True, True, False, False, None, None, False)
  assert not ap1_hold_measured_angle(True, True, False, False, EAC_ACTIVE, "EAC_ERROR_TMP_FAULT", False)


def test_inhibit_alert_counts_only_when_openpilot_wants_lateral():
  a = Ap1EpasInhibitAlert()
  assert not any(a.update(True, 0, False, "EAC_INHIBITED", False) for _ in range(AP1_INHIBIT_ALERT_FRAMES - 1))
  assert a.update(True, 0, False, "EAC_INHIBITED", False)
  # Any break resets the count: active EPAS, hands pause (level-3 self-shutoff), resume hold, fault, no lat
  for args in ((True, 0, False, EAC_ACTIVE, False), (True, 3, False, "EAC_INHIBITED", False),
               (True, 0, True, "EAC_INHIBITED", False), (True, 0, False, "EAC_INHIBITED", True),
               (False, 0, False, "EAC_INHIBITED", False)):
    a = Ap1EpasInhibitAlert()
    for _ in range(AP1_INHIBIT_ALERT_FRAMES * 2):
      assert not a.update(*args), args


def test_stalk_detent_personality_map():
  raws = (0, 33, 66, 100, 133, 166, 200)
  want = (0, 0, 0, None, 1, None, 2)
  for raw, w in zip(raws, want, strict=True):
    assert ap1_stalk_personality(map_stalk_follow(raw)) == w, raw
  assert ap1_stalk_personality(None) is None
  assert ap1_stalk_personality(map_stalk_follow(255)) is None  # SNA with no history: not ready
  held = map_stalk_follow(255, map_stalk_follow(200))
  assert ap1_stalk_personality(held) == 2


# --- CarInterface wiring with the real parser / packer ----------------------

STOCK_CLUSTER = {
  c.AUTOPILOT_STATUS: bytes.fromhex(AUTOPILOT_STATUS_HEX[0]),
  c.DAS_STATUS2: bytes.fromhex(DAS_STATUS2_HEX[0]),
  c.DAS_LANES: bytes.fromhex(DAS_LANES_HEX[0]),
}


def make_ci(params_list=None):
  CI = interfaces[CAR.TESLA_AP1_MODELS]
  fp = gen_empty_fingerprint()
  CP = CI.get_params(CAR.TESLA_AP1_MODELS, fp, [], False, False, False)
  CP_SP = CI.get_params_sp(CP, CAR.TESLA_AP1_MODELS, fp, [], False, False, False)
  setup_interfaces(CI, CP, CP_SP, params_list or [])
  return CI(CP, CP_SP)


class Car:
  """Packs the AP1 chassis frames the AP1 CarState reads, with live counters."""

  def __init__(self):
    self.packer = CANPacker("tesla_can")
    self.step = 0
    self.t = 0
    self.hands = 0
    self.eac = "EAC_ACTIVE"
    self.err = "EAC_ERROR_IDLE"
    self.angle = 5.0
    self.dtr = 66
    self.cluster = False
    self.cluster_counter_jump = False

  def _msg(self, name, bus, values):
    msg = self.packer.dbc.name_to_msg[name]
    vals = dict(values)
    for sig in msg.sigs:
      if sig.endswith("Counter"):
        vals[sig] = self.step % (1 << msg.sigs[sig].size)
    return self.packer.make_can_msg(name, bus, vals)

  def frames(self):
    f = [
      self._msg("ESP_B", 0, {"ESP_vehicleSpeed": 60}),
      self._msg("DI_torque1", 0, {"DI_pedalPos": 0}),
      self._msg("DI_torque2", 0, {"DI_gear": 4}),
      self._msg("STW_ANGLHP_STAT", 0, {}),
      self._msg("EPAS_sysStatus", 0, {"EPAS_handsOnLevel": self.hands, "EPAS_eacStatus": EAC[self.eac],
                                      "EPAS_eacErrorCode": EPAS_ERR[self.err], "EPAS_internalSAS": -self.angle}),
      self._msg("DI_state", 0, {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 40}),
      self._msg("STW_ACTN_RQ", 0, {"DTR_Dist_Rq": self.dtr}),
      self._msg("GTW_carState", 0, {}),
      self._msg("BrakeMessage", 0, {"driverBrakeStatus": 1}),
      self._msg("SDM1", 0, {"SDM_bcklDrivStatus": 1}),
      self._msg("DAS_control", 2, {"DAS_accState": 4}),
    ]
    if self.cluster and self.step % 10 == 0:
      for addr, dat in STOCK_CLUSTER.items():
        if self.cluster_counter_jump:
          dat = bytes([dat[0] ^ 0xFF]) + dat[1:]  # garbage byte: breaks the checksum on 0x399 / 0x389
        f.append((addr, dat, 2))
    return f

  def tick(self, ci, CC, CC_SP=None):
    self.t += DT_NS
    cs, cs_sp = ci.update([(self.t, self.frames())])
    _, sends = ci.apply(CC, CC_SP or structs.CarControlSP(), self.t)
    self.step += 1
    return cs, cs_sp, sends


def make_cc(enabled=True, lat=True, angle=15.0, curvature=0.001):
  CC = structs.CarControl()
  CC.enabled = enabled
  CC.latActive = lat
  CC.longActive = enabled
  CC.actuators.steeringAngleDeg = angle
  CC.actuators.curvature = curvature
  return CC.as_reader()


def steer_frames(sends):
  out = []
  for addr, dat, _bus in sends:
    if addr == 0x488:
      raw = ((dat[0] & 0x7F) << 8) | dat[1]
      out.append((dat[2] >> 6, -((raw * 0.1) - 1638.35)))
  return out


def warm(car, ci, n=20):
  for _ in range(n):
    cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  return cs


def test_can_valid_and_steering_pressed_at_level_1():
  ci, car = make_ci(), Car()
  cs = warm(car, ci)
  assert cs.canValid
  assert not cs.steeringPressed
  car.hands = 1
  cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  assert cs.steeringPressed
  car.hands = 3
  cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  assert cs.steeringPressed
  assert not cs.steerFaultTemporary


def test_latched_codes_are_not_temporary_faults_other_codes_are():
  ci, car = make_ci(), Car()
  warm(car, ci)
  for err, fault in (("EAC_ERROR_HANDS_ON", False), ("EAC_ERROR_HIGH_ANGLE_REQ", False), ("EAC_ERROR_TMP_FAULT", True)):
    car.err = err
    cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
    assert cs.steerFaultTemporary == fault, err


def test_engage_soft_start_then_planner_angle():
  ci, car = make_ci(), Car()
  warm(car, ci)
  types_angles = []
  for _ in range(AP1_ENGAGE_SOFT_START_FRAMES + 10):
    _, _, sends = car.tick(ci, make_cc(angle=15.0))
    types_angles += steer_frames(sends)
  soft = types_angles[:AP1_ENGAGE_SOFT_START_FRAMES // 2]
  assert all(t == STEERING_CONTROL_ANGLE and a == pytest.approx(5.0, abs=0.11) for t, a in soft)
  assert types_angles[-1][0] == STEERING_CONTROL_ANGLE
  assert types_angles[-1][1] > 5.5  # moving toward the planner angle


def test_override_hold_resume_and_grey_border_flag():
  ci, car = make_ci(), Car()
  warm(car, ci)
  for _ in range(40):
    car.tick(ci, make_cc())
  # Firm override: level 3 -> NONE, steeringPressed
  car.hands = 3
  cs, cs_sp, sends = car.tick(ci, make_cc())
  cs, cs_sp, sends = car.tick(ci, make_cc())
  assert cs.steeringPressed
  assert {t for t, _ in steer_frames(sends)} <= {STEERING_CONTROL_NONE}
  # Easing off at level 1: still NONE
  car.hands = 1
  for _ in range(100):
    cs, cs_sp, sends = car.tick(ci, make_cc())
    assert all(t == STEERING_CONTROL_NONE for t, _ in steer_frames(sends))
  # Hands off: NONE for the hold, steerOverrideHold keeps the border grey with steeringPressed False
  car.hands = 0
  for i in range(AP1_RESUME_HOLD_FRAMES - 1):
    cs, cs_sp, sends = car.tick(ci, make_cc())
    assert all(t == STEERING_CONTROL_NONE for t, _ in steer_frames(sends)), i
    assert not cs.steeringPressed
    if i > 0:
      assert cs_sp.steerOverrideHold, i
  # Hold elapses: ANGLE resumes at the measured wheel (soft-start), then the flag clears
  resumed = []
  for _ in range(12):
    cs, cs_sp, sends = car.tick(ci, make_cc())
    resumed += steer_frames(sends)
  assert resumed and resumed[0][0] == STEERING_CONTROL_ANGLE
  assert all(t == STEERING_CONTROL_ANGLE and a == pytest.approx(5.0, abs=0.11) for t, a in resumed)
  assert not cs_sp.steerOverrideHold


def test_disengage_clears_hold_and_releases_0x488():
  ci, car = make_ci(), Car()
  warm(car, ci)
  for _ in range(5):
    car.tick(ci, make_cc())
  car.hands = 3
  car.tick(ci, make_cc())
  car.hands = 0
  for _ in range(3):
    _, cs_sp, sends = car.tick(ci, make_cc(enabled=False, lat=False))
  assert steer_frames(sends) == []
  assert not cs_sp.steerOverrideHold
  # Re-engage with hands off: soft-start ANGLE, not a leftover hold
  got = []
  for _ in range(4):
    _, _, sends = car.tick(ci, make_cc())
    got += steer_frames(sends)
  assert got and all(t == STEERING_CONTROL_ANGLE for t, _ in got)


def test_mads_lateral_only_gets_hold_and_soft_start():
  ci, car = make_ci(), Car()
  warm(car, ci)
  cc_sp = structs.CarControlSP()
  cc_sp.mads.available = True
  cc_sp.mads.active = True
  got = []
  for _ in range(6):
    _, _, sends = car.tick(ci, make_cc(enabled=False, lat=True), cc_sp)
    got += steer_frames(sends)
  assert got and all(t == STEERING_CONTROL_ANGLE and a == pytest.approx(5.0, abs=0.11) for t, a in got)
  car.hands = 3
  _, _, sends = car.tick(ci, make_cc(enabled=False, lat=True), cc_sp)
  _, cs_sp, sends = car.tick(ci, make_cc(enabled=False, lat=True), cc_sp)
  assert all(t == STEERING_CONTROL_NONE for t, _ in steer_frames(sends))


def test_epas_inhibited_quiet_warning_after_1s():
  ci, car = make_ci(), Car()
  warm(car, ci)
  car.eac = "EAC_INHIBITED"
  flags = []
  for _ in range(AP1_INHIBIT_ALERT_FRAMES + 5):
    cs, cs_sp, sends = car.tick(ci, make_cc())
    flags.append(cs_sp.steerInactiveSilent)
    # Measured-angle ANGLE recovery the whole time
    assert all(t == STEERING_CONTROL_ANGLE and a == pytest.approx(5.0, abs=0.11) for t, a in steer_frames(sends))
  assert not any(flags[:AP1_INHIBIT_ALERT_FRAMES - 1])
  assert flags[-1]
  assert not cs.steerFaultTemporary  # warning only, latActive is not touched
  # Level-3 self-shutoff while inhibited never raises it
  ci, car = make_ci(), Car()
  warm(car, ci)
  car.eac, car.hands = "EAC_INHIBITED", 3
  for _ in range(AP1_INHIBIT_ALERT_FRAMES * 2):
    _, cs_sp, _ = car.tick(ci, make_cc())
    assert not cs_sp.steerInactiveSilent


def test_stalk_personality_request():
  ci, car = make_ci(), Car()
  _, cs_sp, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  assert cs_sp.personalityRequestValid and cs_sp.personalityRequest == 0  # detent 3 -> aggressive
  for raw, valid, want in ((133, True, 1), (166, False, 0), (200, True, 2), (255, True, 2), (0, True, 0)):
    car.dtr = raw
    cs, cs_sp, _ = car.tick(ci, make_cc(enabled=False, lat=False))
    assert (cs_sp.personalityRequestValid, cs_sp.personalityRequest) == (valid, want), raw
    # no gapAdjustCruise presses any more (they cycled the personality the wrong way)
    assert not any(be.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise for be in cs.buttonEvents)


def _cluster_sends(sends):
  return {addr: dat for addr, dat, bus in sends if addr in c.CLUSTER_ADDRS and bus == 0}


def test_ic_integration_flag_default_on_for_ap1_and_param_off():
  assert make_ci().CP_SP.flags & TeslaFlagsSP.AP1_IC_INTEGRATION
  assert make_ci([{"TeslaAp1IcIntegration": True}]).CP_SP.flags & TeslaFlagsSP.AP1_IC_INTEGRATION
  assert not make_ci([{"TeslaAp1IcIntegration": False}]).CP_SP.flags & TeslaFlagsSP.AP1_IC_INTEGRATION


def test_cluster_substitution_engaged_with_model_path():
  ci, car = make_ci(), Car()
  car.cluster = True
  cs = warm(car, ci)
  assert cs.canValid
  cc_sp = structs.CarControlSP()
  cc_sp.modelPathX = [float(i) for i in range(1, 61)]
  cc_sp.modelPathY = [0.0001 * x * x for x in cc_sp.modelPathX]
  got = {}
  for _ in range(20):
    cs, _, sends = car.tick(ci, make_cc(), cc_sp)
    got.update(_cluster_sends(sends))
  assert set(got) == set(c.CLUSTER_ADDRS)
  ap = c.unpack(c.AUTOPILOT_STATUS, got[c.AUTOPILOT_STATUS])
  assert ap["autopilotStatus"] == 5
  stock_ctr = c.unpack(c.AUTOPILOT_STATUS, STOCK_CLUSTER[c.AUTOPILOT_STATUS])["DAS_statusCounter"]
  assert int(ap["DAS_statusCounter"]) == (int(stock_ctr) + 1) % 16
  assert got[c.AUTOPILOT_STATUS][7] == c.tesla_checksum(c.AUTOPILOT_STATUS, got[c.AUTOPILOT_STATUS])
  lanes = c.unpack(c.DAS_LANES, got[c.DAS_LANES])
  assert lanes["DAS_virtualLaneC0"] == pytest.approx(0.0, abs=0.035)
  assert lanes["DAS_virtualLaneC1"] == pytest.approx(0.0, abs=0.0016)
  assert lanes["DAS_virtualLaneC2"] == pytest.approx(0.0004, abs=2e-5)
  assert lanes["DAS_virtualLaneViewRange"] == 60
  # Without a model path: actuator curvature + 50 m
  got = {}
  for _ in range(20):
    _, _, sends = car.tick(ci, make_cc(curvature=0.0002))
    got.update(_cluster_sends(sends))
  lanes = c.unpack(c.DAS_LANES, got[c.DAS_LANES])
  assert lanes["DAS_virtualLaneC2"] == pytest.approx(0.0004, abs=2e-5)
  assert lanes["DAS_virtualLaneViewRange"] == 50


def test_cluster_nothing_when_disengaged_or_toggle_off():
  ci, car = make_ci(), Car()
  car.cluster = True
  for _ in range(30):
    _, _, sends = car.tick(ci, make_cc(enabled=False, lat=False))
    assert _cluster_sends(sends) == {}
  ci, car = make_ci([{"TeslaAp1IcIntegration": False}]), Car()
  car.cluster = True
  for _ in range(30):
    _, _, sends = car.tick(ci, make_cc())
    assert _cluster_sends(sends) == {}


def test_bad_stock_cluster_frames_never_invalidate_can():
  ci, car = make_ci(), Car()
  car.cluster = True
  car.cluster_counter_jump = True
  for _ in range(200):
    cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  assert cs.canValid
  # And a car that never sends them is fine too
  ci, car = make_ci(), Car()
  for _ in range(200):
    cs, _, _ = car.tick(ci, make_cc(enabled=False, lat=False))
  assert cs.canValid


def test_cluster_frames_do_not_change_actuator_frames():
  def run(params):
    ci, car = make_ci(params), Car()
    car.cluster = True
    warm(car, ci)
    out = []
    for _ in range(60):
      _, _, sends = car.tick(ci, make_cc())
      out.append([s for s in sends if s[0] not in c.CLUSTER_ADDRS])
    return out
  assert run([]) == run([{"TeslaAp1IcIntegration": False}])
