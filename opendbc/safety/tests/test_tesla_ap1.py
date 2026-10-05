#!/usr/bin/env python3
"""AP1 Model S chassis safety (tesla_ap1.h).

Reference: BogGyver/Tinkla panda safety_tesla.h (has_ap_hardware path), via BogPilot's AP1 port.
Safety param uses BogGyver/Tinkla's FLAG_TESLA_* numbering, not the Model 3/Y layout.
Does not weaken Model 3/Y tests in test_tesla.py.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import TeslaAp1SafetyFlags, TeslaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_STEER = 0x488
MSG_STALK = 0x45
MSG_LONG = 0x2b9
MSG_HOLD = 0x349
# AP1 cluster substitution (BogPilot ap1-driving-milestone-2): addr -> stock-drop window in us
CLUSTER = {0x399: 750000, 0x389: 750000, 0x239: 150000}
# DAS frames that are still never sent (no invented signals, no ALCA / body controls)
OTHER_DAS = (0x309, 0x3a9, 0x3e9, 0x329, 0x369)


class TestTeslaAp1SafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_STEER,)}
  # Interceptor: stock frames are not statically blacklisted. Dynamic substitute only.
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  TX_MSGS = [
    [MSG_STEER, 0],
    [MSG_STALK, 0],
    [MSG_STALK, 2],
    [MSG_LONG, 0],
    [MSG_HOLD, 0],
    [0x399, 0],
    [0x389, 0],
    [0x239, 0],
  ]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # EPAS_internalSAS DBC range is about +/-819 deg; keep tests inside that.
  STEER_ANGLE_MAX = 800  # inside EPAS_internalSAS DBC range; safety max_angle is 819.2
  DEG_TO_CAN = 10
  ANGLE_RATE_BP = [2., 7., 17.]
  ANGLE_RATE_UP = [8., 4., 2.5]
  ANGLE_RATE_DOWN = [9., 5., 4.5]
  LATERAL_FREQUENCY = 50

  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.52
  INACTIVE_ACCEL = 0.0

  packer: CANPackerSafety
  LONGITUDINAL = True
  SAFETY_PARAM = int(TeslaAp1SafetyFlags.HAS_AP | TeslaAp1SafetyFlags.LONG_CONTROL)

  def setUp(self):
    self.packer = CANPackerSafety("tesla_can")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True, bus: int = 0):
    values = {
      "DAS_steeringAngleRequest": angle,
      "DAS_steeringControlType": 1 if enabled else 0,
    }
    if increment_timer:
      # 50 Hz
      self.__class__.cnt_angle_cmd = getattr(self.__class__, "cnt_angle_cmd", 0) + 1
      self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float):
    values = {"EPAS_internalSAS": angle}
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _speed_msg(self, speed):
    values = {"DI_vehicleSpeed": speed / 0.447}
    return self.packer.make_can_msg_safety("DI_torque2", 0, values)

  def _speed_msg_2(self, speed: float):
    return self._speed_msg(speed)

  def _vehicle_moving_msg(self, speed: float):
    return self._speed_msg(speed)

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": 1.0 if gas else 0.0}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=4, accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": -8,
      "DAS_jerkMax": 8,
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def _hold_clear_msg(self, dat=b"\x00" * 8):
    return libsafety_py.make_CANPacket(MSG_HOLD, 0, dat)

  def _stalk_cancel_msg(self, lever=1, bus=0):
    values = {"SpdCtrlLvr_Stat": lever}
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", bus, values)

  def test_rx_hook_speed_mismatch(self):
    # AP1 does not run a second-speed mismatch check.
    pass

  def test_rt_limits(self):
    raise unittest.SkipTest("AP1 uses steer_angle_cmd_checks lookup rates, not VM RT limits")

  def test_angle_cmd_when_enabled(self):
    # Base test mixes STEER_ANGLE_MAX with the rate table; keep a focused AP1 check.
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(10)
    self._set_prev_desired_angle(0)
    # small step within rate limit at 10 m/s (up rate interp ~3.4)
    self.assertTrue(self._tx(self._angle_cmd_msg(2.0, True)))
    # large step must fail
    self.assertFalse(self._tx(self._angle_cmd_msg(20.0, True)))

  def test_ap1_flags_are_boggyver_layout(self):
    # BogGyver/panda board/safety/safety_tesla.h: FLAG_TESLA_LONG_CONTROL = 2, FLAG_TESLA_HAS_AP = 16
    self.assertEqual(int(TeslaAp1SafetyFlags.LONG_CONTROL), 2)
    self.assertEqual(int(TeslaAp1SafetyFlags.HAS_AP), 16)
    # The AP1 selector must never collide with a Model 3/Y/X safety bit
    for flag in TeslaSafetyFlags:
      self.assertEqual(int(flag) & int(TeslaAp1SafetyFlags.HAS_AP), 0, flag)

  def test_model3y_params_never_select_ap1(self):
    # Every Model 3/Y/X param combination keeps the Model 3/Y safety: AP1-only 0x45 cancel TX is rejected
    all_m3y = 0
    for flag in TeslaSafetyFlags:
      all_m3y |= int(flag)
    for param in range(all_m3y + 1):
      if param & ~all_m3y:
        continue
      self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, param)
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._stalk_cancel_msg(1)), param)
    # The old SunnyTesla AP1 bit (0x100) no longer selects AP1 either
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0x101)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._stalk_cancel_msg(1)))

  def test_model3y_long_bit_does_not_enable_ap1_long(self):
    # Model 3/Y LONG_CONTROL (1) means nothing to AP1. Only BogGyver/Tinkla LONG_CONTROL (2) does.
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, int(TeslaAp1SafetyFlags.HAS_AP | TeslaSafetyFlags.LONG_CONTROL))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._stalk_cancel_msg(1)))
    self.assertFalse(self._tx(self._long_control_msg(10)))
    self.assertFalse(self._tx(self._hold_clear_msg()))

  def test_hold_clear_all_zero_only(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._hold_clear_msg()))
    self.assertFalse(self._tx(self._hold_clear_msg(b"\x02" + b"\x00" * 7)))

  def test_stalk_cancel_only(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._stalk_cancel_msg(1)))
    self.assertFalse(self._tx(self._stalk_cancel_msg(2)))

  def test_steer_control_type_angle_or_none(self):
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    for t in (2, 3):
      values = {"DAS_steeringAngleRequest": 0, "DAS_steeringControlType": t}
      msg = self.packer.make_can_msg_safety("DAS_steeringControl", 0, values)
      self.assertFalse(self._tx(msg))

  def test_interceptor_forwards_stock_until_op_tx(self):
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_STEER))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_LONG))
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_STEER))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_LONG))
    if self.LONGITUDINAL:
      self.assertTrue(self._tx(self._long_control_msg(10, accel_limits=(0, 0))))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_LONG))

  def test_no_aeb(self):
    if not self.LONGITUDINAL:
      raise unittest.SkipTest("no long")
    self.safety.set_controls_allowed(True)
    for aeb in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb)), aeb == 0)

  def test_stock_aeb_blocks_op_long(self):
    if not self.LONGITUDINAL:
      raise unittest.SkipTest("no long")
    self.safety.set_controls_allowed(True)
    aeb = self._long_control_msg(10, aeb_event=1, bus=2)
    self.assertTrue(self._rx(aeb))
    self.assertFalse(self._tx(self._long_control_msg(10, aeb_event=0)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_LONG))

  def test_accel_limits(self):
    if not self.LONGITUDINAL:
      raise unittest.SkipTest("no long")
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_msg(self.MAX_ACCEL)))
    self.assertTrue(self._tx(self._accel_msg(self.MIN_ACCEL)))
    self.assertFalse(self._tx(self._accel_msg(self.MAX_ACCEL + 0.1)))
    self.assertFalse(self._tx(self._accel_msg(self.MIN_ACCEL - 0.1)))
    # both accel limits negative is blocked
    self.assertFalse(self._tx(self._long_control_msg(10, accel_limits=(-1, -0.5))))

  # --- AP1 cluster frames (port of BogPilot panda/tests/safety/test_tesla_ap1_cluster.py) ---

  @staticmethod
  def _status(state):
    dat = bytearray(8)
    dat[0] = state & 0x0F
    return bytes(dat)

  def _cluster_tx(self, addr, dat=None, bus=0):
    return self._tx(libsafety_py.make_CANPacket(addr, bus, dat if dat is not None else b"\x00" * 8))

  def test_cluster_tx_allowed_on_bus0_only(self):
    for addr in CLUSTER:
      self.assertTrue(self._cluster_tx(addr, self._status(2)), hex(addr))
      self.assertFalse(self._cluster_tx(addr, self._status(2), bus=2), hex(addr))
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(addr, 0, b"\x00" * 7)), hex(addr))

  def test_other_das_frames_not_allowed(self):
    self.safety.set_controls_allowed(True)
    for addr in OTHER_DAS:
      self.assertFalse(self._cluster_tx(addr), hex(addr))

  def test_active_autopilot_state_needs_controls_allowed(self):
    for state in range(16):
      self.safety.set_controls_allowed(False)
      self.assertEqual(self._cluster_tx(0x399, self._status(state)), state not in (3, 4, 5), state)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._cluster_tx(0x399, self._status(state)), state)
    # Only 0x399 carries that state. The same bits on 0x389 / 0x239 are not checked.
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._cluster_tx(0x389, self._status(5)))
    self.assertTrue(self._cluster_tx(0x239, self._status(5)))

  def test_stock_cluster_forwarded_without_op_tx(self):
    for addr in list(CLUSTER) + list(OTHER_DAS):
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr), hex(addr))

  def test_stock_cluster_dropped_only_while_op_recently_sent(self):
    for addr, timeout in CLUSTER.items():
      self.setUp()
      t = 5_000_000
      self.safety.set_timer(t)
      self.assertTrue(self._cluster_tx(addr, self._status(2)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, addr), hex(addr))
      self.safety.set_timer(t + timeout - 1)
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, addr), hex(addr))
      self.safety.set_timer(t + timeout)
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr), hex(addr))
      # Other cluster addresses are tracked separately
      self.safety.set_timer(t)
      self._cluster_tx(addr, self._status(2))
      for other in CLUSTER:
        if other != addr:
          self.assertEqual(0, self.safety.safety_fwd_hook(2, other), (hex(addr), hex(other)))
      # Bus 0 to 2 is unchanged
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr))

  def test_rejected_cluster_tx_does_not_start_substitution(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._cluster_tx(0x399, self._status(5)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x399))

  def test_init_clears_cluster_substitution(self):
    self.assertTrue(self._cluster_tx(0x239))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x239))
    self.setUp()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x239))

  def test_cluster_tx_does_not_change_steer_or_long_forwarding(self):
    self.assertTrue(self._cluster_tx(0x399, self._status(2)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_STEER))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_LONG))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_HOLD))


class TestTeslaAp1M3YParamNoCluster(unittest.TestCase):
  """Model 3/Y/X params never get the AP1 cluster TX or the stock-cluster drop."""

  def test_no_cluster_without_has_ap(self):
    lib = libsafety_py.libsafety
    for param in (0, int(TeslaSafetyFlags.LONG_CONTROL), int(TeslaSafetyFlags.FSD_14)):
      lib.set_safety_hooks(CarParams.SafetyModel.tesla, param)
      lib.init_tests()
      lib.set_controls_allowed(True)
      for addr in CLUSTER:
        self.assertFalse(lib.safety_tx_hook(libsafety_py.make_CANPacket(addr, 0, b"\x02" + b"\x00" * 7)), (param, hex(addr)))
        self.assertEqual(0, lib.safety_fwd_hook(2, addr), (param, hex(addr)))


class TestTeslaAp1LongSafety(TestTeslaAp1SafetyBase):
  pass


class TestTeslaAp1LatOnlySafety(TestTeslaAp1SafetyBase):
  LONGITUDINAL = False
  SAFETY_PARAM = int(TeslaAp1SafetyFlags.HAS_AP)

  def test_no_long_tx(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._long_control_msg(10)))
    self.assertFalse(self._tx(self._hold_clear_msg()))

  def test_hold_clear_all_zero_only(self):
    raise unittest.SkipTest("hold clear requires LONG")


class TestTeslaAp1ReleaseBuild(unittest.TestCase):
  """Release (non-ALLOW_DEBUG) panda builds ignore the AP1 long flag. Lateral and cancel still work."""

  @classmethod
  def setUpClass(cls):
    cls.lib = libsafety_py.ffi.dlopen(libsafety_py._build_libsafety(release=True))

  def setUp(self):
    self.packer = CANPackerSafety("tesla_can")
    self.lib.set_safety_hooks(CarParams.SafetyModel.tesla, int(TeslaAp1SafetyFlags.HAS_AP | TeslaAp1SafetyFlags.LONG_CONTROL))
    self.lib.init_tests()
    self.lib.set_controls_allowed(True)

  def _tx(self, msg):
    return self.lib.safety_tx_hook(msg)

  def test_long_rejected_without_allow_debug(self):
    long_msg = self.packer.make_can_msg_safety("DAS_control", 0, {"DAS_setSpeed": 10, "DAS_accState": 4,
                                                                   "DAS_accelMin": 0, "DAS_accelMax": 0})
    self.assertFalse(self._tx(long_msg))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(MSG_HOLD, 0, b"\x00" * 8)))
    # stock 0x2b9 keeps flowing from the autopilot side
    self.assertEqual(0, self.lib.safety_fwd_hook(2, MSG_LONG))

  def test_cancel_still_allowed(self):
    cancel = self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": 1})
    self.assertTrue(self._tx(cancel))


if __name__ == "__main__":
  unittest.main()
