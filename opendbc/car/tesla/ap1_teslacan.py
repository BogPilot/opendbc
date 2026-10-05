"""AP1 Model S CAN packing. Uses tesla_can.dbc on the chassis bus.

Checksum for DAS_steeringControl / DAS_control matches tesla_checksum.
STW_ACTN_RQ uses CRC poly 0x11d (crcmod mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)),
same as BogPilot/Tinkla. 0x349 is not in the DBC; it is an all-zero Hold clear.
"""

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.ap1_actuator_plan import steering_control_type
from opendbc.car.tesla.values import CANBUS, CarControllerParams


def _stw_crc(data: bytes) -> int:
  # Same as crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
  crc = 0x00
  for b in data:
    crc ^= b
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ 0x11D) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc ^ 0xFF


def _tesla_sum_checksum(msg_id: int, dat: bytes) -> int:
  ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
  ret += sum(dat)
  return ret & 0xFF


class Ap1TeslaCAN:
  def __init__(self, packer):
    self.packer = packer

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": steering_control_type(enabled),
      "DAS_steeringControlCounter": counter,
    }
    # Packer auto-checksum for tesla_can; still set counter explicitly
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.chassis, values)

  def create_action_request(self, msg_stw_actn_req, cancel, bus, counter):
    values = {s: msg_stw_actn_req[s] for s in [
      "SpdCtrlLvr_Stat",
      "VSL_Enbl_Rq",
      "SpdCtrlLvrStat_Inv",
      "DTR_Dist_Rq",
      "TurnIndLvr_Stat",
      "HiBmLvr_Stat",
      "WprWashSw_Psd",
      "WprWash_R_Sw_Posn_V2",
      "StW_Lvr_Stat",
      "StW_Cond_Flt",
      "StW_Cond_Psd",
      "HrnSw_Psd",
      "StW_Sw00_Psd",
      "StW_Sw01_Psd",
      "StW_Sw02_Psd",
      "StW_Sw03_Psd",
      "StW_Sw04_Psd",
      "StW_Sw05_Psd",
      "StW_Sw06_Psd",
      "StW_Sw07_Psd",
      "StW_Sw08_Psd",
      "StW_Sw09_Psd",
      "StW_Sw10_Psd",
      "StW_Sw11_Psd",
      "StW_Sw12_Psd",
      "StW_Sw13_Psd",
      "StW_Sw14_Psd",
      "StW_Sw15_Psd",
      "WprSw6Posn",
      "MC_STW_ACTN_RQ",
      "CRC_STW_ACTN_RQ",
    ]}

    if cancel:
      values["SpdCtrlLvr_Stat"] = 1
      values["MC_STW_ACTN_RQ"] = counter

    # Pack without relying on auto CRC for this poly; set CRC after a dry pack.
    # The packer will overwrite CRC_STW_ACTN_RQ if it thinks it has a tesla checksum
    # signal named *Checksum. CRC_STW_ACTN_RQ does not match that pattern, so we set it.
    data = self.packer.pack(0x45, {k: v for k, v in values.items() if k != "CRC_STW_ACTN_RQ"})
    values["CRC_STW_ACTN_RQ"] = _stw_crc(bytes(data[:7]))
    return self.packer.make_can_msg("STW_ACTN_RQ", bus, values)

  def create_longitudinal_commands(self, acc_state, speed, min_accel, max_accel, cnt):
    values = {
      "DAS_setSpeed": speed * CV.MS_TO_KPH,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.AP1_JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.AP1_JERK_LIMIT_MAX,
      "DAS_accelMin": min_accel,
      "DAS_accelMax": max_accel,
      "DAS_controlCounter": cnt,
    }
    return [self.packer.make_can_msg("DAS_control", CANBUS.chassis, values)]

  @staticmethod
  def create_ap1_hold_clear(bus=CANBUS.chassis):
    """All-zero chassis 0x349. Clears Tesla Hold's press-accelerator prompt.

    Bit layout is Tinkla create_das_warningMatrix3 with DAS_gas_to_resume = 0.
    0x349 is not in tesla_can.dbc.
    """
    return 0x349, b"\x00" * 8, bus
