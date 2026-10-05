"""AP1 Model S CarState. Chassis bus 0 + autopilot chassis bus 2, tesla_can.dbc.

Ported from BogPilot/openpilot tag ap1-driving-milestone-1 (013f1ffa)
selfdrive/car/tesla/carstate.py AP1 path. No Raven branch.
"""

from __future__ import annotations

import copy
from collections import deque
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.ap1_hso import ap1_steering_pressed
from opendbc.car.tesla.ap1_stalk_follow import dtr_sample, follow_seconds, parse_stalk_raw
from opendbc.car.tesla.ap1_steer_fault import steer_fault_temporary
from opendbc.car.tesla.values import CANBUS, DBC, GEAR_MAP

ButtonType = structs.CarState.ButtonEvent.Type

# From BogPilot values.BUTTONS for STW_ACTN_RQ
_BUTTONS = (
  (ButtonType.leftBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", (1,)),
  (ButtonType.rightBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", (2,)),
  (ButtonType.accelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", (4, 16)),
  (ButtonType.decelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", (8, 32)),
  (ButtonType.cancel, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", (1,)),
  (ButtonType.resumeCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", (2,)),
  # Stalk distance scroll -> personality cycle (sunnypilot selfdrived listens for gapAdjustCruise)
  (ButtonType.gapAdjustCruise, "STW_ACTN_RQ", "DTR_Dist_Rq", (0, 33, 66, 100, 133, 166, 200)),
)

_DOORS = (
  "DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR",
  "DOOR_STATE_FrontTrunk", "BOOT_STATE",
)


class Ap1CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
    self.button_states = {event_type: False for event_type, *_ in _BUTTONS}

    self.msg_stw_actn_req = None
    self.hands_on_level = 0
    self.steer_warning = None
    self.eac_fault = False
    self.acc_state = 0
    self.das_control_counters: deque[int] = deque(maxlen=32)
    self.stalk_follow = None
    self._prev_dtr_raw = None

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.chassis]
    cp_cam = can_parsers[Bus.ap_party]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgo < 0.1

    # Gas pedal
    gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0
    ret.gasPressed = gas > 0

    # Brake pedal
    ret.brakePressed = bool(cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)

    # Steering wheel
    epas_status = cp.vl["EPAS_sysStatus"]
    self.hands_on_level = int(epas_status["EPAS_handsOnLevel"])
    self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(
      int(epas_status["EPAS_eacErrorCode"]), None)
    steer_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(
      int(epas_status["EPAS_eacStatus"]), None)

    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
    ret.steeringPressed = ap1_steering_pressed(self.hands_on_level)
    self.eac_fault = steer_status == "EAC_FAULT"
    ret.steerFaultPermanent = self.eac_fault
    ret.steerFaultTemporary = steer_fault_temporary(self.steer_warning, True)

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(
      int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(
      int(cp.vl["DI_state"]["DI_speedUnits"]), None)

    acc_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    ret.cruiseState.enabled = acc_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
    ret.cruiseState.available = (cruise_state == "STANDBY") or ret.cruiseState.enabled
    ret.cruiseState.standstill = False

    # Gear
    gear_name = self.can_define.dv["DI_torque2"]["DI_gear"].get(
      int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")
    ret.gearShifter = GEAR_MAP[gear_name]

    # Buttons. Distance detents only emit gapAdjustCruise on a change between known raws.
    button_events = []
    for event_type, addr, signal, values in _BUTTONS:
      raw = cp.vl[addr][signal]
      if event_type == ButtonType.gapAdjustCruise:
        state = (raw in values) and (self._prev_dtr_raw is not None) and (raw != self._prev_dtr_raw)
        # Rising-edge only for personality cycle
        pressed = bool(state)
        if self.button_states[event_type] != pressed and pressed:
          event = structs.CarState.ButtonEvent(type=event_type, pressed=True)
          button_events.append(event)
          # Matching falling edge so selfdrived's "not pressed" check fires
          button_events.append(structs.CarState.ButtonEvent(type=event_type, pressed=False))
        self.button_states[event_type] = pressed
        if raw in values:
          self._prev_dtr_raw = raw
      else:
        state = raw in values
        if self.button_states[event_type] != state:
          event = structs.CarState.ButtonEvent(type=event_type, pressed=state)
          button_events.append(event)
        self.button_states[event_type] = state
    ret.buttonEvents = button_events

    # Doors
    ret.doorOpen = any(
      self.can_define.dv["GTW_carState"][door].get(int(cp.vl["GTW_carState"][door]), "OPEN") == "OPEN"
      for door in _DOORS
    )

    # Blinkers
    ret.leftBlinker = cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    ret.stockAeb = cp_cam.vl["DAS_control"]["DAS_aebEvent"] == 1

    # Stalk follow. Also publish seconds on cruiseState.speedOffset for any
    # follow-time consumer (BogPilot used FrogPilotFollowing).
    stw = cp.vl.get("STW_ACTN_RQ")
    ts_map = getattr(cp, "ts_nanos", {}).get("STW_ACTN_RQ", {})
    if not isinstance(stw, dict) or not isinstance(ts_map, dict):
      raw = None
    else:
      raw = dtr_sample(stw.get("DTR_Dist_Rq"), ts_map.get("DTR_Dist_Rq", 0))
    self.stalk_follow = parse_stalk_raw(raw, self.stalk_follow)
    # follow_seconds(self.stalk_follow) is available on CS.stalk_follow.
    # sunnypilot CarState.cruiseState has no speedOffset field (BogPilot/FrogPilot did).
    # Personality is cycled via gapAdjustCruise button events from DTR_Dist_Rq changes.
    _ = follow_seconds(self.stalk_follow)

    # Messages needed by carcontroller
    self.msg_stw_actn_req = copy.copy(cp.vl["STW_ACTN_RQ"])
    self.acc_state = int(cp_cam.vl["DAS_control"]["DAS_accState"])
    self.das_control_counters.extend(cp_cam.vl_all["DAS_control"]["DAS_controlCounter"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    chassis_msgs = [
      ("ESP_B", 50),
      ("DI_torque1", 100),
      ("DI_torque2", 100),
      ("STW_ANGLHP_STAT", 100),
      ("EPAS_sysStatus", 25),
      ("DI_state", 10),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),
      ("BrakeMessage", 50),
      ("SDM1", 10),
    ]
    cam_msgs = [
      ("DAS_control", 40),
    ]
    return {
      Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_msgs, CANBUS.chassis),
      # ap_party key keeps CarInterfaceBase / Ext call sites consistent; physical bus is 2.
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.chassis], cam_msgs, CANBUS.autopilot_chassis),
    }
