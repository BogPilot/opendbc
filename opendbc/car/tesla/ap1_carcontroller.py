"""AP1 Model S CarController. Chassis 0x488 / 0x2b9 / 0x45 / 0x349.

Ported from BogPilot/openpilot tag ap1-driving-milestone-1 (013f1ffa)
selfdrive/car/tesla/carcontroller.py AP1 path.
"""

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.ap1_actuator_plan import (
  ap1_should_send_hold_clear,
  build_actuator_plan,
  longitudinal_command_allowed,
)
from opendbc.car.tesla.ap1_hso import ap1_lat_active
from opendbc.car.tesla.ap1_teslacan import Ap1TeslaCAN
from opendbc.car.tesla.values import CANBUS, DBC


class Ap1CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    # CarControllerBase expects dbc_names; AP1 only has Bus.chassis
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_angle_last = 0.0
    self.packer = CANPacker(dbc_names[Bus.chassis] if Bus.chassis in dbc_names else DBC[CP.carFingerprint][Bus.chassis])
    self.tesla_can = Ap1TeslaCAN(self.packer)

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    lat_active = ap1_lat_active(CC.latActive, CS.hands_on_level)
    plan = build_actuator_plan(
      self.frame,
      lat_active,
      False,  # AP1 never cancels cruise on hands-on
      self.CP.openpilotLongitudinalControl,
      CC.enabled,
      CC.longActive,
      CS.out.steeringAngleDeg,
      actuators.steeringAngleDeg,
      self.apply_angle_last,
      CS.out.vEgo,
      actuators.accel,
      CS.acc_state,
      CS.das_control_counters,
      CC.cruiseControl.cancel,
      chassis_das_only=True,
      epas_error=CS.steer_warning,
      eac_fault=bool(CS.eac_fault),
      hands_on_level=CS.hands_on_level,
    )
    self.apply_angle_last = plan.apply_angle_last

    can_sends = []

    if plan.steer is not None:
      can_sends.append(self.tesla_can.create_steering_control(
        plan.steer.angle_deg, plan.steer.enabled, plan.steer.counter))

    for cmd in plan.longitudinal:
      can_sends.extend(self.tesla_can.create_longitudinal_commands(
        cmd.acc_state, cmd.target_speed, cmd.min_accel, cmd.max_accel, cmd.counter))

    if plan.cancel and CS.msg_stw_actn_req is not None:
      for counter in range(16):
        can_sends.append(self.tesla_can.create_action_request(
          CS.msg_stw_actn_req, True, CANBUS.chassis, counter))
        can_sends.append(self.tesla_can.create_action_request(
          CS.msg_stw_actn_req, True, CANBUS.autopilot_chassis, counter))

    long_allowed = longitudinal_command_allowed(
      self.CP.openpilotLongitudinalControl, CC.enabled, CC.longActive)
    if ap1_should_send_hold_clear(True, long_allowed, CS.acc_state, self.frame):
      can_sends.append(self.tesla_can.create_ap1_hold_clear())

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)
    self.frame += 1
    return new_actuators, can_sends
