from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.ap1_carcontroller import Ap1CarController
from opendbc.car.tesla.ap1_carstate import Ap1CarState
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR

from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP

# Chassis fingerprint: bus 0 must include these three AP1 addresses (BogPilot classify_ap1_chassis).
_AP1_CHASSIS_ADDRS = frozenset((0x45, 0x2B9, 0x488))


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    if CP.flags & TeslaFlags.AP1:
      # Swap in the AP1 chassis stack before CarInterfaceBase builds CS/CC.
      self.CarState = Ap1CarState
      self.CarController = Ap1CarController
    super().__init__(CP, CP_SP)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    if candidate == CAR.TESLA_AP1_MODELS:
      # AP1 Model S: Mobileye chassis bus 0. Known-working BogPilot path.
      # Safety param is AP1 | LONG when alpha long is on. LONG alone is Model 3/Y.
      ret.flags |= TeslaFlags.AP1.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.AP1.value
      ret.radarUnavailable = True
      ret.alphaLongitudinalAvailable = True
      ret.dashcamOnly = False
      ret.steerLimitTimer = 1.0
      ret.steerActuatorDelay = 0.25
      ret.longitudinalActuatorDelay = 0.5
      ret.pcmCruise = True
      # Warn if the live fingerprint is missing the chassis signature, but still
      # allow CarPlatformBundle / fixed selection (same as BogPilot force fingerprint).
      bus0 = fingerprint.get(CANBUS.chassis, {}) if isinstance(fingerprint, dict) else {}
      if isinstance(bus0, dict) and bus0 and not (_AP1_CHASSIS_ADDRS <= set(bus0)):
        # Soft note only; dashcam stays off so a forced platform can still engage.
        pass
      if alpha_long:
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value
      return ret

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.FSD_14.value

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X,)  # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:

    if candidate == CAR.TESLA_AP1_MODELS:
      # No BSM on this port. No vehicle-bus MADS screen button.
      stock_cp.enableBsm = False
      return ret

    stock_cp.enableBsm = True

    if candidate == CAR.TESLA_MODEL_X:
      stock_cp.dashcamOnly = False

    if 0x3DF in fingerprint[1]:
      ret.flags |= TeslaFlagsSP.HAS_VEHICLE_BUS.value
      ret.safetyParam |= TeslaSafetyFlagsSP.HAS_VEHICLE_BUS

    return ret
