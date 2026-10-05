from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.ap1_carcontroller import Ap1CarController
from opendbc.car.tesla.ap1_carstate import Ap1CarState
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaAp1SafetyFlags, TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR

from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP


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
    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # AP1 Model S is its own port. Return before any Model 3/Y/X setting is applied.
    if candidate == CAR.TESLA_AP1_MODELS:
      return CarInterface._get_params_ap1(ret)

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

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
  def _get_params_ap1(ret: structs.CarParams) -> structs.CarParams:
    """AP1 Model S (Mobileye, chassis bus 0). A separate car port, nothing shared with Model 3/Y/X.

    Reference: BogGyver/Tinkla AP1 (BogGyver/openpilot tesla_unity_dev selfdrive/car/tesla/interface.py and
    BogGyver/panda board/safety/safety_tesla.h), via BogPilot's AP1 port.

    Longitudinal is openpilot's by default and is not gated on AlphaLongitudinalEnabled (BogPilot
    selfdrive/car/tesla/interface.py AP1 branch). BogGyver/Tinkla itself defaults AP1 to stock ACC and
    opts in with TinklaEnableOPLong; there is no BogGyver opt-out, so none is added here.
    """
    ret.flags |= TeslaFlags.AP1.value
    # BogGyver/Tinkla FLAG_TESLA_HAS_AP selects the AP1 safety; FLAG_TESLA_LONG_CONTROL allows chassis 0x2b9.
    ret.safetyConfigs[0].safetyParam = int(TeslaAp1SafetyFlags.HAS_AP | TeslaAp1SafetyFlags.LONG_CONTROL)
    ret.openpilotLongitudinalControl = True
    ret.alphaLongitudinalAvailable = False
    ret.radarUnavailable = True
    ret.dashcamOnly = False
    ret.pcmCruise = True
    # BogGyver/Tinkla does not steer at standstill (steerAtStandstill left at its default, False).
    ret.steerAtStandstill = False
    # BogGyver/Tinkla interface.py: steerLimitTimer 1.0, steerActuatorDelay 0.25,
    # longitudinalActuatorDelayUpperBound 0.5
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    ret.longitudinalActuatorDelay = 0.5
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
