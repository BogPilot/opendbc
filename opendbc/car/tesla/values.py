from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, AngleSteeringLimitsVM
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])

@dataclass
class TeslaCarHW4ModelSXDocs(TeslaCarDocsHW4):
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "community"


@dataclass
class TeslaAp1CarDocs(CarDocs):
  package: str = "All"
  # Custom AP1 / Mobileye chassis harness. Not the Model 3/Y Tesla A/B harness.
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "community"


@dataclass
class TeslaAp1PlatformConfig(PlatformConfig):
  # Chassis bus uses tesla_can.dbc (AP1 Model S). No party/radar DBC for this platform.
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.chassis: 'tesla_can'})


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party', Bus.adas: 'tesla_model3_vehicle'})


class CAR(Platforms):
  TESLA_AP1_MODELS = TeslaAp1PlatformConfig(
    [TeslaAp1CarDocs("Tesla AP1 Model S 2014-16")],
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=15.0),
  )
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      TeslaCarDocsHW3("Tesla Model 3 (with HW3) 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 (with HW4) 2024-25"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated', Bus.adas: 'tesla_model3_vehicle'},
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y (with HW3) 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y (with HW4) 2024-25"),
    ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated', Bus.adas: 'tesla_model3_vehicle'},
  )
  TESLA_MODEL_X = TeslaPlatformConfig(
    [TeslaCarHW4ModelSXDocs("Tesla Model X (with HW4) 2024")],
    CarSpecs(mass=2495., wheelbase=2.960, steerRatio=12.0),
  )


# Model 3/Y/X EPS firmware: '<prefix>,<E|Y|X><variant>.<major>[.<minor>]'.
_M3Y_FW_VERSION_RE = br".+,[EYX]\d?[A-Z]*\d{3}\.\d+(?:\.\d+)?"
# AP1 Model S (Mobileye, chassis bus 0). Formats seen on the user's AP1 rlogs:
#   electricBrakeBooster 0x64d F181: Tesla part number, e.g. b'1037123-00-A'
#   eps                  0x730 F188: Tesla part number padded with NULs, e.g. b'1016704-00-HAA' + 10 * b'\x00'
#   fwdRadar             0x671 F181: 19 raw bytes starting b'\x01\x00W' (Bosch MRR)
_AP1_FW_VERSION_RE = br"\d{7}-\d{2}-[A-Z]{1,3}\x00*|\x01\x00W[\x00-\xff]{16}"

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:" + _M3Y_FW_VERSION_RE + br")|(?:" + _AP1_FW_VERSION_RE + br")",
  requests=[
    # Model 3/Y/X EPS. Whitelisted to the EPS so it does not also hit the AP1 chassis ECUs below.
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      bus=0,
    ),
    # AP1 Model S: brake booster 0x64d -> 0x65d and Bosch radar 0x671 -> 0x681 answer UDS F181 on the
    # chassis bus. Same request BogPilot's AP1 port uses (BogPilot selfdrive/car/tesla/values.py,
    # FW_QUERY_CONFIG, rx_offset 0x10). BogGyver/Tinkla's own AP1 port has no FW query (it CAN-fingerprints).
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.electricBrakeBooster, Ecu.fwdRadar],
      rx_offset=0x10,
      bus=0,
    ),
    # AP1 Model S EPAS 0x730 -> 0x738 answers UDS F188 (manufacturer ECU software number). Seen on the
    # user's AP1 rlogs. The Model 3/Y EPS does not use this DID in the database, so it cannot match AP1.
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
  ]
)

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
    b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4H015.05.0',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
    b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
  ]
}


class CANBUS:
  party = 0
  vehicle = 1
  autopilot_party = 2
  # AP1 Model S aliases. Same numbers as party / autopilot_party.
  chassis = 0
  autopilot_chassis = 2


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


class CarControllerParams:
  # AP1 Model S: Tinkla earlytesla-panda TESLA_LOOKUP_ANGLE_RATE_UP / _DOWN.
  # Used by ap1_carcontroller. Model 3/Y still uses ANGLE_LIMITS below.
  AP1_ANGLE_LIMITS = AngleSteeringLimits(
    819.2,  # deg, EPAS_internalSAS range
    ([2., 7., 17.], [8., 4., 2.5]),
    ([2., 7., 17.], [9., 5., 4.5]),
  )
  AP1_STEER_STEP = 2  # 50 Hz
  AP1_ACCEL_TO_SPEED_MULTIPLIER = 3
  AP1_JERK_LIMIT_MAX = 8
  AP1_JERK_LIMIT_MIN = -8

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    # EPAS faults above this angle
    360,  # deg
    # limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
    MAX_ANGLE_RATE=5,  # deg/20ms frame, EPS faults at 12 at a standstill
  )

  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0    # m/s^2
  ACCEL_MIN = -3.48  # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9  # m/s^3, ACC faults at 5.0


class TeslaSafetyFlags(IntFlag):
  # Model 3/Y/X safety param (opendbc/safety/modes/tesla.h). Never used for AP1.
  LONG_CONTROL = 1
  FSD_14 = 2


class TeslaAp1SafetyFlags(IntFlag):
  """AP1 Model S safety param (opendbc/safety/modes/tesla_ap1.h).

  Numbering is BogGyver/Tinkla's (BogGyver/panda board/safety/safety_tesla.h, FLAG_TESLA_*):
    POWERTRAIN = 1, LONG_CONTROL = 2, RADAR_BEHIND_NOSECONE = 4, HAS_IC_INTEGRATION = 8,
    HAS_AP = 16, NEED_RADAR_EMULATION = 32, ENABLE_HAO = 64, HAS_IBOOSTER = 128.
  This port implements only HAS_AP (selects the AP1 safety) and LONG_CONTROL (chassis 0x2b9).
  The other BogGyver/Tinkla bits are not implemented and are never set.
  HAS_AP must stay clear of every TeslaSafetyFlags bit (tested).
  """
  LONG_CONTROL = 2
  HAS_AP = 16


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  MISSING_DAS_SETTINGS = 4
  AP1 = 0x100


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1
