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


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br".+,[EYX]\d?[A-Z]*\d{3}\.\d+(?:\.\d+)?",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
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
  LONG_CONTROL = 1
  FSD_14 = 2
  # AP1 Model S on the chassis bus. Bit 8 (0x100). Clear of LONG_CONTROL and FSD_14.
  # BogPilot's panda used bit 3 (8) for the same concept; that numbering is not used here.
  AP1 = 0x100


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  MISSING_DAS_SETTINGS = 4
  AP1 = 0x100


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1
