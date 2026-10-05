"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class TeslaFlagsSP(IntFlag):
  HAS_VEHICLE_BUS = 1  # Multi-finger infotainment press signal is present on the VEHICLE bus with the deprecated Tesla harness installed
  COOP_STEERING = 2  # Coop steering
  MADS_SCREEN_BUTTON_3_FINGER = 4
  MADS_SCREEN_BUTTON_4_FINGER = 8
  MADS_SCREEN_BUTTON_5_FINGER = 16
  AP1_IC_INTEGRATION = 32  # AP1 Model S: openpilot substitutes the instrument-cluster frames (0x399 / 0x389 / 0x239)


class MadsScreenButtonType:
  OFF = 0
  THREE_FINGER = 1
  FOUR_FINGER = 2
  FIVE_FINGER = 3


class TeslaSafetyFlagsSP:
  HAS_VEHICLE_BUS = 1
  MADS_SCREEN_BUTTON_3_FINGER = 2
  MADS_SCREEN_BUTTON_4_FINGER = 4
  MADS_SCREEN_BUTTON_5_FINGER = 8
