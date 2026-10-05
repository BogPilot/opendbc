"""AP1 hands-on pause. No CAN.

Tinkla (501c7de) HSO, default TinklaHso True and TinklaHandsOnLevel 2.0:
steeringPressed is EPAS_handsOnLevel >= 2. While that is true, lateral
planning is paused and cruise is not dropped. steerTempUnavailableSilent
is warning-only. It is not no-entry and not a soft disable.

The 50-frame numb period and the 15 degree handoff in HSO_module.py are
not applied. Resume is the next step after hands_on_level drops below 2,
if cruise is still enabled and EPAS is not EAC_FAULT. Code 6 and a latched
code 3 do not block that resume. Any other non-idle name stays a steer warning.
"""

# selfdrive/car/tesla/carstate.py: load_float_param("TinklaHandsOnLevel", 2.0)
AP1_HANDS_ON_LEVEL = 2

_HARSH = "steerTempUnavailable"
_SILENT = "steerTempUnavailableSilent"


def ap1_steering_pressed(hands_on_level):
  """True at the Tinkla HSO threshold. Level 1 is not an interrupt."""
  return hands_on_level >= AP1_HANDS_ON_LEVEL


def ap1_lat_active(lat_active, hands_on_level):
  """Path lateral is off while hands are at or above the HSO threshold.

  controlsd also clears latActive from steeringPressed on AP1, which resets
  the lateral controller. This gate is the actuator-side check so a planned
  angle is not sent if that bit is still set.
  """
  if ap1_steering_pressed(hands_on_level):
    return False
  return bool(lat_active)


def ap1_hso_event_names(names):
  """Replace the soft-disable steer event with the silent warning.

  steerUnavailable (EAC_FAULT) is left alone. That one still disables.
  Duplicate silent names are dropped.
  """
  out = []
  for name in names:
    if name == _HARSH:
      name = _SILENT
    if name not in out:
      out.append(name)
  return out
