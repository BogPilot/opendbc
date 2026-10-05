"""Temporary steer-fault gate. No CAN, no parser.

Tinkla carstate.py (501c7de) sets steerWarning unless the EPAS error name
is EAC_ERROR_IDLE. An unknown name is a warning. HANDS_ON (3) is a warning
on that tree. EAC_FAULT is steerFaultPermanent and is not decided here.

AP1 does not treat EAC_ERROR_HIGH_ANGLE_REQ (6) or EAC_ERROR_HANDS_ON (3)
as a temporary fault. Both stay latched after the wheel is released, so
warning on them keeps lat_active false and 0x488 never returns to ANGLE.
The hands-on pause is hands_on_level, not this flag. Every other non-idle
name still warns, matching Tinkla.

Model 3/Y keeps the BogPilot set from before the AP1 exceptions:
IDLE and HANDS_ON are not a temporary fault, and code 6 is.
"""

_BENIGN = ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON")
# AP1: idle, code 6, and latched code 3 so an angle can be sent again.
_AP1_NOT_WARNING = ("EAC_ERROR_IDLE", "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HANDS_ON")
# Latched names that should be commanded as ANGLE once hands are below 2.
AP1_LATCHED_ANGLE_ERRORS = ("EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HANDS_ON")


def steer_fault_temporary(steer_warning, ap1):
  """True when this error name should set CarState.steerFaultTemporary."""
  allowed = _AP1_NOT_WARNING if ap1 else _BENIGN
  return steer_warning not in allowed
