#!/usr/bin/env python3
"""Print Alpha Longitudinal / CarParams state for keyboard joystick testing."""
from openpilot.common.params import Params
from openpilot.cereal import car, messaging

p = Params()

print("AlphaLongitudinalEnabled=", p.get_bool("AlphaLongitudinalEnabled"))
print("JoystickDebugMode=", p.get_bool("JoystickDebugMode"))
print("IsOffroad=", p.get_bool("IsOffroad"))

raw = p.get("CarParams")
if raw is None:
  print("CarParams: None (onroad fingerprint 후 생성)")
else:
  CP = messaging.log_from_bytes(raw, car.CarParams)
  print("carFingerprint=", CP.carFingerprint)
  print("flags=", CP.flags)
  print("alphaLongitudinalAvailable=", CP.alphaLongitudinalAvailable)
  print("openpilotLongitudinalControl=", CP.openpilotLongitudinalControl)
  print("pcmCruise=", CP.pcmCruise)
  print("minEnableSpeed=", CP.minEnableSpeed, "minSteerSpeed=", CP.minSteerSpeed)
  print("steerAtStandstill=", CP.steerAtStandstill)
  for i, sc in enumerate(CP.safetyConfigs):
    print(f"safetyConfigs[{i}].safetyModel=", sc.safetyModel, "safetyParam=", sc.safetyParam)

  ok = (
    CP.carFingerprint == "HYUNDAI_IONIQ_5"
    and CP.alphaLongitudinalAvailable
    and p.get_bool("AlphaLongitudinalEnabled")
    and CP.openpilotLongitudinalControl
    and not CP.pcmCruise
  )
  print("GO_FOR_OP_LONG_JOYSTICK=", ok)
