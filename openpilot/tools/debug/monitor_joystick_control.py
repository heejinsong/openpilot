#!/usr/bin/env python3
"""Live monitor for keyboard joystick → joystickd → carControl path."""
from openpilot.cereal import messaging, car
from openpilot.common.params import Params

p = Params()
sm = messaging.SubMaster(['selfdriveState', 'carControl', 'pandaStates', 'carState'])

def load_cp():
  raw = p.get("CarParams")
  if raw is None:
    return None
  return messaging.log_from_bytes(raw, car.CarParams)

CP = load_cp()
n = 0
while True:
  sm.update(100)
  n += 1
  if n % 20 == 0:  # refresh CarParams ~2s
    CP = load_cp()

  ss, cc = sm['selfdriveState'], sm['carControl']
  ca = [ps.controlsAllowed for ps in sm['pandaStates']] if sm.seen['pandaStates'] else []
  v = sm['carState'].vEgo if sm.seen['carState'] else float('nan')

  print(
    f"v={v:5.2f} "
    f"en={int(ss.enabled)} act={int(ss.active)} "
    f"lat={int(cc.latActive)} long={int(cc.longActive)} "
    f"accel={cc.actuators.accel:+5.2f} tq={cc.actuators.torque:+5.2f} "
    f"OPlong={getattr(CP, 'openpilotLongitudinalControl', None)} "
    f"pcm={getattr(CP, 'pcmCruise', None)} "
    f"panda_ca={ca}"
  )
