from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.cereal import car

p = Params()
CP = None
raw = p.get("CarParams")

if raw:
  CP = messaging.log_from_bytes(raw, car.CarParams)
sm = messaging.SubMaster(['selfdriveState', 'carControl', 'pandaStates'])

while True:
  sm.update(100)
  ss, cc = sm['selfdriveState'], sm['carControl']
  ca = [ps.controlsAllowed for ps in sm['pandaStates']] if sm.seen['pandaStates'] else []

  print(
    f"en={ss.enabled} act={ss.active} "
    f"lat={cc.latActive} long={cc.longActive} "
    f"accel={cc.actuators.accel:.2f} tq={cc.actuators.torque:.2f} "
    f"OPlong={getattr(CP,'openpilotLongitudinalControl',None)} "
    f"pcm={getattr(CP,'pcmCruise',None)} "
    f"panda_ca={ca}"
  )