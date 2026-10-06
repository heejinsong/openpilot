#!/usr/bin/env python3
"""
waypointsd: external timed waypoints (imu_link) -> lat + long setpoints for controlsd.

Input (preferred): UDP packets from the AV laptop ROS2 bridge
  tools/waypoints/ros2_to_comma_waypoints.py
  payload = Float64MultiArray.data as little-endian float64[]
  [version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0, ..., x7,y7]
  frame = imu_link (x forward, y left), ~20 Hz

Fallback: WAYPOINTS_MOCK=1 -> straight-line mock (debug only).

When WaypointMode is on, plannerd is stopped; this process owns longitudinalPlan.
"""
from __future__ import annotations

import math
import os
import socket
import time

import numpy as np

from openpilot.cereal import messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, should_stop, smooth_value
from openpilot.tools.waypoints.plan_msg import (
  DT_WAYPOINT,
  UnpackedPlan,
  unpack_plan_bytes,
)

N_WAYPOINTS = 8
T_WAYPOINTS = (np.arange(N_WAYPOINTS, dtype=float) + 1.0) * DT_WAYPOINT  # 0.5 .. 4.0

ACTION_T = 0.5
LAT_SMOOTH_SECONDS = 0.1
LONG_SMOOTH_SECONDS = 0.3
MIN_LOOKAHEAD = 1.0

# No new plan within this window -> stop using waypoints (lat falls back to model)
WAYPOINT_TIMEOUT_S = 0.35

UDP_PORT = int(os.environ.get("WAYPOINTS_UDP_PORT", "9845"))
USE_MOCK = os.environ.get("WAYPOINTS_MOCK", "0") == "1"


def curvature_from_local_waypoints(t: np.ndarray, x: np.ndarray, y: np.ndarray,
                                  action_t: float = ACTION_T) -> tuple[float, float, float]:
  x_t = float(np.interp(action_t, t, x))
  y_t = float(np.interp(action_t, t, y))
  L = max(math.hypot(x_t, y_t), MIN_LOOKAHEAD)
  alpha = math.atan2(y_t, x_t)
  kappa = 2.0 * math.sin(alpha) / L
  return float(kappa), x_t, y_t


def speeds_from_path(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
  v = np.zeros(len(t), dtype=float)
  px, py, pt = 0.0, 0.0, 0.0
  for i in range(len(t)):
    dt = max(float(t[i] - pt), 1e-3)
    v[i] = math.hypot(float(x[i]) - px, float(y[i]) - py) / dt
    px, py, pt = float(x[i]), float(y[i]), float(t[i])
  return v


def accel_from_local_speeds(t: np.ndarray, v: np.ndarray, v_ego: float, a_ego: float,
                           action_t: float = ACTION_T) -> float:
  v_t = float(np.interp(action_t, t, v))
  return float(2.0 * (v_t - v_ego) / action_t - a_ego)


class WaypointTracker:
  def __init__(self):
    self.t = T_WAYPOINTS.copy()
    self.x = np.zeros(N_WAYPOINTS, dtype=float)
    self.y = np.zeros(N_WAYPOINTS, dtype=float)
    self.v = np.zeros(N_WAYPOINTS, dtype=float)
    self.has_waypoints = False
    self.last_recv_mono = 0.0
    self.prev_kappa = 0.0
    self.prev_a = 0.0

  def update_from_plan(self, plan: UnpackedPlan, recv_mono: float | None = None) -> None:
    """Apply irisav plan; times are relative to plan stamp, shifted by packet age."""
    if plan.n != N_WAYPOINTS:
      # Allow n!=8 by resampling onto default horizon if needed later; for now require 8.
      raise ValueError(f"expected n={N_WAYPOINTS}, got {plan.n}")

    now = time.monotonic() if recv_mono is None else recv_mono
    # Clocks on laptop vs comma are not assumed synced; only use local receive age.
    # Treat plan as generated ~now when packet arrives; age grows until next packet.
    t_rel = plan.waypoint_times_from_plan()  # 0.5 .. 4.0 at receive instant
    self.t = t_rel
    self.x = plan.x.astype(float)
    self.y = plan.y.astype(float)
    self.v = speeds_from_path(self.t, self.x, self.y)
    self.has_waypoints = True
    self.last_recv_mono = now

  def update_local_waypoints(self, x: np.ndarray, y: np.ndarray,
                             t: np.ndarray | None = None,
                             v: np.ndarray | None = None) -> None:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    assert x.shape == y.shape == (N_WAYPOINTS,)
    self.t = T_WAYPOINTS.copy() if t is None else np.asarray(t, dtype=float).reshape(-1)
    self.x, self.y = x, y
    self.v = speeds_from_path(self.t, self.x, self.y) if v is None else np.asarray(v, dtype=float)
    self.has_waypoints = True
    self.last_recv_mono = time.monotonic()

  def fresh(self, now: float | None = None) -> bool:
    if not self.has_waypoints:
      return False
    now = time.monotonic() if now is None else now
    return (now - self.last_recv_mono) <= WAYPOINT_TIMEOUT_S

  def get_actions(self, v_ego: float, a_ego: float) -> tuple[float, float] | None:
    now = time.monotonic()
    if not self.fresh(now):
      return None

    # Advance time base by packet age so ACTION_T is "from now"
    age = now - self.last_recv_mono
    t_now = self.t - age
    if t_now[-1] <= 0.05:
      return None
    # Keep interp domain valid: clamp negative times to small epsilon for speed/path at "now"
    t_use = np.maximum(t_now, 1e-3)

    kappa, _, _ = curvature_from_local_waypoints(t_use, self.x, self.y, ACTION_T)
    a = accel_from_local_speeds(t_use, self.v, v_ego, a_ego, ACTION_T)
    a = float(np.clip(a, ACCEL_MIN, ACCEL_MAX))

    kappa = float(smooth_value(kappa, self.prev_kappa, LAT_SMOOTH_SECONDS))
    a = float(smooth_value(a, self.prev_a, LONG_SMOOTH_SECONDS))
    self.prev_kappa, self.prev_a = kappa, a
    return kappa, a


def _mock_straight_waypoints(v_ego: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  v = max(v_ego, MIN_SPEED)
  x = T_WAYPOINTS * v
  y = np.zeros_like(x)
  return x, y, np.full_like(x, v)


def _open_udp(port: int) -> socket.socket:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind(("0.0.0.0", port))
  sock.setblocking(False)
  cloudlog.info(f"waypointsd listening UDP 0.0.0.0:{port}")
  return sock


def _drain_udp(sock: socket.socket, tracker: WaypointTracker) -> bool:
  """Read latest UDP plan; return True if at least one valid plan applied."""
  got = False
  while True:
    try:
      blob, _addr = sock.recvfrom(4096)
    except BlockingIOError:
      break
    try:
      plan = unpack_plan_bytes(blob)
      tracker.update_from_plan(plan)
      got = True
    except ValueError as e:
      cloudlog.warning(f"waypointsd bad UDP plan: {e}")
  return got


def main():
  params = Params()
  cloudlog.info("waypointsd is waiting for CarParams")
  messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState', 'modelV2'], poll='modelV2')
  pm = messaging.PubMaster(['lateralManeuverPlan', 'longitudinalPlan', 'alertDebug'])

  tracker = WaypointTracker()
  udp_sock = None if USE_MOCK else _open_udp(UDP_PORT)
  if USE_MOCK:
    cloudlog.warning("waypointsd WAYPOINTS_MOCK=1 — using straight mock, not ROS2")

  while True:
    sm.update()

    CS = sm['carState']
    v_ego = max(CS.vEgo, 0.0)
    a_ego = float(CS.aEgo)

    if USE_MOCK:
      x, y, v = _mock_straight_waypoints(v_ego)
      tracker.update_local_waypoints(x, y, v=v)
    elif udp_sock is not None:
      _drain_udp(udp_sock, tracker)

    actions = tracker.get_actions(v_ego, a_ego)
    lat_active = bool(sm['carControl'].latActive)
    age = (time.monotonic() - tracker.last_recv_mono) if tracker.has_waypoints else float("inf")

    alert_msg = messaging.new_message('alertDebug')
    alert_msg.valid = True
    if actions is None:
      alert_msg.alertDebug.alertText1 = 'waypointsd: waiting for plan'
      alert_msg.alertDebug.alertText2 = f'udp:{UDP_PORT} age={age:.2f}s mock={int(USE_MOCK)}'
      kappa, a_target = 0.0, 0.0
    else:
      kappa, a_target = actions
      alert_msg.alertDebug.alertText1 = f'wp κ={kappa:+.4f} a={a_target:+.2f}'
      alert_msg.alertDebug.alertText2 = f'v={v_ego * 3.6:.1f}km/h age={age*1000:.0f}ms'
    pm.send('alertDebug', alert_msg)

    lat_msg = messaging.new_message('lateralManeuverPlan')
    lat_msg.valid = actions is not None and lat_active
    if lat_msg.valid:
      lat_msg.lateralManeuverPlan.desiredCurvature = float(kappa)
    pm.send('lateralManeuverPlan', lat_msg)

    # On timeout: still publish long with aTarget=0 (plannerd is off) — coast / hold
    long_msg = messaging.new_message('longitudinalPlan')
    long_msg.valid = True
    long_plan = long_msg.longitudinalPlan
    long_plan.aTarget = float(a_target) if actions is not None else 0.0
    long_plan.shouldStop = bool(should_stop(v_ego, long_plan.aTarget))
    long_plan.allowBrake = True
    long_plan.allowThrottle = bool(actions is not None)
    long_plan.hasLead = False
    long_plan.speeds = [0.2]
    pm.send('longitudinalPlan', long_msg)


if __name__ == '__main__':
  main()
