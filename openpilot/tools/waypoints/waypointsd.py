#!/usr/bin/env python3
"""
waypointsd: external timed waypoints (imu_link) -> lat + long setpoints for controlsd.

Input: UDP Float64MultiArray layout (x,y only) from the laptop ROS2 bridge.
  [version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0, ..., x7,y7]
  frame = imu_link (x forward, y left), ~20 Hz

Smoothing (same (x,y) plan, nicer κ/a):
  - speed-proportional lookahead time
  - path-speed estimate + light moving average + curvature speed cap
  - EMA + rate limits (jerk) on κ and a

Fallback: WAYPOINTS_MOCK=1 -> straight-line mock.
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
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, should_stop, smooth_value
from openpilot.tools.waypoints.plan_msg import (
  DT_WAYPOINT,
  UnpackedPlan,
  unpack_plan_bytes,
)

N_WAYPOINTS = 8
T_WAYPOINTS = (np.arange(N_WAYPOINTS, dtype=float) + 1.0) * DT_WAYPOINT  # 0.5 .. 4.0

# Lookahead time scales with speed: t = clip(L/v), L = clip(gain*v, Lmin, Lmax)
LOOKAHEAD_GAIN = 1.0          # seconds of "distance gain" ~ L ≈ v * gain at mid speeds
LOOKAHEAD_MIN_M = 3.0
LOOKAHEAD_MAX_M = 12.0
ACTION_T_MIN = 0.35
ACTION_T_MAX = 1.2

LAT_SMOOTH_SECONDS = 0.2
LONG_SMOOTH_SECONDS = 0.4

# Rate limits after EMA (comfort)
MAX_KAPPA_RATE = 0.12          # 1/m per second
MAX_ACCEL_JERK = 1.2           # m/s^3

# Soft lateral accel cap used only to trim path-derived speeds in bends
A_LAT_MAX = 1.8               # m/s^2

MIN_LOOKAHEAD = 1.0
WAYPOINT_TIMEOUT_S = 0.5

UDP_PORT = int(os.environ.get("WAYPOINTS_UDP_PORT", "9845"))
USE_MOCK = os.environ.get("WAYPOINTS_MOCK", "0") == "1"


def lookahead_time(v_ego: float) -> float:
  """Speed-proportional lookahead horizon along the timed plan."""
  v = max(v_ego, MIN_SPEED)
  L = float(np.clip(LOOKAHEAD_GAIN * v, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M))
  return float(np.clip(L / v, ACTION_T_MIN, ACTION_T_MAX))


def rate_limit(new: float, prev: float, max_rate: float, dt: float) -> float:
  if dt <= 0 or max_rate <= 0:
    return new
  return float(prev + np.clip(new - prev, -max_rate * dt, max_rate * dt))


def curvature_from_local_waypoints(t: np.ndarray, x: np.ndarray, y: np.ndarray,
                                  action_t: float) -> tuple[float, float, float]:
  x_t = float(np.interp(action_t, t, x))
  y_t = float(np.interp(action_t, t, y))
  L = max(math.hypot(x_t, y_t), MIN_LOOKAHEAD)
  alpha = math.atan2(y_t, x_t)
  kappa = 2.0 * math.sin(alpha) / L
  return float(kappa), x_t, y_t


def _segment_curvatures(x: np.ndarray, y: np.ndarray) -> np.ndarray:
  """Rough |κ| at each waypoint from consecutive triples (endpoints copy neighbor)."""
  n = len(x)
  kappa = np.zeros(n, dtype=float)
  if n < 3:
    return kappa
  for i in range(1, n - 1):
    x0, y0 = float(x[i - 1]), float(y[i - 1])
    x1, y1 = float(x[i]), float(y[i])
    x2, y2 = float(x[i + 1]), float(y[i + 1])
    a = math.hypot(x1 - x0, y1 - y0)
    b = math.hypot(x2 - x1, y2 - y1)
    c = math.hypot(x2 - x0, y2 - y0)
    area2 = abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
    denom = a * b * c
    kappa[i] = (2.0 * area2 / denom) if denom > 1e-3 else 0.0
  kappa[0] = kappa[1]
  kappa[-1] = kappa[-2]
  return kappa


def speeds_from_path(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
  """Path speeds from (x,y,t), then MA + curvature-based speed cap for comfort."""
  v = np.zeros(len(t), dtype=float)
  px, py, pt = 0.0, 0.0, 0.0
  for i in range(len(t)):
    dt = max(float(t[i] - pt), 1e-3)
    v[i] = math.hypot(float(x[i]) - px, float(y[i]) - py) / dt
    px, py, pt = float(x[i]), float(y[i]), float(t[i])

  # 3-point moving average to kill segment noise
  v_s = v.copy()
  for i in range(len(v)):
    i0, i1 = max(0, i - 1), min(len(v) - 1, i + 1)
    v_s[i] = float(np.mean(v[i0:i1 + 1]))

  # Limit speed in bends: v <= sqrt(a_lat_max / |κ|)
  kappa = _segment_curvatures(x, y)
  for i in range(len(v_s)):
    k = abs(float(kappa[i]))
    if k > 1e-4:
      v_s[i] = min(v_s[i], math.sqrt(A_LAT_MAX / k))

  return v_s


def accel_from_local_speeds(t: np.ndarray, v: np.ndarray, v_ego: float, a_ego: float,
                           action_t: float) -> float:
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
    self.prev_action_mono: float | None = None

  def update_from_plan(self, plan: UnpackedPlan, recv_mono: float | None = None) -> None:
    if plan.n != N_WAYPOINTS:
      raise ValueError(f"expected n={N_WAYPOINTS}, got {plan.n}")

    now = time.monotonic() if recv_mono is None else recv_mono
    self.t = plan.waypoint_times_from_plan()
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
      self.prev_action_mono = None
      return None

    dt = DT_MDL if self.prev_action_mono is None else max(now - self.prev_action_mono, 1e-3)
    self.prev_action_mono = now

    age = now - self.last_recv_mono
    t_now = self.t - age
    if t_now[-1] <= 0.05:
      return None
    t_use = np.maximum(t_now, 1e-3)

    action_t = lookahead_time(v_ego)
    # Stay inside remaining horizon
    action_t = float(min(action_t, float(t_use[-1]) * 0.95))

    kappa_raw, _, _ = curvature_from_local_waypoints(t_use, self.x, self.y, action_t)
    a_raw = accel_from_local_speeds(t_use, self.v, v_ego, a_ego, action_t)
    a_raw = float(np.clip(a_raw, ACCEL_MIN, ACCEL_MAX))

    # EMA then rate-limit (jerk) for comfort
    kappa = float(smooth_value(kappa_raw, self.prev_kappa, LAT_SMOOTH_SECONDS, dt=dt))
    a = float(smooth_value(a_raw, self.prev_a, LONG_SMOOTH_SECONDS, dt=dt))
    kappa = rate_limit(kappa, self.prev_kappa, MAX_KAPPA_RATE, dt)
    a = rate_limit(a, self.prev_a, MAX_ACCEL_JERK, dt)
    a = float(np.clip(a, ACCEL_MIN, ACCEL_MAX))

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

  tracker = WaypointTracker() # plan 저장
  udp_sock = None if USE_MOCK else _open_udp(UDP_PORT) # mock이 아닐때만 9845 listen
  if USE_MOCK:
    cloudlog.warning("waypointsd WAYPOINTS_MOCK=1 — using straight mock, not ROS2")

  while True:
    sm.update()

    CS = sm['carState']
    v_ego = max(CS.vEgo, 0.0) # 현재 속도
    a_ego = float(CS.aEgo) # 현재 가속도

    if USE_MOCK:
      x, y, v = _mock_straight_waypoints(v_ego)
      tracker.update_local_waypoints(x, y, v=v)
    elif udp_sock is not None:
      _drain_udp(udp_sock, tracker)

    # k, a 계산
    actions = tracker.get_actions(v_ego, a_ego) # fresh면 k, a 계산
    lat_active = bool(sm['carControl'].latActive)
    age = (time.monotonic() - tracker.last_recv_mono) if tracker.has_waypoints else float("inf")
    action_t = lookahead_time(v_ego)

    # 화면 debug
    alert_msg = messaging.new_message('alertDebug')
    alert_msg.valid = True
    if actions is None:
      alert_msg.alertDebug.alertText1 = 'waypointsd: waiting for plan'
      alert_msg.alertDebug.alertText2 = f'udp:{UDP_PORT} age={age:.2f}s mock={int(USE_MOCK)}'
      kappa, a_target = 0.0, 0.0
    else:
      kappa, a_target = actions
      alert_msg.alertDebug.alertText1 = f'wp κ={kappa:+.4f} a={a_target:+.2f}'
      alert_msg.alertDebug.alertText2 = f'v={v_ego * 3.6:.1f}km/h t_la={action_t:.2f}s age={age * 1000:.0f}ms'
    pm.send('alertDebug', alert_msg)

    # lateralPlan 계산
    lat_msg = messaging.new_message('lateralManeuverPlan')
    lat_msg.valid = actions is not None and lat_active
    if lat_msg.valid:
      lat_msg.lateralManeuverPlan.desiredCurvature = float(kappa)
    pm.send('lateralManeuverPlan', lat_msg)

    # longitudinalPlan 계산
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
