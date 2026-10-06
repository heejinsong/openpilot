#!/usr/bin/env python3
"""
Unpack / pack irisav planner Float64MultiArray layout.

data = [version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0, y0, ..., x{n-1}, y{n-1}]
Frame: imu_link (x forward, y left), meters.
Waypoint k is targeted at plan_t + (k + 1) * 0.5 s.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

PLAN_VERSION = 1.0
DT_WAYPOINT = 0.5
HEADER_LEN = 6  # version, t_sec, t_nsec, plan_sec, plan_nsec, n


@dataclass
class UnpackedPlan:
  version: float
  t_sec: float
  t_nsec: float
  plan_sec: float
  plan_nsec: float
  n: int
  x: np.ndarray  # (n,) forward
  y: np.ndarray  # (n,) left

  @property
  def plan_t_s(self) -> float:
    return float(self.plan_sec) + float(self.plan_nsec) * 1e-9

  @property
  def odom_t_s(self) -> float:
    return float(self.t_sec) + float(self.t_nsec) * 1e-9

  def waypoint_times_from_plan(self) -> np.ndarray:
    """Absolute-ish times relative to plan stamp: (k+1)*0.5."""
    return (np.arange(self.n, dtype=float) + 1.0) * DT_WAYPOINT


def unpack_plan(data) -> UnpackedPlan:
  arr = np.asarray(data, dtype=float).reshape(-1)
  if arr.size < HEADER_LEN:
    raise ValueError(f"plan too short: {arr.size}")
  version, t_sec, t_nsec, plan_sec, plan_nsec, n_f = arr[:HEADER_LEN]
  n = int(round(float(n_f)))
  if n < 1:
    raise ValueError(f"invalid n={n}")
  need = HEADER_LEN + 2 * n
  if arr.size < need:
    raise ValueError(f"plan size {arr.size} < expected {need}")
  xy = arr[HEADER_LEN:need].reshape(n, 2)
  return UnpackedPlan(
    version=float(version),
    t_sec=float(t_sec),
    t_nsec=float(t_nsec),
    plan_sec=float(plan_sec),
    plan_nsec=float(plan_nsec),
    n=n,
    x=xy[:, 0].copy(),
    y=xy[:, 1].copy(),
  )


def pack_plan_bytes(data) -> bytes:
  """Serialize plan doubles as little-endian float64 blob for UDP."""
  arr = np.asarray(data, dtype=np.float64).reshape(-1)
  return arr.tobytes(order='C')


def unpack_plan_bytes(blob: bytes) -> UnpackedPlan:
  n_doubles = len(blob) // 8
  arr = np.frombuffer(blob, dtype='<f8', count=n_doubles)
  return unpack_plan(arr)
