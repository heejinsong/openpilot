#!/usr/bin/env python3
"""
send_test_plan: stream synthetic waypoint plans from the laptop to the comma.

Standard library only (no ROS2, numpy or openpilot needed), so it runs on any
laptop. Packets use the exact layout waypointsd expects (see plan_msg.py):

  [version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0, ..., x7,y7]   float64 LE
  frame imu_link (x forward, y left), waypoint k at (k+1)*0.5 s

If udp_probe.py runs on the comma it echoes every packet back, and this script
prints round-trip time and loss once per second.

  python3 send_test_plan.py <COMMA_IP>                       # straight, 5 m/s, 20 Hz
  python3 send_test_plan.py <COMMA_IP> --shape arc --curvature 0.02
"""
import argparse
import math
import socket
import struct
import time

PLAN_VERSION = 1.0
DT_WAYPOINT = 0.5
N_WAYPOINTS = 8
ECHO_MAGIC = 4242.0


def make_waypoints(shape: str, speed: float, curvature: float) -> list[tuple[float, float]]:
  pts = []
  for k in range(N_WAYPOINTS):
    s = speed * (k + 1) * DT_WAYPOINT  # arc length travelled at constant speed
    if shape == 'arc' and abs(curvature) > 1e-6:
      pts.append((math.sin(curvature * s) / curvature, (1. - math.cos(curvature * s)) / curvature))
    else:
      pts.append((s, 0.))
  return pts


def pack_plan(pts: list[tuple[float, float]], stamp_ns: int) -> bytes:
  sec, nsec = divmod(stamp_ns, 1_000_000_000)
  data = [PLAN_VERSION, float(sec), float(nsec), float(sec), float(nsec), float(len(pts))]
  for x, y in pts:
    data += [x, y]
  return struct.pack(f'<{len(data)}d', *data)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('comma_ip')
  ap.add_argument('--port', type=int, default=9845)
  ap.add_argument('--hz', type=float, default=20.)
  ap.add_argument('--shape', choices=('straight', 'arc'), default='straight')
  ap.add_argument('--speed', type=float, default=5., help='m/s, sets waypoint spacing')
  ap.add_argument('--curvature', type=float, default=0.01, help='1/m for --shape arc (positive = left)')
  ap.add_argument('--duration', type=float, default=0., help='stop after N seconds (0 = run forever)')
  args = ap.parse_args()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setblocking(False)
  dest = (args.comma_ip, args.port)
  pts = make_waypoints(args.shape, args.speed, args.curvature)
  print(f"sending {args.shape} plan to {dest[0]}:{dest[1]} at {args.hz:g} Hz, Ctrl+C to stop")
  print("  waypoints: " + " ".join(f"({x:.1f},{y:+.2f})" for x, y in pts))

  period = 1. / args.hz
  t_start = time.monotonic()
  next_send = t_start
  t_report = t_start + 1.
  sent = echoed = 0
  win_sent = 0
  win_rtt: list[float] = []

  try:
    while True:
      now = time.monotonic()
      if args.duration and now - t_start > args.duration:
        break
      if now >= next_send:
        sock.sendto(pack_plan(pts, time.time_ns()), dest)
        sent += 1
        win_sent += 1
        next_send += period
        if next_send < now:  # fell behind (e.g. laptop suspended), don't burst
          next_send = now + period

      while True:
        try:
          blob, _ = sock.recvfrom(256)
        except (BlockingIOError, ConnectionResetError):  # Windows reports ICMP port-unreachable as a reset
          break
        if len(blob) == 32:
          magic, t_sec, t_nsec, _recv_wall = struct.unpack('<4d', blob)
          if magic == ECHO_MAGIC:
            echoed += 1
            win_rtt.append(time.time() - (t_sec + t_nsec * 1e-9))

      if now >= t_report:
        t_report = now + 1.
        if win_rtt:
          print(f"sent {win_sent:3d}/s | echoes {len(win_rtt):3d} | RTT min {1e3 * min(win_rtt):.1f} "
                f"avg {1e3 * sum(win_rtt) / len(win_rtt):.1f} max {1e3 * max(win_rtt):.1f} ms")
        else:
          print(f"sent {win_sent:3d}/s | no echoes (fine if udp_probe isn't running, or echo is off)")
        win_sent = 0
        win_rtt = []
      time.sleep(min(0.002, max(0., next_send - time.monotonic())))
  except KeyboardInterrupt:
    pass

  loss = f", {100. * (1 - echoed / sent):.1f}% without echo" if sent and echoed else ""
  print(f"\ntotal: sent {sent}, echoes {echoed}{loss}")


if __name__ == '__main__':
  main()
