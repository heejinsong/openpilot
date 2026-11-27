#!/usr/bin/env python3
"""
udp_probe: check that waypoint plans from the laptop actually reach the comma.

Run this ON THE COMMA (over SSH) while the laptop runs send_test_plan.py or the
ROS2 bridge. It binds the same UDP port as waypointsd, decodes every packet with
the same parser (plan_msg.unpack_plan_bytes), and once per second prints:

  rate, inter-arrival gaps, decode errors, how often waypointsd would have gone
  stale (gap > 0.5 s), the latest waypoints, and the clock offset between the
  plan timestamp and the comma's clock.

By default it echoes a small reply to the sender so send_test_plan.py can report
round-trip time. Disable with --no-echo.

  cd /data/openpilot && python3 openpilot/tools/waypoints/udp_probe.py

waypointsd must not be running at the same time (it owns the port when
WaypointMode is on and the car is onroad), otherwise the bind fails.
"""
import argparse
import errno
import os
import select
import socket
import struct
import sys
import time

try:
  from openpilot.tools.waypoints.plan_msg import unpack_plan_bytes
except ImportError:  # running outside the openpilot PYTHONPATH (e.g. loopback test on a laptop)
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from plan_msg import unpack_plan_bytes  # type: ignore[no-redef]

# Mirrors of waypointsd constants (not imported: waypointsd pulls in cereal/params).
N_WAYPOINTS_EXPECTED = 8   # waypointsd.N_WAYPOINTS
WAYPOINT_TIMEOUT_S = 0.5   # waypointsd.WAYPOINT_TIMEOUT_S
ECHO_MAGIC = 4242.0       # first double of an echo reply (see send_test_plan.py)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--port', type=int, default=int(os.environ.get('WAYPOINTS_UDP_PORT', '9845')))
  ap.add_argument('--no-echo', action='store_true', help="don't reply to the sender")
  ap.add_argument('--duration', type=float, default=0., help='stop after N seconds (0 = run forever)')
  args = ap.parse_args()
  sys.stdout.reconfigure(line_buffering=True)  # type: ignore[union-attr]  # keep output live when piped to a file

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    sock.bind(('0.0.0.0', args.port))
  except OSError as e:
    if e.errno == errno.EADDRINUSE:
      print(f"UDP port {args.port} is already in use. Is waypointsd running (WaypointMode on and onroad)?\n"
            f"Turn WaypointMode off or go offroad, then retry.")
      sys.exit(1)
    raise
  sock.setblocking(False)
  print(f"listening on UDP 0.0.0.0:{args.port} (echo {'off' if args.no_echo else 'on'}), Ctrl+C to stop")

  t_start = time.monotonic()
  t_report = t_start + 1.
  last_recv = None
  total_ok = total_bad = total_stale = 0
  win_ok = win_bad = 0
  win_gaps: list[float] = []
  senders: set[str] = set()
  last_plan = None
  last_offset = None

  try:
    while True:
      now = time.monotonic()
      if args.duration and now - t_start > args.duration:
        break
      ready, _, _ = select.select([sock], [], [], max(0., t_report - now))
      if ready:
        while True:
          try:
            blob, addr = sock.recvfrom(4096)
          except BlockingIOError:
            break
          recv_mono, recv_wall = time.monotonic(), time.time()
          senders.add(f"{addr[0]}:{addr[1]}")
          try:
            plan = unpack_plan_bytes(blob)
          except ValueError as e:
            total_bad += 1
            win_bad += 1
            print(f"  bad packet from {addr[0]} ({len(blob)} bytes): {e}")
            continue
          if plan.n != N_WAYPOINTS_EXPECTED:
            print(f"  warning: n={plan.n}, waypointsd expects n={N_WAYPOINTS_EXPECTED} and will reject this plan")
          if last_recv is not None:
            gap = recv_mono - last_recv
            win_gaps.append(gap)
            if gap > WAYPOINT_TIMEOUT_S:
              total_stale += 1
          last_recv = recv_mono
          total_ok += 1
          win_ok += 1
          last_plan = plan
          last_offset = recv_wall - plan.odom_t_s
          if not args.no_echo:
            sock.sendto(struct.pack('<4d', ECHO_MAGIC, plan.t_sec, plan.t_nsec, recv_wall), addr)

      now = time.monotonic()
      if now >= t_report:
        t_report = now + 1.
        if win_ok == 0 and win_bad == 0:
          since = f"last packet {now - last_recv:.1f} s ago" if last_recv is not None else "nothing received yet"
          print(f"waiting... ({since})")
        else:
          gaps = f"gap avg {1e3 * sum(win_gaps) / len(win_gaps):.0f} ms max {1e3 * max(win_gaps):.0f} ms" if win_gaps else "gap n/a"
          print(f"{win_ok:3d} Hz ok, {win_bad} bad | {gaps} | stale gaps total {total_stale} | from {', '.join(sorted(senders))}")
          if last_plan is not None:
            xs, ys = last_plan.x, last_plan.y
            print(f"     n={last_plan.n} v{last_plan.version:g}  wp0=({xs[0]:+.2f},{ys[0]:+.2f})  "
                  f"wp{last_plan.n - 1}=({xs[-1]:+.2f},{ys[-1]:+.2f}) m  "
                  f"stamp->recv {1e3 * last_offset:+.0f} ms (includes laptop/comma clock offset)")
        win_ok = win_bad = 0
        win_gaps = []
  except KeyboardInterrupt:
    pass

  print(f"\ntotal: {total_ok} ok, {total_bad} bad, {total_stale} gaps > {WAYPOINT_TIMEOUT_S} s")


if __name__ == '__main__':
  main()
