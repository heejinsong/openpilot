#!/usr/bin/env python3
"""
ros2_to_comma_waypoints: forward the planner trajectory from ROS2 to the comma over UDP.

Runs on the AV laptop (source ROS2 + the irisav workspace first). Subscribes to a
std_msgs/Float64MultiArray whose `data` already has the waypointsd layout

  [version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0, ..., x{n-1},y{n-1}]   (imu_link)

and sends `data` unchanged as little-endian float64 to <comma_ip>:9845.
Malformed messages are dropped here so they never reach the car.

  python3 ros2_to_comma_waypoints.py --comma-ip <COMMA_IP>
"""
import argparse
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

HEADER_LEN = 6
N_WAYPOINTS = 8  # waypointsd rejects any other n


class Forwarder(Node):
  def __init__(self, topic: str, dest: tuple[str, int]):
    super().__init__('comma_waypoint_forwarder')
    self.dest = dest
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sent = self.dropped = 0
    self.t_report = time.monotonic() + 2.
    # Latest-only: a late trajectory is worse than a missing one.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    self.create_subscription(Float64MultiArray, topic, self.on_plan, qos)
    self.get_logger().info(f"forwarding {topic} -> udp://{dest[0]}:{dest[1]}")

  def on_plan(self, msg: Float64MultiArray):
    data = list(msg.data)
    n = int(round(data[5])) if len(data) >= HEADER_LEN else -1
    if n != N_WAYPOINTS or len(data) < HEADER_LEN + 2 * n:
      self.dropped += 1
      self.get_logger().warning(f"dropping malformed plan: len={len(data)} n={n} (expected n={N_WAYPOINTS})",
                                throttle_duration_sec=2.)
    else:
      data = data[:HEADER_LEN + 2 * n]
      self.sock.sendto(struct.pack(f'<{len(data)}d', *data), self.dest)
      self.sent += 1

    now = time.monotonic()
    if now >= self.t_report:
      self.get_logger().info(f"sent {self.sent / 2.:.1f} Hz, dropped {self.dropped}")
      self.sent = self.dropped = 0
      self.t_report = now + 2.


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--comma-ip', required=True)
  ap.add_argument('--port', type=int, default=9845)
  ap.add_argument('--topic', default='/irisav_viewer/planner/trajectory')
  args = ap.parse_args()

  rclpy.init()
  node = Forwarder(args.topic, (args.comma_ip, args.port))
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()
