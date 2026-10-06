# Waypoint follow mode (external planner -> controlsd)

## On comma
1. Enable **Alpha Longitudinal** and **Waypoint Mode** (Developer settings).
2. Engage with SET/RES as usual.
3. `waypointsd` listens on **UDP port 9845** for planner plans.

Mock (no network): `WAYPOINTS_MOCK=1` in the process environment.

## On AV laptop (ROS2, same WiFi)
```bash
# source ROS2 + your irisav workspace
python3 openpilot/tools/waypoints/ros2_to_comma_waypoints.py --comma-ip <COMMA_IP>
```

Subscribes `/irisav_viewer/planner/trajectory` (`Float64MultiArray`) and UDP-sends the same `data` doubles to comma.

## Packet layout
See `plan_msg.py` — same as `pack_plan` / `unpack_plan`:
`[version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0,...,x7,y7]` in `imu_link`.

No cereal `bridge` required (unlike laptop joystick ZMQ). Firewall must allow UDP 9845 to the comma.
