# Waypoint follow mode (external planner -> controlsd)

Laptop planner --UDP 9845--> `waypointsd` (comma) --> `lateralManeuverPlan` + `longitudinalPlan` --> `controlsd`

| file | runs on | what |
|---|---|---|
| `waypointsd.py` | comma | the real consumer, started by manager when **WaypointMode** is on and onroad |
| `udp_probe.py` | comma | link check: decodes and reports what arrives on the port, echoes back |
| `send_test_plan.py` | laptop | synthetic plans (straight / arc), stdlib only, reports RTT from echoes |
| `ros2_to_comma_waypoints.py` | laptop | forwards the ROS2 planner topic to the comma |
| `plan_msg.py` | both | packet layout |

## Packet layout
Little-endian float64: `[version, t_sec, t_nsec, plan_sec, plan_nsec, n, x0,y0,...,x7,y7]`,
frame `imu_link` (x forward, y left, meters), waypoint k at `(k+1)*0.5 s`. `waypointsd` requires `n = 8`.

## Step 1: check the link (no car needed)
Laptop and comma on the same network. Find the comma IP in Settings > Network, or `ip -4 addr` over SSH.

On the comma (SSH), with openpilot offroad or WaypointMode off so the port is free:
```bash
cd /data/openpilot && python3 openpilot/tools/waypoints/udp_probe.py
```
On the laptop:
```bash
python3 openpilot/tools/waypoints/send_test_plan.py <COMMA_IP>                      # synthetic
python3 openpilot/tools/waypoints/ros2_to_comma_waypoints.py --comma-ip <COMMA_IP>  # real planner
```
Good link: probe shows ~20 Hz, `0 bad`, `stale gaps total 0`, and the sender shows RTT in the
low tens of ms. If the probe stays at `waiting...`, check the IP, that both are on the same
subnet, and the laptop firewall (outgoing UDP 9845, plus incoming replies for the echo).

## Step 2: in the car
1. Enable **Alpha Longitudinal** and **Waypoint Mode** (Developer settings).
2. Engage with SET/RES as usual.
3. Stream plans. The onroad debug alert shows `wp κ=... a=...` with the plan age; it shows
   `waypointsd: waiting for plan` when nothing fresh (< 0.5 s) has arrived.

Mock (no network): `WAYPOINTS_MOCK=1` in the process environment.
