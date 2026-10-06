# Wheelbot: autonomous patrol robot (Raspberry Pi 5 + ROS 2 Jazzy)

Wheelbot is a small wheeled robot that **maps a room with a spinning laser scanner, then drives
itself around a loop of patrol points**. It pauses at each point and repeats forever, steering
around walls and obstacles on the way.

## Hardware

| Part | Job |
|---|---|
| Raspberry Pi 5 (Ubuntu 24.04) | The robot's brain; runs all the software |
| RPLidar (360° laser scanner, 12 m range) | Measures the distance to everything around the robot ~7 times a second |
| Gyro (IMU) | Tells the robot how much it has turned |
| Arduino Uno + motor drivers | Turn speed commands from the Pi into wheel movement |
| Power bank (5 V, 3 A+ USB-C PD) | Powers everything; a weak bank causes most problems |

## How it works

1. **Mapping (SLAM).** `slam_toolbox` combines the laser scans and wheel/gyro movement while the
   robot is driven by keyboard, and saves a floor plan (`<map>.pgm` + `<map>.yaml`).
2. **Localisation (AMCL).** On the saved map, the robot matches its live laser scan against the
   walls to work out where it is, starting from one "2D Pose Estimate".
3. **Navigation (Nav2).** Given a target spot, Nav2 plans a path and steers the wheels along it,
   avoiding anything the laser sees.
4. **Patrol (`patrol.py`).** Sends Nav2 the recorded points in order, waits 5 s at each, loops
   forever, and retries a goal if the robot stalls.
5. **Auto-resume.** A user systemd service (`systemd/wheelbot-patrol.service`) restarts Nav2 and
   the patrol at boot, restoring the last pose saved by `pose_saver.py`.

## Repository layout

| Path | Purpose |
|---|---|
| `scripts/patrol.py` | `record` patrol points (RViz "Publish Point") / run the patrol loop |
| `scripts/autopatrol.sh` | Boot-time bringup + Nav2 (no RViz) + pose restore + patrol |
| `scripts/pose_saver.py` | Keeps `lastpose.json` updated while driving |
| `scripts/renav.sh` | Restart Nav2 keeping the AMCL pose, then restart the patrol |
| `scripts/patrol_headless.sh`, `scripts/plan_patrol.py` | Headless start: scan-match the pose near a prior |
| `scripts/make_nav2_params.sh` | Generates lighter-CPU Nav2 params for the Pi |
| `scripts/nudge.py` | Drive straight ahead/back up to 1 m via a Nav2 goal |
| `scripts/loc_probe.py` | Localisation debugging helper |
| `systemd/wheelbot-patrol.service` | Auto-resume service (`~/.config/systemd/user/`) |
| `docs/event-guide.md` | Step-by-step: connect, check lidar, map, navigate, patrol, troubleshoot |

The code runs from `~/wheelbot` on the Pi.

> **Not uploaded yet:** `start_mapping.sh`, `start_nav.sh`, `stop_all.sh`, `teleop.sh`,
> `lidar_check.py`, `points.yaml`, the maps and the Arduino firmware are still only on the Pi.
> Run `collect_from_pi.ps1 -PiIp <pi-ip>` on the laptop to pull them in.

## Quick start (on the Pi)

```bash
bash ~/wheelbot/start_mapping.sh        # map the room, drive with teleop
ros2 run nav2_map_server map_saver_cli -f ~/wheelbot/event
bash ~/wheelbot/start_nav.sh event      # navigate; set 2D Pose Estimate in RViz
python3 ~/wheelbot/patrol.py record     # click patrol points
python3 ~/wheelbot/patrol.py            # patrol forever
bash ~/wheelbot/stop_all.sh             # stop everything
```

See [docs/event-guide.md](docs/event-guide.md) for the full guide.

## Known limits

- Needs a strong, charged power bank; low power freezes the lidar and drops Wi-Fi.
- Keep RViz closed during patrol: the Pi gets overloaded and goals start failing.
- People and moved furniture are not on the map; a big change to the room means re-mapping.
