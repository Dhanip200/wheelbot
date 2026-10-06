# Event day: map the venue + patrol (Pi 5 wheelbot)

Robot code lives in `~/wheelbot` on the Pi. Full build/wiring notes: `~/wheelbot/README.md`.

| | |
|---|---|
| Pi user / host | `<pi-user>` / `<pi-hostname>` |
| Wi-Fi | `<your-hotspot>` (Pi joins it automatically) |
| Last known IP | `<pi-ip>` (can change - see step 1) |
| Remote screen | Windows **Remote Desktop** (RDP, port 3389). This Ubuntu has no VNC server; RDP is the "VNC". |

---

## 1. Connect to the Pi

1. Turn on the <your-hotspot> hotspot, then power the Pi. Wait ~1-2 min for Ubuntu to boot.
2. Find the Pi's IP: phone hotspot -> **Connected devices** (the Pi hostname), or on the Pi run `hostname -I`.
3. Laptop: `Win + R` -> `mstsc /v:<pi-ip>` -> login `<pi-user>` + password.
4. Terminal access: `ssh <pi-user>@<pi-ip>` (or open a terminal inside Remote Desktop).

Remote Desktop is always on: it starts at boot, the Pi auto-logs in as your user, and screen lock,
screen blank, idle suspend and Wi-Fi power saving are all turned off. Nothing to start by hand.
If Remote Desktop stops answering but SSH works, restart it:
```bash
systemctl --user restart gnome-remote-desktop.service
```

## Power (read this first)

A flat or weak power bank was behind most of the problems on 2026-10-03: the Pi dropping off
Wi-Fi, the lidar freezing / spinning slow, and `M1 NO RESPONSE` motor warnings.
- Pi 5 + lidar + Uno needs **5 V at 3 A or more over USB-C PD** (5 A is ideal). Many power banks can't.
- Bring a **fully charged bank + a spare**. Map on a wall supply if you can.
- Check after a run: `cat /sys/devices/platform/soc/soc:firmware/get_throttled` must print `0`.
  Anything else (e.g. `0x50000`) = the Pi was under-powered since boot.

## 2. Check the lidar (stop ROS first)

```bash
bash ~/wheelbot/stop_all.sh
python3 ~/wheelbot/lidar_check.py
```
Need `health answered 10/10` and `scan ~5.5-7 rotations/s`. Readings of 4.7-5.0 were seen on
2026-10-03; if it's lower than that, reseat the lidar USB cable and check the Pi power supply.

## 3. Map the venue (SLAM)

```bash
bash ~/wheelbot/start_mapping.sh
```
Keep the robot **still for ~5 s** while it starts (gyro calibration). The script runs the lidar
check first (un-sticks a frozen lidar driver), then opens RViz and a **TELEOP** window on the Pi
screen (see it in Remote Desktop).

Check it's really mapping: in RViz the red laser dots must show and the grey map must grow.
No dots = no lidar data -> `bash ~/wheelbot/stop_all.sh` and run `start_mapping.sh` again.

Drive: click the TELEOP window, then
`i` forward, `,` back, `j`/`l` turn, `u`/`o` curve, `k` stop, `z` slower, `q` faster.

Less lag: drive from a laptop PowerShell window over SSH instead (only use one teleop at a time):
```
ssh -t <pi-user>@<pi-ip> bash ~/wheelbot/teleop.sh
```

Tips for a clean map:
- Drive slowly along every wall; turn slowly.
- Drive back through places you've already mapped so SLAM can close loops.
- Do it before the crowd arrives - people show up as walls.

Save the map when it looks complete (in a second terminal):
```bash
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/wheelbot/event
bash ~/wheelbot/stop_all.sh
```
That writes `~/wheelbot/event.pgm` + `event.yaml`. The old `room` map is untouched.

Saved maps so far: `room` (2026-10-01), `test_map` (2026-10-03 practice run).
Use any of them with `bash ~/wheelbot/start_nav.sh <name>`.

## 4. Navigate on the event map

```bash
bash ~/wheelbot/start_nav.sh event
```
Keep it still ~5 s again. Then in RViz:
1. **2D Pose Estimate** - click where the robot really is, drag the way it faces. The red laser dots
   must snap onto the map walls. (Nav2 waits up to 10 min for this.)
2. Test with **Nav2 Goal** - click a spot + drag a direction; the robot should drive there.

## 5. Patrol

The patrol points in `points.yaml` belong to the room map, so record new ones for the event map.
Back up the room points first:
```bash
cp ~/wheelbot/points.yaml ~/wheelbot/points_room.yaml
python3 ~/wheelbot/patrol.py record    # RViz top bar: "Publish Point", click 4 spots in order
python3 ~/wheelbot/patrol.py           # loops forever, 5 s stop at each point
```
- "Publish Point" alone does nothing - `patrol.py record` must be running to catch the clicks.
- Re-select "Publish Point" before every click (RViz switches back after each one).
- Click in open floor, not on or right next to walls. Spread the points out.
- More/fewer points: `patrol.py record --n 6`. Other options: `--wait 10`, `--timeout 90`, `--once`.
- Editing `points.yaml` or re-running `record` while patrolling takes effect on the next lap.

## 5b. Auto-resume after a restart (set up 2026-10-04)

The Pi resumes the patrol by itself on every boot: robot + Nav2 on the `event` map (no RViz),
pose restored from `~/wheelbot/lastpose.json`, then `patrol.py` with the current `points.yaml`.
Just power it on, keep it **still** and wait ~2 min. Log: `tail -f /tmp/autopatrol.log /tmp/patrol.log`.
- `pose_saver.py` (started by the service) keeps `lastpose.json` updated while the robot drives.
- **Don't move the robot while it's off** - it restarts from where it stopped. If it was moved:
  `systemctl --user stop wheelbot-patrol`, then `start_nav.sh event` + 2D Pose Estimate as in step 4.
- Stop it: `systemctl --user stop wheelbot-patrol`  (stops patrol, Nav2 and the robot)
- Start it again without rebooting: `systemctl --user start wheelbot-patrol`
- Turn auto-start off (e.g. before re-mapping): `systemctl --user disable wheelbot-patrol` (`enable` = back on)

## 6. Stop

- Patrol only: `Ctrl+C` in its terminal (or `pkill -f patrol.py`).
- Everything: `bash ~/wheelbot/stop_all.sh` (wheels stop by themselves 0.5 s later).
- Keep a hand near the motor power switch the first few runs.

## Troubleshooting

| Problem | Fix |
|---|---|
| Can't find the Pi on Wi-Fi | Wait 2 min after boot; check hotspot device list; turn hotspot off/on once |
| Pi keeps vanishing / Remote Desktop freezes | Power bank flat or too weak - see Power section |
| Map doesn't grow while driving | Lidar driver froze: `stop_all.sh`, then `start_mapping.sh` again |
| `M1 NO RESPONSE` in `/tmp/bring.log`, robot won't go straight | Power first; then left motor driver wiring (RX/TX) |
| Remote Desktop black screen / drops | Log out on the Pi's own screen (if a monitor is attached), reconnect |
| Robot doesn't move on clicked points | `patrol.py` not running - see step 5 |
| Robot doesn't move on Nav2 Goal | Do 2D Pose Estimate first; check `tail /tmp/nav.log` |
| Laser dots don't line up with the map | Wrong pose estimate, or venue changed - re-map (step 3) |
| Lidar `operation timeout` / low rotations | `lidar_check.py`; replug lidar USB; power: `cat /sys/devices/platform/soc/soc:firmware/get_throttled` must be `0` |
| Robot overshoots / wobbles | Lower `VX_MAX` in `make_nav2_params.sh`, run it, restart nav |

Logs: `/tmp/bring.log` (robot), `/tmp/slam.log` (mapping), `/tmp/nav.log` (Nav2), `/tmp/rviz.log`, `/tmp/patrol.log`.
