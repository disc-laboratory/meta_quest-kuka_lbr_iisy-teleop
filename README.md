# Quest 3 → KUKA LBR iisy teleoperation

Teleoperate a KUKA LBR iisy 3 R760 with a Meta Quest 3 controller, and open/close a gripper with the
trigger. Runs on ROS 2 Jazzy (Ubuntu 24.04) using MoveIt Servo and the KUKA iiQKA EAC driver.

```
Quest 3 app (input only)
  └─ TCP :10000 ─> ros_tcp_endpoint ─> /right_controller_odom, /right_*_button_state
      └─> bridge   (grip = clutch, freshness/tracking gates, workspace box, rate limits)
          └─> /servo_node/pose_target_cmds
              └─> MoveIt Servo (collision + singularity checks)
                  └─> joint_trajectory_controller ─> KUKA EAC driver ─> robot
Right trigger ─> /teleop/gripper_command ─> relay_gripper (USB relay) or gripper_io (24 V I/O)
```

> **Safety.** The software gates in this package are not certified safety functions. Always have an
> operator at the pendant with the enabling switch and E-stop in reach.

## Repository layout

| Path | Contents |
| --- | --- |
| `quest_kuka_teleop/` | ROS 2 package: nodes, launch files, configs, URDF/SRDF, helper scripts, tests |
| `tools/` | Standalone read-only diagnostics (input monitor, timing audit, ROS inventory) |
| `docs/` | Step-by-step guides (see below) |
| `dependencies.repos` | Pinned third-party source (Unity ROS-TCP-Endpoint) |

### Nodes (`ros2 run quest_kuka_teleop <name>`)

| Node | Purpose |
| --- | --- |
| `bridge` | Quest pose → clutched, bounded Cartesian target for MoveIt Servo |
| `relay_gripper` | Trigger → 0.5 s pulse on a CH340 USB relay board (CH1 close, CH2 open) |
| `gripper_io` | Trigger → Schunk 24 V Open/Close with sensor feedback (`mock` backend only for now) |
| `fake_quest` | Synthetic Quest input for mock testing. Never use with hardware |
| `episode_recorder` | Records episodes to rosbag2/MCAP with `meta.json` |
| `audit_episode` | Rate and latency report for one recorded episode |

### Launch files

| Launch | Starts |
| --- | --- |
| `robot.launch.py mode:=mock\|hardware` | KUKA EAC driver. Starts unconfigured; nothing moves until activated |
| `teleop.launch.py` | MoveIt Servo, bridge, TCP endpoint, and optionally the gripper node |
| `quest_input_only.launch.py` | TCP endpoint only, for checking Quest input without a robot |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- KUKA LBR iisy 3 R760 with iiQKA and External Control (EAC); PC on `192.168.44.2`, controller on `192.168.44.1`
- Meta Quest 3 running the PickNik teleop app ([docs/QUEST_APP_BUILD.md](docs/QUEST_APP_BUILD.md))
- PC and Quest on the same Wi-Fi network (5 GHz recommended)
- Optional: CH340 USB relay board for the gripper

## Install

```bash
sudo apt update
sudo apt install python3-vcstool python3-serial python3-scipy \
  ros-jazzy-kuka-controllers ros-jazzy-kuka-driver-interfaces \
  ros-jazzy-kuka-mock-hardware-interface ros-jazzy-kuka-lbr-iisy-moveit-config \
  ros-jazzy-moveit-servo ros-jazzy-rosbag2-storage-mcap

# Real-time priority for the driver (log out and back in afterwards)
echo "$USER - rtprio 98
$USER - memlock unlimited" | sudo tee /etc/security/limits.d/90-kuka-rt.conf

# Allow the Quest to reach the PC (replace wlp8s0 with your Wi-Fi interface)
sudo ufw allow in on wlp8s0 to any port 10000 proto tcp

# Relay board access without sudo (log out and back in afterwards)
sudo usermod -aG dialout $USER
```

Build the workspace:

```bash
mkdir -p ~/quest_ws/src && cd ~/quest_ws/src
git clone https://github.com/disc-laboratory/meta_quest-kuka_lbr_iisy-teleop.git quest_kuka_teleop
vcs import < quest_kuka_teleop/dependencies.repos
cd ~/quest_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Run in **every** terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/quest_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Run the unit tests (no ROS or hardware needed):

```bash
python3 -m pytest ~/quest_ws/src/quest_kuka_teleop/quest_kuka_teleop/test
```

## Bring-up order

Do these in order the first time. Do not skip to hardware.

1. **Mock, no robot:** `bash quest_kuka_teleop/scripts/mock_e2e.sh`. Runs the full chain on KUKA's
   mock hardware with fake Quest input, records 40 s and prints an audit.
2. **Quest input only:** `ros2 launch quest_kuka_teleop quest_input_only.launch.py`, connect the Quest
   app, then `python3 tools/monitor.py --seconds 60 --out reports/input.jsonl`. Expect ~60 Hz.
3. **Quest on mock robot with RViz:** `bash quest_kuka_teleop/scripts/quest_mock_session.sh`.
4. **Real robot:** follow [docs/TELEOP_WORKFLOW.md](docs/TELEOP_WORKFLOW.md).

## Controls

| Input | Action |
| --- | --- |
| Hold right grip | Robot TCP follows controller motion relative to where you pressed |
| Release right grip | Motion stops. Reposition your hand and hold again to continue |
| Right trigger press | Toggle gripper: first press closes, next press opens |

After connecting or activating, release the grip once before the first hold.

## Configuration

All files are in `quest_kuka_teleop/config/`. After editing, rebuild is not needed with
`--symlink-install`, but running nodes must be restarted.

| File | What to set |
| --- | --- |
| `teleop.yaml` | Default bridge settings (conservative). `base_from_quest` and the workspace box are placeholders |
| `bridge_expanded.yaml` | Tuned bridge profile used on the real robot: larger workspace, higher speed |
| `servo.yaml` / `servo_expanded.yaml` | MoveIt Servo settings; `override_velocity_scaling_factor` sets joint speed |
| `tcp.yaml` | tool0 → gripper finger centre offset. Re-measure after changing the gripper |
| `session.yaml` | Metadata written into each recorded episode |

Key bridge parameters:

| Parameter | Meaning |
| --- | --- |
| `scale` | Robot motion per unit of hand motion |
| `max_target_speed`, `max_target_angular_speed` | Target rate limits (m/s, rad/s) |
| `max_lead`, `max_lead_angle` | Max distance the target may lead the measured TCP |
| `workspace_min`, `workspace_max` | TCP box in `base_link` (m), enforced when `enforce_workspace: true` |
| `max_input_age`, `max_input_hold` | Input staleness limits (s) before motion pauses / clutch drops |

## Documentation

| Guide | Use it for |
| --- | --- |
| [docs/TELEOP_WORKFLOW.md](docs/TELEOP_WORKFLOW.md) | Running a session on the real robot, start to shutdown |
| [docs/RELAY_GRIPPER.md](docs/RELAY_GRIPPER.md) | Gripper control through a USB relay board |
| [docs/QUEST_APP_BUILD.md](docs/QUEST_APP_BUILD.md) | Building and installing the Quest 3 app (Windows + Unity) |
| [docs/GRIPPER_IO_WIRING.md](docs/GRIPPER_IO_WIRING.md) | Wiring brief for PC-controlled 24 V gripper I/O |
| [docs/RECORDING.md](docs/RECORDING.md) | Recording and auditing demonstrations |

## Known limitations

- `base_from_quest` is identity: it assumes the operator faces along robot +X. Calibrate for your cell.
- The relay gripper has no finger feedback; `/teleop/gripper_state` reports `UNKNOWN` position.
- `gripper_io` has only a `mock` backend until a 24 V I/O module is chosen.
- The SRDF collision matrix is minimal and hand-written.

## License

Apache-2.0. See [LICENSE](LICENSE). ROS-TCP-Endpoint is fetched separately under its own license.
