# Running a teleop session on the real robot

Use this for a fresh session. If the robot is already running correctly, do not start duplicate launches.

Commands below are run from the repository root (`~/quest_ws/src/quest_kuka_teleop`) after the
per-terminal setup in the [README](../README.md#install). `SHARE` is used for config paths:

```bash
SHARE=$(ros2 pkg prefix --share quest_kuka_teleop)
```

## 1. Network and pendant

1. Connect the PC and the Quest to the same 5 GHz Wi-Fi network. Keep the robot Ethernet connected.
2. At the pendant: acknowledge controller errors, select the controller mode your EAC setup requires,
   and release SPOC for external control. Keep the E-stop in reach.
3. Keep the right grip released during startup.

Check the network (replace interface names with yours):

```bash
ping -I enp7s0 -c 4 192.168.44.1        # robot controller
sudo iw dev wlp8s0 set power_save off   # avoids Wi-Fi latency bursts
iw dev wlp8s0 link                      # freq 5xxx MHz = 5 GHz
ip -4 addr show wlp8s0                  # enter THIS address in the Quest app
```

The Wi-Fi address can change between sessions. Check it every time.

## 2. Terminal 1: driver

```bash
ulimit -Sr        # should print 98 after the limits.conf step in the README
ros2 launch quest_kuka_teleop robot.launch.py mode:=hardware
```

Leave it running. The robot starts unconfigured.

## 3. Terminal 2: configure

```bash
bash quest_kuka_teleop/scripts/robot_lifecycle.sh configure
```

Expect `inactive [2]`. To check the current state at any time: `robot_lifecycle.sh state`.

## 4. Terminal 3: Servo, bridge and Quest endpoint

```bash
ros2 launch quest_kuka_teleop teleop.launch.py \
  bridge_config:=$SHARE/config/bridge_expanded.yaml \
  servo_config:=$SHARE/config/servo_expanded.yaml \
  workspace_collision_model:=true \
  tool_envelope_radius:=0.09 \
  tool_envelope_length:=0.166 \
  table_top_z:=-0.05 \
  gripper:=false
```

- `tool_envelope_*` is a padded cylinder enclosing the mounted gripper; `table_top_z` is the table
  height relative to the robot base (m). Measure these for your setup.
- For the relay gripper, see [RELAY_GRIPPER.md](RELAY_GRIPPER.md).

Open the Quest app and connect to the PC's Wi-Fi IP. If this launch is restarted, reconnect the app.

## 5. Terminal 2: activate

With the right grip released:

```bash
bash quest_kuka_teleop/scripts/robot_lifecycle.sh activate
```

Type `ACTIVATE` at the prompt. Expect `active [3]`. Brakes release at this point.

If activation fails, read the error in Terminal 1. An operation-source permission error means SPOC
is not released on the pendant.

## 6. Operate

- Release the right grip once after connecting or activating.
- Hold the grip: the TCP follows the controller (translation and rotation) relative to the press pose.
- Release: motion stops. Reposition your hand and hold again to keep moving.
- If motion stops on its own (stale input, tracking loss, Servo halt), release and press grip again
  after the cause clears. Terminal 3 logs the reason as `state: <reason>`.

## 7. Check or change speed

```bash
ros2 param get /quest_bridge scale
ros2 param get /quest_bridge max_target_speed
ros2 param get /quest_bridge max_target_angular_speed
ros2 param get /servo_node moveit_servo.override_velocity_scaling_factor
```

Edit `bridge_expanded.yaml` (scale, target rate limits in m/s and rad/s) or `servo_expanded.yaml`
(joint velocity scaling, a fraction of the joint limits). To apply: deactivate, stop Terminal 3,
relaunch it, reconnect the Quest, activate.

## 8. Shutdown

1. Release the grip.
2. Terminal 2: `bash quest_kuka_teleop/scripts/robot_lifecycle.sh deactivate` → expect `inactive [2]`.
3. Ctrl+C in Terminal 3, then Terminal 1.

In an emergency use the E-stop. Do not wait for terminal commands.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No Quest data | Same network? Firewall rule for port 10000? Campus Wi-Fi often blocks device-to-device traffic: use a hotspot or private router |
| `robot_manager not visible` | `ROS_DOMAIN_ID` differs between terminals |
| `state: outside_workspace` | TCP is outside `workspace_min/max` in the bridge config |
| `state: input_stale` / `tracking_lost` | Wi-Fi gaps or controller out of headset view |
| `state: servo_halt:*` | Collision or singularity. Release grip, move away using another direction |
