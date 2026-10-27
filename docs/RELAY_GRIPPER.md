# Gripper control with a USB relay board

The Quest right trigger opens and closes the gripper through a CH340 USB relay board
(LCUS style, 9600 baud, 4-byte frames `A0 <channel> <0|1> <checksum>`). No change to the Quest app.

## Behaviour

- Each trigger press (released → pressed) toggles the command on `/teleop/gripper_command`
  (`True` = close).
- First press after the bridge starts: **close**, CH1 on for 0.5 s. Next press: **open**, CH2 on for 0.5 s.
- Holding the trigger does not repeat. Release before pressing again.
- The trigger works independently of the grip (clutch); the arm need not be active.
- Both channels are switched off at startup, after each pulse, before a direction change and on
  shutdown. Nothing is pulsed at startup.

## Find the port

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

Prefer the `/dev/serial/by-id/...` path when several USB serial devices are connected. Your user
must be in the `dialout` group. Do not run ROS with sudo.

Only one process may use the port. Stop any other relay script or relay node first.

## Option A: launch with teleop (recommended)

Add these arguments to the `teleop.launch.py` command from [TELEOP_WORKFLOW.md](TELEOP_WORKFLOW.md),
replacing `gripper:=false`:

```bash
  gripper:=true gripper_backend:=relay \
  relay_port:=/dev/ttyUSB0 \
  relay_open_channel:=2 relay_close_channel:=1 \
  relay_pulse_duration:=0.5
```

## Option B: add to a running session

If teleop is already running with `gripper:=false`, start one relay node in another terminal:

```bash
ros2 run quest_kuka_teleop relay_gripper --ros-args \
  -p relay_port:=/dev/ttyUSB0 \
  -p relay_open_channel:=2 \
  -p relay_close_channel:=1 \
  -p relay_pulse_duration:=0.5
```

It ignores commands sent before it started and waits for the next trigger press. That press follows
the bridge's current toggle state, so it may open rather than close.

## First test

Keep fingers clear. Release the trigger once, press to close, release, press again to open.

## Inspect

```bash
ros2 topic echo /teleop/gripper_command
ros2 topic echo /teleop/gripper_state
```

`result` is `pulsing`, `pulse_complete` or `error`. `state` is always `UNKNOWN`: the relay has no
finger feedback, so a completed pulse does not confirm a grasp.

On a serial error the node latches `error`. Fix the connection and restart the node.

## Limits

- Pulse timing uses a 10 ms ROS timer, not a hardware timer.
- If the process is killed, the USB is unplugged or the PC fails, the OFF command may not reach a
  latching relay.
