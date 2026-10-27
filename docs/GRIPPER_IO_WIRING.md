# PC-controlled Schunk gripper: wiring brief for the lab electrician

**Goal:** let a Linux PC open/close the Schunk Co-act ESG-C K060 (part 1482460) on the KUKA LBR
iisy 3 R760 while the PC also controls the arm through KUKA's External API. That API carries
motion only (no I/O), and iiQKA 1.2.6 has no logic to forward a PC signal to the gripper. So the
gripper signals must come from PC-controlled I/O.

## What the robot uses today (from the iiQKA pendant, Setup → I/O → Signals)

| Signal | Direction (robot view) | Mapped to |
| --- | --- | --- |
| Open | output | Interface Board Standard (IFB-STD) Outputs.Output 0 |
| Close | output | IFB-STD Outputs.Output 1 |
| Sensor A | input | IFB-STD Inputs.Input 0 |
| Sensor B | input | IFB-STD Inputs.Input 1 |
| Reserved 1 | input | IFB-STD Inputs.Input 2 |

The IFB-STD sits on the KUKA System Bus (SYS-X48, EtherCAT) of the KR C5 micro. There's no
PROFINET or EtherNet/IP device configured. Logic (outer grip): Gripped = A 0 / B 1, Released = A 1 / B 0.
Physically, the gripper plugs into the robot's tool-flange connector below the A6 light ring.

## Requested change

Connect a **24 V digital I/O module controlled by the PC** to the gripper's Open, Close, Sensor A and
Sensor B lines, **in place of** IFB-STD Output 0/1 and Input 0/1.

- **Never parallel two sources.** IFB outputs and PC outputs must not drive the same wire. Disconnect or
  switch over (e.g. a changeover terminal), or use a separate gripper cable.
- Keep the gripper's 24 V supply and GND exactly as they are today unless the Schunk manual says
  otherwise. The PC module shares the gripper's GND reference.
- Work with the controller powered off, following KUKA and Schunk documentation.
- These are **not safety signals** (the pendant marks them "Not safe"). E-stop, safety door and
  enabling switch are not touched.

## Information needed (please confirm from the documentation)

1. ESG-C K060 manual: connector pinout, input type (PNP/NPN), input current, sensor output type,
   and whether Open/Close are level- or pulse-controlled.
2. KR C5 micro / iisy media-flange pinout: where IFB-STD Output 0/1 and Input 0/1 are physically
   wired, and whether a breakout at the flange or the controller is the cleaner place to switch over.

## Suggested I/O module (to confirm against item 1)

An **Ethernet Modbus/TCP remote I/O** with 24 V **PNP (sourcing) outputs** and 24 V inputs, e.g. a
4–8 DI / 4–8 DO DIN-rail unit. It connects to the PC's Ethernet (a third NIC or a switch on a
separate subnet, **not** the KUKA KONI 192.168.44.x network). Software support for Modbus/TCP is
straightforward (pymodbus). A USB 24 V DIO module is an acceptable alternative.

## Software status (PC side, tested on the mock backend)

ROS node `gripper_io` (`ros2 run quest_kuka_teleop gripper_io`):
- Quest right trigger → `/teleop/gripper_command` (True = close).
- The node drives Close or Open (break-before-make, never both), holds the output until the sensors
  confirm GRIPPED or RELEASED or 3 s pass, then clears both.
- It publishes `/teleop/gripper_state`, with sensor bits, outputs, state and result.
- A hardware backend for the chosen module is added once the module and pinout are known.
