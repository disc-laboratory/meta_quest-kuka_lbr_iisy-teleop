# Building and installing the Quest 3 app

The app is PickNik's open-source Unity project. It is **input only**: it streams controller poses
and button states over TCP to the ROS 2 PC. It does not control the robot.

Done once per headset, on a Windows PC with Unity.

## Fixed versions

Use exactly these. Newer Unity or XR packages change XR behaviour.

| Item | Value |
| --- | --- |
| Source | https://github.com/PickNikRobotics/meta_quest_teleoperation (archived) |
| Commit | `bbaef0762fdb0b429b8ea12a4ca65040748b41dd` |
| Unity project folder | `UnityProject/` |
| Unity editor | **6000.1.6f1** with Android Build Support (OpenJDK, Android SDK & NDK) |
| Android settings | min SDK 32, IL2CPP, ARM64, package id `com.unity.template.vr`, product "ROS Teleop" |
| PC endpoint | `ros_tcp_endpoint` v0.7.0, TCP port **10000** |

Topics published by the app: `/left_controller_odom`, `/right_controller_odom` (`nav_msgs/Odometry`,
frame `quest`, ROS FLU axes), `/<hand>_<grip|trigger|a|b|menu>_button_state` (`std_msgs/Bool`, 60 Hz),
`*_button_event` (`std_msgs/Empty`) and `/tf`. The ROS package depends on these; do not change them.

## 1. Prepare Windows

1. Install Git and Unity Hub.
2. In Unity Hub: Installs → Install Editor → 6000.1.6f1 (use the archive if not listed) → add
   **Android Build Support** with **OpenJDK** and **Android SDK & NDK Tools**.
3. `adb.exe` is at
   `C:\Program Files\Unity\Hub\Editor\6000.1.6f1\Editor\Data\PlaybackEngines\AndroidPlayer\SDK\platform-tools`.

## 2. Get the source

Clone to a short path; long paths break Android builds.

```bat
git clone https://github.com/PickNikRobotics/meta_quest_teleoperation C:\dev\meta_quest_teleoperation
cd C:\dev\meta_quest_teleoperation
git checkout bbaef0762fdb0b429b8ea12a4ca65040748b41dd
```

## 3. Enable developer mode on the Quest

1. Meta Horizon phone app → Devices → Quest 3 → Headset settings → Developer Mode → on.
   You may need to create a developer organisation at developers.meta.com first.
2. Reboot the headset.
3. Connect it to the PC with a USB-C data cable. In the headset, accept **Allow USB debugging**
   (tick "Always allow").
4. `adb devices` must list the headset as `device` (not `unauthorized`). If it is missing, install
   the Meta Quest ADB driver.

## 4. Open the project

1. Unity Hub → Add → `C:\dev\meta_quest_teleoperation\UnityProject`, editor 6000.1.6f1.
   The first import downloads ROS-TCP-Connector, so internet is required. Decline any "upgrade project" prompt.
2. File → Build Profiles → Android → **Switch Platform**.
3. Project Settings → XR Plug-in Management → Android: **OpenXR** enabled, **Meta Quest Support** ticked.
4. Project Settings → Player → Android: Scripting Backend **IL2CPP**, Target Architectures **ARM64** only.
5. Make sure the scene containing the `ROSPublishers` component is scene 0 in the build list.

## 5. Build and install

File → Build Profiles → Android → Run Device = Quest 3 → **Build and Run**. Save the APK as
`C:\dev\builds\quest_teleop.apk`.

To reinstall later without Unity:

```bat
adb install -r C:\dev\builds\quest_teleop.apk
```

## 6. First connection

1. On the Linux PC (after the per-terminal setup):
   ```bash
   ros2 launch quest_kuka_teleop quest_input_only.launch.py
   ```
2. In the headset: Library → filter **Unknown Sources** → **ROS Teleop**.
3. Press **Left X** to open the keyboard, type the Linux PC's Wi-Fi IP (`ip -4 addr show <wifi-if>`),
   press Done. The IP is saved for later launches.
4. On the Linux PC, check input while moving both controllers and pressing grip and trigger:
   ```bash
   python3 tools/monitor.py --seconds 60 --out reports/input.jsonl
   ```
   Pass: about 60 Hz on `/right_controller_odom` and the button topics.

## If no data arrives

- The endpoint terminal logs incoming connections. No connection usually means the Wi-Fi isolates
  clients (common on campus networks). Put the PC and Quest on a phone hotspot or private router.
- Test from the headset: `adb shell ping -c 3 <linux_ip>`.
- Check the Linux firewall allows TCP 10000 on the Wi-Fi interface.
- Keep the robot Ethernet network (192.168.44.x) separate from the Wi-Fi network.
