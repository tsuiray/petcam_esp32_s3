# petcam_esp32_s3

ESP32-S3 firmware for PetCam edge sensing: **micro-ROS (Humble)** over **Wi‑Fi UDP**, publishing **MPU6050** IMU data to an NVIDIA Orin Nano running ROS 2 Humble.

## Architecture

```
ESP32-S3 (STA) ──join──► Home Wi‑Fi AP ◄──join── Orin Nano (STA)
                              │
                              └─ LAN UDP 8888 ─► micro-ros-agent ─► /imu/data
```

Both devices join the **same home Wi‑Fi AP** as normal stations. This is **not** Wi‑Fi Direct / SoftAP.

| Topic | Type | Rate |
|-------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz (see `BOARD_IMU_PUBLISH_HZ`) |

## Requirements

- ESP-IDF **v5.2+** (tested path for `micro_ros_espidf_component` humble)
- Python packages inside the IDF venv:
  ```bash
  pip install catkin_pkg colcon-common-extensions lark "empy<4"
  ```
- Orin: ROS 2 **Humble** + Docker (for the agent image)

## Clone / submodule

```bash
git clone --recurse-submodules https://github.com/tsuiray/petcam_esp32_s3.git C:\Petcam\edge\esp32
cd C:\Petcam\edge\esp32
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Configure Wi‑Fi and Agent IP

Edit [`sdkconfig.defaults`](sdkconfig.defaults) **or** run menuconfig:

```bash
idf.py set-target esp32s3
idf.py menuconfig
```

Under **micro-ROS Settings**:

1. Network interface: **WLAN**
2. **WiFi Configuration** → home AP SSID / password (same AP Orin uses)
3. **micro-ROS Agent IP** → Orin’s LAN IP on that AP (e.g. `192.168.1.50`)
4. **micro-ROS Agent Port** → `8888`

Tips:

- Prefer a DHCP reservation for the Orin so the Agent IP stays stable.
- Disable **AP / client isolation** on the router if devices cannot ping each other.
- Agent IP is the **Orin** address, not the ESP32’s IP and not the router gateway.

## Build & flash

```bash
# From an ESP-IDF exported shell (no ROS 2 setup.bash sourced)
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

First build compiles `libmicroros` (several minutes). To force a micro-ROS rebuild later:

```bash
idf.py clean-microros
```

### Smoke-check connectivity (same path as int32_publisher)

After flash, the serial log should show:

1. Wi‑Fi connected (SSID from config)
2. `Waiting for micro-ROS agent...` then `agent is reachable`
3. `Publishing sensor_msgs/Imu on /imu/data`

If step 2 loops: wrong Agent IP, agent not running, or AP isolation.

## Orin Nano: start micro-ROS Agent (Humble)

On the Orin (same Wi‑Fi AP as the ESP32):

```bash
# Confirm Orin Wi‑Fi IP (use this in ESP menuconfig / sdkconfig.defaults)
ip -4 addr show

docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

In another terminal on the Orin:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

Expected: `/imu/data` appears; `linear_acceleration` / `angular_velocity` update; `orientation_covariance[0] == -1` (no fused orientation).

## Hardware defaults

Configured in [`main/board_config.h`](main/board_config.h):

| Item | Default |
|------|---------|
| I2C SDA | GPIO 8 |
| I2C SCL | GPIO 9 |
| MPU6050 address | `0x68` |
| Publish rate | 50 Hz |

Change pins/address there to match your board.

## Project layout

```
main/
  main.c                 # Wi‑Fi STA + micro-ROS node + agent ping/retry
  imu_publisher.c/h      # /imu/data timer publisher
  sensors/mpu6050.c/h    # MPU6050 I2C driver (±2g / ±250 dps → SI units)
  board_config.h
components/
  micro_ros_espidf_component/   # submodule @ humble
```

## Out of scope (this revision)

- mmWave distance driver / `/mmwave/range`
- DC motor control / `/cmd_vel`
