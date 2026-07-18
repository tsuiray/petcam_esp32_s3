# petcam_esp32_s3 (Arduino IDE)

ESP32-S3 firmware for PetCam: **micro-ROS (Humble)** over **Wi‑Fi UDP**, publishing **MPU6050** IMU data to an NVIDIA Orin Nano (ROS 2 Humble).

Built for **Arduino IDE** (not ESP-IDF). Sketch lives in [`esp32/`](esp32/) so the folder name matches `esp32.ino`.

## Architecture

```
ESP32-S3 (STA) ──join──► Home Wi‑Fi AP ◄──join── Orin Nano (STA)
                              │
                              └─ LAN UDP 8888 ─► micro-ros-agent ─► /imu/data
```

Both devices join the **same home Wi‑Fi AP** as stations. **Not** Wi‑Fi Direct / SoftAP.

| Topic | Type | Rate |
|-------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz (`IMU_PUBLISH_PERIOD_MS = 20`) |

The firmware **reconnects** if Wi‑Fi drops or the micro-ROS agent goes away (entities are torn down and recreated when the agent is reachable again).

## Arduino IDE setup

### 1. ESP32 board package

1. **File → Preferences → Additional boards manager URLs**  
   Add: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Tools → Board → Boards Manager** → install **esp32** by Espressif
3. Select your board, e.g. **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**
4. Pick the correct **Port**

Recommended Tools settings (typical S3 DevKit):

- USB CDC On Boot: **Enabled** (if using native USB serial)
- Upload Speed: **921600** (or lower if uploads fail)

### 2. Install `micro_ros_arduino` (Humble)

Arduino Library Manager may not ship the exact Humble build. Prefer a git checkout into your libraries folder:

**Windows (CMD):**

```cmd
cd %USERPROFILE%\Documents\Arduino\libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git
```

If your Sketchbook location differs, use **File → Preferences → Sketchbook location** `\libraries`.

Restart Arduino IDE after installing.

### ESP32-S3 note

Official packages often **do not ship** `src/esp32s3/libmicroros.a`. Arduino then fails with
`Precompiled library in ...\esp32s3 not found` and many `undefined reference` link errors.

Fix (CMD) — copy the ESP32 archive into the S3 folder (both filenames some IDE versions expect):

```cmd
cd %USERPROFILE%\OneDrive\Documents\Arduino\libraries\micro_ros_arduino\src
mkdir esp32s3 2>nul
copy /Y esp32\libmicroros.a esp32s3\libmicroros.a
copy /Y esp32\libmicroros.a esp32s3\libmicro_ros_arduino.a
```

Then restart Arduino IDE and compile again.

If you later see **relocation / wrong ELF** errors, the ESP32 `.a` is not binary-compatible with S3; you must rebuild with Docker:

```bash
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v "%USERPROFILE%/OneDrive/Documents/Arduino/libraries/micro_ros_arduino:/project" --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p esp32s3
```

### 3. Configure Wi‑Fi and Agent IP (secrets)

Do **not** commit real SSID/password. Use a local override:

```cmd
cd C:\Petcam\edge\esp32\esp32
copy board_config.local.h.example board_config.local.h
notepad board_config.local.h
```

Set at least:

```cpp
#define WIFI_SSID "YOUR_HOME_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_HOME_WIFI_PASSWORD"
#define MICROROS_AGENT_IP "192.168.1.100"   // Orin LAN IP on the home AP
#define MICROROS_AGENT_PORT 8888
```

[`board_config.local.h`](esp32/board_config.local.h.example) is gitignored. Defaults and pins live in [`board_config.h`](esp32/board_config.h) (SDA=8, SCL=9, addr=`0x68`).

### 4. Open and upload

1. **File → Open** → `C:\Petcam\edge\esp32\esp32\esp32.ino`  
   (folder name `esp32` must match `esp32.ino`)
2. Click **Upload**
3. Open **Serial Monitor** at **115200** baud

Expected log:

1. Wi‑Fi IP printed
2. `Waiting for micro-ROS agent...` then `agent is reachable`
3. `Publishing sensor_msgs/Imu on /imu/data`
4. If the agent or Wi‑Fi drops later, the sketch retries without reboot (LED blinks while waiting)

## Orin Nano: micro-ROS Agent (Humble)

On the Orin (same home Wi‑Fi AP):

```bash
ip -4 addr show   # use this IP in board_config.local.h

docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

Verify:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

Tips:

- Prefer DHCP reservation for the Orin IP.
- Disable AP **client isolation** if devices cannot ping each other.
- Agent IP = **Orin** address, not the ESP32 IP.

## Project files

| File | Role |
|------|------|
| `esp32/esp32.ino` | Wi‑Fi + micro-ROS node + reconnect + `/imu/data` timer |
| `esp32/board_config.h` | Defaults (pins, rates); includes optional local secrets |
| `esp32/board_config.local.h.example` | Template for SSID / agent IP (copy → `.local.h`) |
| `esp32/mpu6050.h` / `esp32/mpu6050.cpp` | MPU6050 driver (±2g / ±250 dps → SI) |
| `esp32/wifi_transport.cpp` | Local Wi‑Fi UDP transport (S3 library workaround) |

## Out of scope

- mmWave driver / `/mmwave/range`
- DC motor / `/cmd_vel`
