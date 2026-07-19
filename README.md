# petcam_esp32_s3 (Arduino IDE)

ESP32-S3 firmware for PetCam: **micro-ROS (Humble)** over **Wi‑Fi TCP** (default; UDP optional), publishing **MPU6050** / SIM IMU data to an NVIDIA Orin Nano (ROS 2 Humble).

Built for **Arduino IDE** (not ESP-IDF).

## Architecture

```
ESP32-S3 (STA) ──join──► Home Wi‑Fi AP ◄──join── Orin Nano (STA)
                              │
                              └─ LAN TCP 8888 ─► micro-ros-agent ─► /imu/data
```

Both devices join the **same home Wi‑Fi AP** as stations. **Not** Wi‑Fi Direct / SoftAP.

Default transport is **TCP** so it matches Orin `create_map` (`micro_ros_agent tcp4 --port 8888`).

| Topic | Type | Rate |
|-------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz (`IMU_PUBLISH_PERIOD_MS = 20`) |

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

### 3. Configure Wi‑Fi, Agent IP, and transport

Edit [`board_config.h`](board_config.h):

```cpp
#define WIFI_SSID "YOUR_HOME_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_HOME_WIFI_PASSWORD"
#define MICROROS_AGENT_IP "192.168.1.100"   // Orin LAN IP on the home AP
#define MICROROS_AGENT_PORT 8888
#define MICROROS_TRANSPORT MICROROS_TRANSPORT_TCP  // default; or _UDP
```

Also set I2C pins / MPU6050 address if needed (`BOARD_I2C_SDA`, `BOARD_I2C_SCL`, `BOARD_MPU6050_ADDR`). Defaults: SDA=8, SCL=9, addr=`0x68`.

### 4. Open and upload

1. **File → Open** → `C:\Petcam\edge\esp32\esp32.ino`  
   (folder name `esp32` must match `esp32.ino`)
2. Click **Upload**
3. Open **Serial Monitor** at **115200** baud

Expected log: Wi‑Fi IP → `micro-ROS TCP transport -> <orin-ip>:8888` → agent reachable → `Publishing sensor_msgs/Imu on /imu/data`.

## Orin Nano: micro-ROS Agent (Humble)

On the Orin (same home Wi‑Fi AP). Prefer the create_map helper (TCP):

```bash
ip -4 addr show   # use this IP in board_config.h
./scripts/run_create_map.sh          # starts micro_ros_agent tcp4 :8888
```

Manual agent (must match `MICROROS_TRANSPORT`):

```bash
# TCP (firmware default)
ros2 run micro_ros_agent micro_ros_agent tcp4 --port 8888 -v6
# or Docker:
docker run -it --rm --net=host microros/micro-ros-agent:humble tcp4 --port 8888 -v6

# UDP fallback (only if MICROROS_TRANSPORT_UDP on ESP32)
# docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
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
- Transport mode on ESP32 and Orin **must match** (TCP↔tcp4 or UDP↔udp4).

## Simulation vs real IMU

In [`board_config.h`](board_config.h):

```cpp
#define IMU_DATA_MODE IMU_DATA_MODE_SIM   // synthetic L-home path (~500 sq ft)
// #define IMU_DATA_MODE IMU_DATA_MODE_REAL // MPU6050 hardware
```

| Mode | Source | Files |
|------|--------|-------|
| `IMU_DATA_MODE_SIM` | Precomputed closed L-home lap (~490 sq ft); **same samples every loop** | [`imu_sim.cpp`](imu_sim.cpp) / [`imu_sim.h`](imu_sim.h) |
| `IMU_DATA_MODE_REAL` | MPU6050 over I2C | [`mpu6050.cpp`](mpu6050.cpp) |

SIM notes for Orin `create_map`:

- `linear_acceleration.x/y` are **world-frame** accel (not body-frame); `z=0` (no gravity) so a naive Euler integrator redraws the same polygon each lap.
- One lap is buffered and replayed identically (`lap` counter in Serial log).
- If the map still drifts, check that create_map uses the same `dt` (20 ms) and zeros velocity when accel stays near 0 at corners.

## Project files

| File | Role |
|------|------|
| `esp32.ino` | Wi‑Fi + micro-ROS node + `/imu/data` publish loop |
| `board_config.h` | SSID, Agent IP, transport, I2C pins, rate, **IMU mode** |
| `imu_sim.h` / `imu_sim.cpp` | Simulation path (~500 sq ft L-home) |
| `mpu6050.h` / `mpu6050.cpp` | MPU6050 driver (±2g / ±250 dps → SI) |
| `wifi_tcp_transport.h` / `.cpp` | Wi‑Fi **TCP** XRCE transport (default) |
| `wifi_transport.cpp` | Wi‑Fi **UDP** XRCE transport (fallback) |

## Out of scope

- mmWave driver / `/mmwave/range`
- DC motor / `/cmd_vel`
