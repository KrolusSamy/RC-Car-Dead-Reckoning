# ESP32-C6 Robot Dead-Reckoning Ground Station

An ESP-IDF firmware and Python ground station for a small two-wheel robot. The ESP32 estimates its 2D pose from wheel encoders and gyro heading, uses the accelerometer for tilt, streams telemetry to a laptop, and accepts short movement taps from a phone browser.

> **Project status:** working prototype for demonstration and experiments. Position estimates drift over time; the chassis, wheel calibration, and motion controller are not yet tuned well enough for precision navigation. This project does not autonomously drive to waypoints.

## What is in the repository

- `main/`, `components/`, `CMakeLists.txt`, `dependencies.lock`, and `partitions.csv` — ESP-IDF firmware for ESP32-C6.
- `server.py` — laptop ground station: live pose map, telemetry log, waypoint references, estimator comparison, gyro calibration request, and OTA file serving.
- `web_controller.py` — phone-friendly page for short forward, reverse, left, and right taps.
- `docs/hardware-wiring.svg` — wiring overview.
- `TECHNICAL_README.md` — telemetry, network, estimator, calibration, safety, and known limitations.
- `docs/WIFI_IOT_DIAGRAM_PROMPT.md` — a detailed prompt for making a network/data-flow illustration.

## Hardware at a glance

ESP32-C6, MPU6050 IMU, two LM393 wheel encoders, HC-SR04 ultrasonic sensor, L298N dual motor driver, two DC motors, a battery pack, and a buck converter. The ultrasonic Echo pin is 5 V and must pass through the documented 1 kΩ / 2 kΩ divider before reaching the ESP32 input.

![Hardware wiring overview](docs/hardware-wiring.svg)

## Quick start

### 1. Build and flash the firmware

Install ESP-IDF **v5.1.1** and its ESP32-C6 toolchain, open this repository in the ESP-IDF terminal, then run:

```powershell
idf.py set-target esp32c6
idf.py build
idf.py -p COMx flash monitor
```

Replace `COMx` with the board's serial port. The first build can take a while while ESP-IDF resolves managed components. Check `components/middleware/basic-software/services/include/SRV_WIFI_Cfg.h` and `SRV_UDP_Cfg.h` before building: configure the ESP32 to join your laptop hotspot and set the laptop's reachable hotspot address. Do not publish personal Wi-Fi credentials in a public repository.

The repository includes `sdkconfig.defaults` for ESP32-C6, 4 MiB flash, and the custom OTA partition table. Before building, create your local Wi-Fi configuration:

```powershell
Copy-Item components/middleware/basic-software/services/include/SRV_WIFI_Credentials.h.example components/middleware/basic-software/services/include/SRV_WIFI_Credentials.h
```

Edit the copied file with your laptop hotspot's SSID and password. The local credentials file is ignored by Git. The project uses the ESP-IDF component manager; dependencies are declared in `main/idf_component.yml` and locked in `dependencies.lock`.

### 2. Start the laptop ground station

Connect the laptop to the same Wi-Fi network/hotspot used by the ESP32, then run:

```powershell
python server.py
```

The ground station listens for ESP32 telemetry on UDP port `12345`. Its OTA HTTP service uses TCP port `8070`; choose the firmware `.bin` in the GUI before requesting an OTA update.

### 3. Start the phone controller

Python 3 with Tkinter is required for the ground-station window. The tools use Python's standard library and need no pip packages. In another terminal, run:

```powershell
python web_controller.py
```

Open `http://192.168.137.1:8080/` on a phone connected to the laptop hotspot. If the laptop hotspot uses another address, change `HOST_IP` in `web_controller.py`. If the ESP32 receives a different DHCP address, update `ESP32_IP` there too. The four controls send single short taps; they do not select waypoints or execute a route.

Allow the applications through the laptop firewall on the private hotspot network. The relevant traffic is UDP `12345` for telemetry/commands and TCP `8070`/`8080` for OTA and the controller page.

## Safety and calibration

- Keep the robot lifted or in a clear test area during the first command test.
- Keep the robot stationary through startup gyro warm-up and bias calibration.
- The firmware blocks forward motion when the ultrasonic reading is at or below **5 cm**. This is a near-contact stop threshold, not a general-purpose obstacle detector.
- Waypoints are laptop-side visual references. They are cleared after an ESP32 disconnect/reconnect and do not trigger autonomous movement.
- Before OTA, use an image built for this board and confirm the ESP32 has a working network path back to the laptop.

See [TECHNICAL_README.md](TECHNICAL_README.md) for implementation details, conventions, calibration values, and limitations.
