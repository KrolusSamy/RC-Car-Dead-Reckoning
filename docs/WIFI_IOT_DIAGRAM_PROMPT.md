# Wi-Fi / IoT architecture diagram prompt

Use this prompt in an image or diagram generator when a separate network-flow illustration is needed:

> Create a clean, presentation-ready **2D network and data-flow architecture diagram** for a small ESP32-C6 robot project. Use a white background, restrained navy/teal/blue palette, high contrast, generous spacing, crisp vector-like linework, and large readable labels. Use exactly three main devices: **Laptop / Ground Station**, **ESP32-C6 Robot**, and **Phone / Web Controller**. Show the laptop creating a Wi-Fi hotspot; the ESP32 joins it in **station mode** and receives a DHCP address (example label `192.168.137.206`, with a note that this can change). Show the laptop hotspot address as `192.168.137.1` for the tested Windows setup, marked as an example.
>
> Draw these clearly separated flows with arrowheads and protocol labels:
> 1. **ESP32 → Laptop: UDP port 12345** — periodic pose/sensor telemetry (`TELEM1`, `TELEM2`, `TELEM3`) and status heartbeats (`WARMUP`, `CALIBRATING`, `READY`).
> 2. **Laptop → ESP32: UDP port 12345** — short movement commands (`CMD_FWD`, `CMD_REV`, `CMD_LEFT`, `CMD_RIGHT`), gyro calibration request (`CAL`), and OTA trigger (`OTA_UPDATE`).
> 3. **Phone browser → Laptop: HTTP TCP port 8080** — loads the four-button web controller and submits one short tap at a time. The laptop controller process forwards the approved movement command to the ESP32 over UDP 12345. The phone does not send commands directly to the ESP32.
> 4. **ESP32 → Laptop: HTTP TCP port 8070** — after the OTA trigger, the ESP32 downloads `/dead_reckoning.bin` from the laptop's selected firmware file.
> 5. **Phone browser → Laptop: HTTP TCP port 8070, `/obstacle_state`** — polls the ground station for an obstacle-blocked flag; the phone can vibrate on a newly blocked state.
>
> Inside the ESP32 device, show small labeled blocks for **wheel encoders**, **MPU6050**, and **HC-SR04** feeding the firmware's pose/safety logic. Inside the laptop show **server.py: map, telemetry, waypoints, OTA file server** and **web_controller.py: mobile page + UDP command forwarder**. Make clear that waypoints are visual references only and the robot is manually driven; do not depict autonomous waypoint navigation. Make the Laptop the center of Wi-Fi coordination, and do not draw an Internet/cloud connection. Add a small note: “All devices must be on the same hotspot; allow the listed ports through the laptop firewall.” Do not include any Wi-Fi password or secret credential. Use consistent typography, align the device blocks on a grid, avoid crossed arrows, and ensure all labels remain legible when displayed on a presentation slide.
