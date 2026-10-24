import http.server
import json
import os
import socket
import threading
import math
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Laptop ground station: ESP32 telemetry/commands use UDP 12345.
UDP_HOST = "0.0.0.0"
UDP_PORT = 12345
ESP32_IP = "192.168.137.206"
OTA_HOST = "0.0.0.0"
OTA_PORT = 8070
OTA_FILE = os.path.join(os.path.dirname(__file__), "ota_payload", "dead_reckoning.bin")
WAYPOINT_FILE = os.path.join(os.path.dirname(__file__), "waypoints.json")
selected_firmware = OTA_FILE
ota_file_lock = threading.Lock()
MAP_METERS_PER_CELL = 0.1

state = {"status": "Waiting for ESP32 telemetry", "x": 0.0, "y": 0.0,
         "theta": 0.0, "theta_unwrapped": 0.0, "last_theta": None,
         "ticks_l": 0, "ticks_r": 0, "gyro_z": 0.0, "bias_z": 0.0,
         "pose_seen": False, "obstacle": False, "peer": "—", "velocity": 0.0,
         "omega": 0.0, "pitch": 0.0, "roll": 0.0, "pose_flags": 0,
         "encoder_heading": 0.0, "ultrasonic_cm": None, "obstacle_alert": False,
         "last_packet_time": 0.0}
state_lock = threading.Lock()
updates = []
pose_history = []
comparison_history = []
obstacle_points = []
waypoints = []
selected_waypoint = None
command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def load_waypoints():
    global waypoints
    try:
        with open(WAYPOINT_FILE, "r", encoding="utf-8") as waypoint_file:
            loaded = json.load(waypoint_file)
        if isinstance(loaded, list):
            waypoints = loaded
    except (OSError, json.JSONDecodeError):
        waypoints = []


def save_waypoints():
    temporary = WAYPOINT_FILE + ".tmp"
    with open(temporary, "w", encoding="utf-8") as waypoint_file:
        json.dump(waypoints, waypoint_file, indent=2)
    os.replace(temporary, WAYPOINT_FILE)


class OtaHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/obstacle_state":
            with state_lock:
                response = json.dumps({"blocked": bool(state["obstacle"])})
            payload = response.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(payload)
            return
        with ota_file_lock:
            firmware_path = selected_firmware
        if self.path != "/dead_reckoning.bin" or not os.path.isfile(firmware_path):
            self.send_error(404, "Firmware file not found")
            return
        size = os.path.getsize(firmware_path)
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(size))
        self.end_headers()
        with open(firmware_path, "rb") as firmware:
            while chunk := firmware.read(8192):
                self.wfile.write(chunk)

    def log_message(self, fmt, *args):
        print("[OTA] " + fmt % args)


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_HOST, UDP_PORT))
    print(f"Ground station listening for ESP32 UDP on {UDP_HOST}:{UDP_PORT}")
    while True:
        payload, address = sock.recvfrom(2048)
        message = payload.decode("utf-8", errors="replace").strip()
        with state_lock:
            state["peer"] = address[0]
            state["last_packet_time"] = time.monotonic()
        if message.startswith("STATUS:"):
            with state_lock:
                state["status"] = message.partition(":")[2]
                if state["status"] == "CALIBRATING":
                    # Pose estimation restarts at the origin after calibration;
                    # start a fresh set of comparison paths at the same point.
                    pose_history.clear()
                    comparison_history.clear()
                    state["last_theta"] = None
                    state["theta_unwrapped"] = 0.0
        elif message.startswith("POSE:"):
            try:
                x, y, theta = (float(v) for v in message.partition(":")[2].split(","))
                with state_lock:
                    _update_pose(x, y, theta)
            except (ValueError, TypeError):
                print(f"[UDP] Invalid pose packet: {message}")
        elif message.startswith("TELEM1,"):
            try:
                _, _timestamp, x, y, theta, ticks_l, ticks_r, gyro_z, bias_z = message.split(",")
                with state_lock:
                    _update_pose(float(x), float(y), float(theta))
                    state.update(ticks_l=int(ticks_l), ticks_r=int(ticks_r),
                                 gyro_z=float(gyro_z), bias_z=float(bias_z))
            except (ValueError, TypeError):
                print(f"[UDP] Invalid TELEM1 packet: {message}")
        elif message.startswith("TELEM2,"):
            try:
                _, _timestamp, velocity, omega, pitch, roll, flags, encoder_heading = message.split(",")
                with state_lock:
                    state.update(velocity=float(velocity), omega=float(omega), pitch=float(pitch),
                                 roll=float(roll), pose_flags=int(flags),
                                 encoder_heading=float(encoder_heading))
            except (ValueError, TypeError):
                print(f"[UDP] Invalid TELEM2 packet: {message}")
        elif message.startswith("TELEM3,"):
            try:
                fields = message.split(",")
                if len(fields) != 14:
                    raise ValueError("TELEM3 field count")
                record = {
                    "main": tuple(float(v) for v in fields[2:5]),
                    "encoder": tuple(float(v) for v in fields[5:8]),
                    "boot_gyro": tuple(float(v) for v in fields[8:11]),
                    "imu": tuple(float(v) for v in fields[11:14]),
                }
                with state_lock:
                    comparison_history.append(record)
                    if len(comparison_history) > 3000:
                        del comparison_history[:len(comparison_history) - 3000]
            except (ValueError, TypeError):
                print(f"[UDP] Invalid TELEM3 packet: {message}")
        elif message.startswith("OBSTACLE_RANGE,"):
            try:
                distance_cm = float(message.split(",", 1)[1])
                with state_lock:
                    # Defend the display as well as the firmware: only
                    # readings at or below 5 cm indicate an active obstacle.
                    if distance_cm > 5.0:
                        state.update(obstacle=False, ultrasonic_cm=None)
                        obstacle_points.clear()
                        continue
                    state["ultrasonic_cm"] = distance_cm
                    angle = math.radians((state["theta"] + 180.0) % 360.0)
                    ox = -state["x"] + (distance_cm / 100.0 + 0.07) * math.cos(angle)
                    oy = -state["y"] + (distance_cm / 100.0 + 0.07) * math.sin(angle)
                    # This is a live proximity marker, not a persistent map
                    # survey; keep only the current reading while the <=5 cm trigger is active.
                    obstacle_points[:] = [{"x": ox, "y": oy, "distance": distance_cm}]
                    state["obstacle"] = True
            except (ValueError, TypeError):
                print(f"[UDP] Invalid obstacle packet: {message}")
        elif message == "OBSTACLE_BLOCKED":
            with state_lock:
                state["obstacle"] = True
                state["obstacle_alert"] = True
        elif message == "OBSTACLE_CLEAR":
            with state_lock:
                state["obstacle"] = False
                state["ultrasonic_cm"] = None
                obstacle_points.clear()
        if not message.startswith("TELEM3,"):
            updates.append(message)
            print(f"[ESP32 {address[0]}] {message}")


def _update_pose(x, y, theta):
    """Update wrapped display heading and a continuous heading for turn trials."""
    previous = state["last_theta"]
    if previous is None:
        state["theta_unwrapped"] = theta
    else:
        delta = (theta - previous + 180.0) % 360.0 - 180.0
        state["theta_unwrapped"] += delta
    state.update(x=x, y=y, theta=theta, last_theta=theta, pose_seen=True)
    # Display frame is rotated 180 degrees from the firmware frame so forward
    # travel appears upward on the ground-station map.
    pose_history.append((-x, -y, (theta + 180.0) % 360.0))
    if len(pose_history) > 1200:
        del pose_history[:len(pose_history) - 1200]


def start_ota_server():
    try:
        http.server.ThreadingHTTPServer((OTA_HOST, OTA_PORT), OtaHandler).serve_forever()
    except OSError as exc:
        print(f"[OTA] HTTP server failed on port {OTA_PORT}: {exc}")


def send_command(command):
    target_ip = state["peer"] if state["peer"] != "—" else ESP32_IP
    command_socket.sendto(command.encode("ascii"), (target_ip, UDP_PORT))
    print(f"[GROUND] Sent {command} to {target_ip}:{UDP_PORT}")


def run_ui():
    global selected_waypoint
    root = tk.Tk()
    root.title("Robot Ground Station")
    root.geometry("1280x860")
    root.minsize(960, 640)
    root.configure(background="#f3f6fa")
    try:
        root.state("zoomed")  # Maximized to the work area, keeping the Windows taskbar clear.
    except tk.TclError:
        pass
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass
    style.configure("TFrame", background="#f3f6fa")
    style.configure("Card.TFrame", background="#ffffff")
    style.configure("TLabel", background="#f3f6fa", foreground="#26364a", font=("Segoe UI", 10))
    style.configure("Title.TLabel", background="#f3f6fa", foreground="#142b45", font=("Segoe UI", 20, "bold"))
    style.configure("Status.TLabel", background="#f3f6fa", foreground="#16794f", font=("Segoe UI", 11, "bold"))
    style.configure("Section.TLabelframe", background="#ffffff", foreground="#31516d", bordercolor="#d5dee8")
    style.configure("Section.TLabelframe.Label", background="#ffffff", foreground="#31516d", font=("Segoe UI", 10, "bold"))
    style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=(10, 6), background="#e7eef6", foreground="#18344e", bordercolor="#cbd7e3")
    style.map("TButton", background=[("active", "#d4e5f4"), ("disabled", "#eef1f4")], foreground=[("disabled", "#8795a3")])
    style.configure("TCheckbutton", background="#ffffff", foreground="#26364a")
    style.configure("TSpinbox", padding=4)
    style.configure("TEntry", fieldbackground="#ffffff", foreground="#26364a")

    root.columnconfigure(0, weight=1)
    root.rowconfigure(2, weight=1)
    status_var = tk.StringVar(value=state["status"])
    details_var = tk.StringVar(value="Waiting for pose telemetry")
    header = ttk.Frame(root)
    header.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 4))
    header.columnconfigure(0, weight=1)
    ttk.Label(header, text="ROBOT GROUND STATION", style="Title.TLabel").grid(row=0, column=0, sticky="w")
    ttk.Label(header, textvariable=status_var, style="Status.TLabel").grid(row=0, column=1, sticky="e", padx=8)
    ttk.Label(header, textvariable=details_var, foreground="#64788d").grid(row=1, column=0, columnspan=2, sticky="w", pady=(3, 0))

    map_controls = ttk.Frame(root, style="Card.TFrame", padding=(10, 7))
    map_controls.grid(row=1, column=0, sticky="ew", padx=16, pady=4)
    ttk.Label(map_controls, text="Map scale (cm/grid)").pack(side="left")
    scale_var = tk.StringVar(value="10")
    ttk.Spinbox(map_controls, from_=5, to=100, increment=5, textvariable=scale_var, width=6).pack(side="left", padx=5)
    auto_follow = tk.BooleanVar(value=True)
    ttk.Checkbutton(map_controls, text="Auto-follow robot", variable=auto_follow).pack(side="left", padx=6)
    ttk.Button(map_controls, text="Zoom +", command=lambda: zoom_map(-2)).pack(side="left", padx=3)
    ttk.Button(map_controls, text="Zoom −", command=lambda: zoom_map(2)).pack(side="left", padx=3)
    ttk.Button(map_controls, text="Compare estimators", command=lambda: open_comparison()).pack(side="left", padx=8)
    map_frame = ttk.Frame(root)
    map_frame.grid(row=2, column=0, sticky="nsew", padx=16, pady=4)
    canvas = tk.Canvas(map_frame, background="#fbfdff", highlightthickness=1, highlightbackground="#d5dee8")
    hbar = ttk.Scrollbar(map_frame, orient="horizontal", command=canvas.xview)
    vbar = ttk.Scrollbar(map_frame, orient="vertical", command=canvas.yview)
    canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
    canvas.grid(row=0, column=0, sticky="nsew")
    vbar.grid(row=0, column=1, sticky="ns")
    hbar.grid(row=1, column=0, sticky="ew")
    map_frame.rowconfigure(0, weight=1)
    map_frame.columnconfigure(0, weight=1)
    canvas.configure(width=850, height=360)
    canvas.bind("<MouseWheel>", lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"))
    waypoint_frame = ttk.LabelFrame(root, text="SAVED MAP REFERENCES", style="Section.TLabelframe", padding=8)
    waypoint_frame.grid(row=3, column=0, sticky="ew", padx=16, pady=4)
    waypoint_frame.columnconfigure(1, weight=1)
    waypoint_name = tk.StringVar(value="Waypoint")
    waypoint_list = tk.Listbox(waypoint_frame, height=2, width=34, exportselection=False,
                               bg="#ffffff", fg="#26364a", selectbackground="#d8eafb",
                               selectforeground="#18344e", relief="flat", highlightthickness=1,
                               highlightbackground="#d5dee8", font=("Segoe UI", 9))
    waypoint_list.grid(row=0, column=0, rowspan=2, sticky="nsw", padx=(0, 10))
    waypoint_status = tk.StringVar(value="Home is saved automatically after the first valid pose. These points are visual map references only.")
    ttk.Label(waypoint_frame, text="Name").grid(row=0, column=1, sticky="w")
    ttk.Entry(waypoint_frame, textvariable=waypoint_name, width=18).grid(row=0, column=1, sticky="w", padx=(42, 8))
    waypoint_buttons = ttk.Frame(waypoint_frame, style="Card.TFrame")
    waypoint_buttons.grid(row=0, column=2, sticky="e")
    ttk.Button(waypoint_buttons, text="Save current pose", command=lambda: save_current_waypoint()).pack(side="left", padx=3)
    ttk.Button(waypoint_buttons, text="Highlight", command=lambda: highlight_waypoint()).pack(side="left", padx=3)
    ttk.Button(waypoint_buttons, text="Clear all", command=lambda: clear_waypoints()).pack(side="left", padx=3)
    ttk.Label(waypoint_frame, textvariable=waypoint_status, foreground="#91a7bd").grid(row=1, column=1, columnspan=2, sticky="w", pady=(4, 0))

    log_frame = ttk.LabelFrame(root, text="LIVE MESSAGES", style="Section.TLabelframe", padding=6)
    log_frame.grid(row=4, column=0, sticky="ew", padx=16, pady=4)
    log_frame.columnconfigure(0, weight=1)
    log = tk.Listbox(log_frame, height=4, bg="#f8fafc", fg="#26364a", selectbackground="#d8eafb",
                     relief="flat", highlightthickness=1, highlightbackground="#d5dee8", font=("Consolas", 9))
    log.grid(row=0, column=0, sticky="ew")
    log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=log.yview)
    log_scroll.grid(row=0, column=1, sticky="ns")
    log.configure(yscrollcommand=log_scroll.set)

    maintenance = ttk.LabelFrame(root, text="FIRMWARE & CALIBRATION", style="Section.TLabelframe", padding=(10, 6))
    maintenance.grid(row=5, column=0, sticky="ew", padx=16, pady=(4, 10))
    maintenance.columnconfigure(1, weight=1)
    firmware_var = tk.StringVar(value=f"Selected: {os.path.basename(selected_firmware)}" if os.path.isfile(selected_firmware) else "No firmware file selected")

    def choose_firmware():
        global selected_firmware
        path = filedialog.askopenfilename(
            title="Choose ESP32 firmware image",
            filetypes=[("ESP-IDF firmware binary", "*.bin"), ("All files", "*.*")],
        )
        if path:
            with ota_file_lock:
                selected_firmware = path
            firmware_var.set(f"Selected: {os.path.basename(path)}")

    def start_ota():
        with ota_file_lock:
            path = selected_firmware
        if not os.path.isfile(path):
            messagebox.showerror("OTA firmware missing", "Choose an existing firmware .bin file first.")
            return
        if not path.lower().endswith(".bin"):
            messagebox.showerror("Invalid firmware file", "Choose an ESP32 .bin firmware image.")
            return
        send_command("OTA_UPDATE")

    ttk.Button(maintenance, text="Choose firmware .bin", command=choose_firmware).grid(row=0, column=0, padx=3, pady=2, sticky="w")
    ttk.Label(maintenance, textvariable=firmware_var, foreground="#91a7bd").grid(row=0, column=1, sticky="w", padx=8)
    ttk.Button(maintenance, text="Force gyro calibration", command=lambda: force_calibration()).grid(row=0, column=2, padx=3, pady=2)
    ttk.Button(maintenance, text="Start OTA update", command=start_ota).grid(row=0, column=3, padx=3, pady=2)
    ttk.Label(maintenance, text=f"OTA file served to ESP32 on port {OTA_PORT}  •  Calibration requires the robot to remain still.",
              foreground="#91a7bd").grid(row=1, column=0, columnspan=4, sticky="w", padx=3, pady=(3, 0))

    auto_home_pending = True
    comparison_window = None
    comparison_odom_canvas = None
    comparison_imu_canvas = None
    comparison_odom_legend = None
    comparison_imu_legend = None

    def refresh_waypoint_list():
        waypoint_list.delete(0, tk.END)
        for point in waypoints:
            waypoint_list.insert(tk.END, f"{point['name']}  ({point['x']:.2f}, {point['y']:.2f}) m, {point['theta']:.1f}°")

    def clear_waypoints():
        global selected_waypoint
        nonlocal auto_home_pending
        if waypoints and not messagebox.askyesno("Clear waypoints", "Delete all saved waypoints from this laptop?"):
            return
        waypoints.clear()
        selected_waypoint = None
        auto_home_pending = False
        save_waypoints()
        refresh_waypoint_list()
        waypoint_status.set("All waypoints cleared. Save the current pose to create a new Home/reference.")

    def force_calibration():
        if messagebox.askyesno("Force gyro calibration", "Keep the car completely still. Calibration will stop motion and temporarily disable driving until it finishes. Continue?"):
            send_command("CAL")

    def save_current_waypoint(name=None):
        if not state["pose_seen"]:
            waypoint_status.set("Waiting for pose telemetry; cannot save a waypoint yet.")
            return
        point_name = (name or waypoint_name.get()).strip()
        if not point_name:
            point_name = f"Waypoint {len(waypoints) + 1}"
        with state_lock:
            point = {"name": point_name, "x": -state["x"], "y": -state["y"],
                     "theta": (state["theta"] + 180.0) % 360.0}
        waypoints.append(point)
        save_waypoints()
        refresh_waypoint_list()
        waypoint_status.set(f"Saved {point_name} as a visual reference. Waypoints do not control the car.")

    def highlight_waypoint():
        global selected_waypoint
        selection = waypoint_list.curselection()
        if not selection:
            waypoint_status.set("Select a waypoint first.")
            return
        selected_waypoint = selection[0]
        waypoint_status.set(f"Selected {waypoints[selected_waypoint]['name']} as the current visual reference.")

    def zoom_map(delta):
        try:
            current = float(scale_var.get())
        except ValueError:
            current = 10.0
        scale_var.set(str(max(5, min(100, current + delta))))

    def draw_comparison_panel(canvas_widget, legend_widget, track_specs):
        canvas_widget.delete("all")
        width = max(1, canvas_widget.winfo_width())
        height = max(1, canvas_widget.winfo_height())
        with state_lock:
            records = list(comparison_history)
        if not records:
            canvas_widget.create_text(width / 2, height / 2,
                                      text="Waiting for firmware comparison telemetry (TELEM3)",
                                      fill="#52677d", font=("Segoe UI", 14))
            legend_widget.configure(text="No comparison samples received yet.")
            return

        pad = 34
        plot_w, plot_h = max(1, width - 2 * pad), max(1, height - 2 * pad)
        points_by_track = {}
        all_x, all_y = [0.0], [0.0]
        for key, label, color in track_specs:
            points = [(record[key][0], record[key][1]) for record in records]
            points_by_track[key] = points
            all_x.extend(p[0] for p in points)
            all_y.extend(p[1] for p in points)
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        span_x, span_y = max(max_x - min_x, 0.05), max(max_y - min_y, 0.05)
        margin_x, margin_y = span_x * 0.08, span_y * 0.08
        min_x, max_x = min_x - margin_x, max_x + margin_x
        min_y, max_y = min_y - margin_y, max_y + margin_y
        span_x, span_y = max_x - min_x, max_y - min_y

        def to_canvas(x, y):
            return (pad + (x - min_x) / span_x * plot_w,
                    height - pad - (y - min_y) / span_y * plot_h)

        for i in range(6):
            gx = pad + i * plot_w / 5
            gy = pad + i * plot_h / 5
            canvas_widget.create_line(gx, pad, gx, height - pad, fill="#e2e8ef")
            canvas_widget.create_line(pad, gy, width - pad, gy, fill="#e2e8ef")
        ox, oy = to_canvas(0.0, 0.0)
        canvas_widget.create_oval(ox - 5, oy - 5, ox + 5, oy + 5,
                                  fill="white", outline="white")
        legend_lines = [f"X range {min_x:.2f} to {max_x:.2f} m  |  Y range {min_y:.2f} to {max_y:.2f} m"]
        for key, label, color in track_specs:
            points = points_by_track[key]
            coords = []
            for x, y in points:
                coords.extend(to_canvas(x, y))
            if len(coords) >= 4:
                canvas_widget.create_line(*coords, fill=color, width=2)
            end_x, end_y = points[-1]
            end_theta = records[-1][key][2]
            ex, ey = to_canvas(end_x, end_y)
            canvas_widget.create_oval(ex - 4, ey - 4, ex + 4, ey + 4,
                                      fill=color, outline=color)
            color_name = {"#50b8ff": "BLUE", "#facc15": "YELLOW",
                          "#fb923c": "ORANGE", "#ff6b8a": "PINK"}.get(color, "TRACK")
            legend_lines.append(
                f"{color_name} — {label}: X={end_x:.2f} m, Y={end_y:.2f} m, heading={end_theta:.1f}°, distance from start={math.hypot(end_x, end_y):.2f} m")
        legend_widget.configure(text="\n".join(legend_lines))

    def draw_comparison():
        if (comparison_odom_canvas is None or comparison_imu_canvas is None or
                comparison_odom_legend is None or comparison_imu_legend is None):
            return
        draw_comparison_panel(
            comparison_odom_canvas, comparison_odom_legend,
            [("main", "Current gyro + stationary bias", "#50b8ff"),
             ("encoder", "Encoder-only heading", "#facc15")])
        draw_comparison_panel(
            comparison_imu_canvas, comparison_imu_legend,
            [("imu", "IMU-only; no encoders or ZUPT", "#ff6b8a")])

    def open_comparison():
        nonlocal comparison_window, comparison_odom_canvas, comparison_imu_canvas
        nonlocal comparison_odom_legend, comparison_imu_legend
        if comparison_window is not None and comparison_window.winfo_exists():
            comparison_window.lift()
            draw_comparison()
            return
        comparison_window = tk.Toplevel(root)
        comparison_window.title("Dead-reckoning estimator comparison")
        comparison_window.geometry("1500x900")
        comparison_window.minsize(1200, 720)
        ttk.Label(comparison_window,
                  text="Diagnostic comparison. IMU-only double integration is expected to drift; it is not a reliable position estimate. Force-calibrate before a fresh run.",
                  wraplength=1450, font=("Segoe UI", 12)).pack(fill="x", padx=12, pady=10)
        panels = ttk.Frame(comparison_window)
        panels.pack(fill="both", expand=True, padx=8, pady=5)
        odom_panel = ttk.LabelFrame(panels, text="ENCODER DISTANCE + HEADING ESTIMATES", padding=8)
        imu_panel = ttk.LabelFrame(panels, text="IMU-ONLY ACCELERATION DOUBLE INTEGRATION", padding=8)
        odom_panel.pack(side="left", fill="both", expand=True, padx=5)
        imu_panel.pack(side="left", fill="both", expand=True, padx=5)
        comparison_odom_canvas = tk.Canvas(odom_panel, background="#fbfdff", highlightthickness=1, highlightbackground="#d5dee8")
        comparison_imu_canvas = tk.Canvas(imu_panel, background="#fbfdff", highlightthickness=1, highlightbackground="#d5dee8")
        comparison_odom_canvas.pack(fill="both", expand=True)
        comparison_imu_canvas.pack(fill="both", expand=True)
        comparison_odom_legend = ttk.Label(odom_panel, text="", justify="left", anchor="w",
                                           font=("Segoe UI", 11), wraplength=690)
        comparison_imu_legend = ttk.Label(imu_panel, text="", justify="left", anchor="w",
                                          font=("Segoe UI", 11), wraplength=690)
        comparison_odom_legend.pack(fill="x", pady=(8, 2))
        comparison_imu_legend.pack(fill="x", pady=(8, 2))
        ttk.Label(comparison_window,
                  text="Each panel uses its own scale. The white dot is the start; the colored dot is the latest estimated position.",
                  font=("Segoe UI", 11)).pack(pady=8)
        draw_comparison()

    def redraw():
        canvas.delete("all")
        try:
            meters_per_grid = max(0.05, float(scale_var.get()) / 100.0)
        except ValueError:
            meters_per_grid = 0.1
        spacing = max(12, int(40 * 0.1 / meters_per_grid))
        grid_extent = 4000
        canvas.configure(scrollregion=(-grid_extent, -grid_extent, grid_extent, grid_extent))
        center_x, center_y = canvas.canvasx(canvas.winfo_width() / 2), canvas.canvasy(canvas.winfo_height() / 2)
        visible_left, visible_top = canvas.canvasx(0), canvas.canvasy(0)
        visible_right, visible_bottom = visible_left + canvas.winfo_width(), visible_top + canvas.winfo_height()
        for gx in range(math.floor(visible_left / spacing) - 1, math.ceil(visible_right / spacing) + 1):
            canvas.create_line(gx * spacing, visible_top, gx * spacing, visible_bottom, fill="#e3eaf1")
        for gy in range(math.floor(visible_top / spacing) - 1, math.ceil(visible_bottom / spacing) + 1):
            canvas.create_line(visible_left, gy * spacing, visible_right, gy * spacing, fill="#e3eaf1")
        canvas.create_line(-14, 0, 14, 0, fill="#8295a8")
        canvas.create_line(0, -14, 0, 14, fill="#8295a8")
        for index, point in enumerate(waypoints):
            wx = point["x"] / meters_per_grid * spacing
            wy = -point["y"] / meters_per_grid * spacing
            is_selected = index == selected_waypoint
            color = "#ff61d8" if is_selected else "#fb923c"
            canvas.create_oval(wx-8, wy-8, wx+8, wy+8, fill=color, outline="white", width=2)
            canvas.create_text(wx-12, wy-12, text=point["name"], fill=color,
                               anchor="se", font=("Segoe UI", 9, "bold"))
            angle_wp = math.radians(point["theta"])
            canvas.create_line(wx, wy, wx+20*math.cos(angle_wp), wy-20*math.sin(angle_wp), fill=color, width=2, arrow=tk.LAST)
        for point in obstacle_points:
            ox = point["x"] / meters_per_grid * spacing
            oy = -point["y"] / meters_per_grid * spacing
            canvas.create_polygon(ox, oy-9, ox-8, oy+7, ox+8, oy+7,
                                 fill="#ff4d4d", outline="white")
            canvas.create_text(ox, oy+17, text="Obstacle", fill="#ff7777")
        if state["pose_seen"]:
            current_x, current_y = -state["x"], -state["y"]
            px = current_x / meters_per_grid * spacing
            py = -current_y / meters_per_grid * spacing
            if auto_follow.get():
                canvas.xview_moveto(max(0.0, min(1.0, (px - canvas.winfo_width() / 2 + grid_extent) / (2 * grid_extent))))
                canvas.yview_moveto(max(0.0, min(1.0, (py - canvas.winfo_height() / 2 + grid_extent) / (2 * grid_extent))))
            history = list(pose_history)
            if len(history) > 1:
                path_coords = []
                for hx, hy, _ in history:
                    path_coords.extend((hx / meters_per_grid * spacing, -hy / meters_per_grid * spacing))
                canvas.create_line(*path_coords, fill="#50b8ff", width=2, dash=(5, 4), smooth=True)
            angle = math.radians((state["theta"] + 180.0) % 360.0)
            tipx, tipy = px + 27 * math.cos(angle), py - 27 * math.sin(angle)
            canvas.create_oval(px-8, py-8, px+8, py+8, fill="#22d3ee", outline="white", width=2)
            canvas.create_line(px, py, tipx, tipy, fill="#facc15", width=4, arrow=tk.LAST)
            canvas.create_text(px+12, py+22, text="Robot", fill="#26364a",
                               anchor="nw", font=("Segoe UI", 9, "bold"))
            flags = state["pose_flags"]
            if flags & ((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 7)):
                if flags & (1 << 1):
                    slope_text = "RAMP UP"
                elif flags & (1 << 2):
                    slope_text = "RAMP DOWN"
                elif flags & (1 << 3):
                    slope_text = "SIDE TILT LEFT"
                elif flags & (1 << 4):
                    slope_text = "SIDE TILT RIGHT"
                else:
                    slope_text = "STEEP UP" if state["pitch"] > 0.0 else "STEEP DOWN"
                canvas.create_text(px, py-25, text=slope_text, fill="#a3e635", font=("Segoe UI", 10, "bold"))
            with state_lock:
                zupt_text = "stationary" if (state["pose_flags"] & 1) else "moving"
                details_var.set(f"Map X={-state['x']:.2f} m  Y={-state['y']:.2f} m  |  heading={(state['theta'] + 180.0) % 360.0:.1f}°  |  speed={state['velocity']:.2f} m/s  |  pitch={state['pitch']:.1f}° roll={state['roll']:.1f}°  |  {zupt_text}")
        else:
            canvas.create_text(center_x, center_y, text="Waiting for POSE:x,y,heading telemetry", fill="#52677d", font=("Segoe UI", 13))
        if state["obstacle"]:
            canvas.create_text(12, 18, anchor="w", text="FRONT OBSTACLE", fill="#ff4d4d", font=("Segoe UI", 12, "bold"))

    shown = 0
    previous_obstacle = False
    was_connected = False
    reconnect_clear_pending = False
    def refresh():
        nonlocal shown, previous_obstacle, auto_home_pending, was_connected, reconnect_clear_pending
        with state_lock:
            obstacle_now = state["obstacle"]
            obstacle_alert = state["obstacle_alert"]
            status_now, peer_now = state["status"], state["peer"]
            pose_seen = state["pose_seen"]
            packet_time = state["last_packet_time"]
        connected = packet_time > 0.0 and (time.monotonic() - packet_time) <= 3.0
        if was_connected and not connected:
            reconnect_clear_pending = True
            status_var.set("ESP32 OFFLINE — waiting for reconnect")
        elif connected and not was_connected and reconnect_clear_pending:
            global selected_waypoint
            waypoints.clear()
            selected_waypoint = None
            pose_history.clear()
            comparison_history.clear()
            obstacle_points.clear()
            with state_lock:
                state["last_theta"] = None
                state["theta_unwrapped"] = 0.0
                state["obstacle"] = False
                state["obstacle_alert"] = False
                state["ultrasonic_cm"] = None
            save_waypoints()
            refresh_waypoint_list()
            auto_home_pending = False
            waypoint_status.set("ESP32 reconnected — old waypoints and map trail cleared. Save a fresh Home reference.")
            updates.append("SYSTEM: ESP32 reconnected; cleared saved waypoints and previous map trail")
            reconnect_clear_pending = False
        if connected:
            status_var.set(("OBSTACLE BLOCKED — forward inhibited  |  " if obstacle_now else "") +
                           f"{status_now}  |  ESP32: {peer_now}")
        elif packet_time > 0.0:
            status_var.set("ESP32 OFFLINE — telemetry timed out")
        else:
            status_var.set(f"{status_now}  |  waiting for ESP32")
        style.configure("Status.TLabel", foreground="#b42332" if obstacle_now else ("#16794f" if connected else "#a45b00"))
        if obstacle_alert and not previous_obstacle:
            root.bell()
            with state_lock:
                state["obstacle_alert"] = False
        previous_obstacle = obstacle_now
        if pose_seen and auto_home_pending and not waypoints:
            save_current_waypoint("Home")
            auto_home_pending = False
        redraw()
        if comparison_window is not None and comparison_window.winfo_exists():
            draw_comparison()
        while shown < len(updates):
            log.insert(tk.END, updates[shown])
            shown += 1
        if log.size() > 300:
            log.delete(0, log.size() - 300)
        root.after(200, refresh)

    refresh()
    load_waypoints()
    auto_home_pending = not bool(waypoints)
    refresh_waypoint_list()
    root.mainloop()


if __name__ == "__main__":
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=start_ota_server, daemon=True).start()
    print(f"Default OTA binary: {OTA_FILE}; choose a .bin in the ground-station GUI to change it")
    run_ui()
