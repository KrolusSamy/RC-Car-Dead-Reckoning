import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# --- CONFIGURATION ---
ESP32_IP = "192.168.137.206"
ESP32_PORT = 12345
HOST_IP = "192.168.137.1"  # Laptop hotspot address used by the ESP32 OTA configuration.
HOST_PORT = 8080
ALLOWED_COMMANDS = {"CMD_FWD", "CMD_REV", "CMD_LEFT", "CMD_RIGHT"}
# ---------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="theme-color" content="#08111f">
  <title>RC Car Controller</title>
  <style>
    :root { color-scheme: dark; --bg:#08111f; --panel:#111d2d; --line:#26384d; --text:#edf5ff; --muted:#99adc3; --blue:#56b8ff; --green:#42d7a0; --red:#ff6878; }
    * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
    body { margin:0; min-height:100vh; min-height:100svh; padding:clamp(12px,3vh,28px) 16px calc(16px + env(safe-area-inset-bottom)); display:grid; place-items:center; background:radial-gradient(ellipse at 50% 0%,#172d46 0%,var(--bg) 60%); color:var(--text); font-family:Inter,"Segoe UI",system-ui,sans-serif; user-select:none; }
    main { width:min(100%,440px); }
    .shell { border:1px solid rgba(151,190,224,.16); border-radius:26px; padding:clamp(18px,5vw,30px); background:linear-gradient(160deg,rgba(21,36,55,.98),rgba(12,22,36,.98)); box-shadow:0 24px 70px rgba(0,0,0,.38); }
    header { display:flex; align-items:center; gap:14px; margin-bottom:20px; }
    .brand { width:48px;height:48px;display:grid;place-items:center;border-radius:16px;background:linear-gradient(145deg,#17436a,#142b43);color:var(--blue);font-size:24px;box-shadow:inset 0 0 0 1px #315876; }
    h1 { margin:0; font-size:clamp(21px,6vw,27px); letter-spacing:-.04em; }
    .subtitle { color:var(--muted);font-size:13px;margin:4px 0 0; }
    .connection { display:flex;align-items:center;gap:8px;width:fit-content;padding:8px 11px;border:1px solid var(--line);border-radius:999px;background:#0b1725;color:#b9cadb;font-size:12px;margin:0 0 14px; }
    .dot { width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 10px rgba(66,215,160,.7); }
    .notice { display:none;margin:0 0 16px;padding:13px 14px;border:1px solid rgba(255,104,120,.42);border-radius:14px;background:rgba(116,31,46,.23);color:#ffb1ba;font-weight:700;font-size:14px;line-height:1.4; }
    .notice.show { display:block; }
    .pad { display:grid;grid-template-columns:repeat(3,minmax(0,1fr));grid-template-rows:repeat(3,clamp(68px,10vh,82px));gap:10px;max-width:350px;margin:20px auto 16px; }
    button.drive { border:1px solid #30475e;border-radius:20px;background:linear-gradient(160deg,#20364c,#16283a);color:#f3f8ff;font-family:inherit;font-size:14px;font-weight:700;line-height:1.2;box-shadow:0 6px 0 #0a1420,0 10px 22px rgba(0,0,0,.2);touch-action:manipulation;cursor:pointer;transition:transform .1s,border-color .1s,background .1s; }
    button.drive:hover { border-color:#5aa6d9;background:linear-gradient(160deg,#294966,#1a344d); }
    button.drive:active { transform:translateY(4px);box-shadow:0 2px 0 #0a1420; }
    button.drive:disabled { opacity:.55;cursor:wait; }
    .forward { grid-area:1/2; }.left { grid-area:2/1; }.right { grid-area:2/3; }.reverse { grid-area:3/2; }
    .arrow { display:block;font-size:22px;line-height:1;margin-bottom:5px;color:var(--blue); }
    .center { grid-area:2/2;display:grid;place-items:center;color:#72869a;font-size:11px;letter-spacing:.11em;text-transform:uppercase; }
    .feedback { min-height:42px;display:flex;align-items:center;justify-content:center;text-align:center;padding:10px 12px;border-radius:13px;background:#0a1522;border:1px solid #1f3144;color:#c7d8e8;font-size:13px; }
    .feedback.error { color:#ffb1ba;border-color:rgba(255,104,120,.4); }
    .tip { text-align:center;color:var(--muted);font-size:12px;line-height:1.55;margin:16px 2px 0; }
    footer { text-align:center;color:#687e94;font-size:11px;margin-top:16px; }
    @media(max-height:650px) { body { padding-top:8px;padding-bottom:8px; }.shell { padding:16px 20px;border-radius:20px; }.pad { grid-template-rows:repeat(3,58px);gap:7px;margin:12px auto; }.connection { margin-bottom:8px; }.tip { margin-top:10px; } }
    @media(max-width:360px) { .pad { gap:7px;grid-template-rows:repeat(3,68px); }.shell { padding:17px; } }
  </style>
</head>
<body>
  <main>
    <section class="shell" aria-label="Robot movement controller">
      <header>
        <div class="brand" aria-hidden="true">⌁</div>
        <div><h1>RC Car Controller</h1><p class="subtitle">Four simple movement taps</p></div>
      </header>
      <div class="connection"><span class="dot"></span><span id="network">Controller page is open</span></div>
      <div id="obstacle" class="notice" role="status" aria-live="assertive">Obstacle detected — forward movement is blocked.</div>
      <div class="pad" aria-label="Movement controls">
        <button class="drive forward" data-command="CMD_FWD"><span class="arrow">▲</span>Forward</button>
        <button class="drive left" data-command="CMD_LEFT"><span class="arrow">◀</span>Left</button>
        <div class="center" aria-hidden="true">Tap<br>to move</div>
        <button class="drive right" data-command="CMD_RIGHT"><span class="arrow">▶</span>Right</button>
        <button class="drive reverse" data-command="CMD_REV"><span class="arrow">▼</span>Reverse</button>
      </div>
      <div id="feedback" class="feedback" aria-live="polite">Ready for a command</div>
      <p class="tip">Each press sends one short movement. Watch the ground-station map between taps. Forward is inhibited when the ESP32 reports an obstacle.</p>
    </section>
    <footer>Robot control network · Keep the car in view</footer>
  </main>
  <script>
    const feedback = document.getElementById('feedback');
    const obstacleBanner = document.getElementById('obstacle');
    const buttons = [...document.querySelectorAll('button.drive')];
    let commandPending = false;
    let wasBlocked = false;
    document.getElementById('network').textContent = `Connected to ${window.location.host}`;
    async function sendCommand(command) {
      if (commandPending) return;
      commandPending = true;
      buttons.forEach(button => button.disabled = true);
      feedback.classList.remove('error');
      feedback.textContent = `Sending ${command.replace('CMD_', '').toLowerCase()}…`;
      try {
        const response = await fetch(`/cmd?action=${encodeURIComponent(command)}`, {cache:'no-store'});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        feedback.textContent = `Command sent: ${command.replace('CMD_', '').toLowerCase()}`;
      } catch (error) {
        feedback.classList.add('error');
        feedback.textContent = 'Could not send command. Check Wi-Fi and the controller server.';
      } finally {
        commandPending = false;
        buttons.forEach(button => button.disabled = false);
      }
    }
    buttons.forEach(button => button.addEventListener('click', () => sendCommand(button.dataset.command)));
    async function updateObstacle() {
      try {
        const response = await fetch(`http://${window.location.hostname}:8070/obstacle_state`, {cache:'no-store'});
        const status = await response.json();
        obstacleBanner.classList.toggle('show', Boolean(status.blocked));
        if (status.blocked && !wasBlocked && navigator.vibrate) navigator.vibrate([140,80,140]);
        wasBlocked = Boolean(status.blocked);
      } catch (_) { /* Ground-station alert is optional; movement requests still work. */ }
    }
    updateObstacle();
    setInterval(updateObstacle, 500);
  </script>
</body>
</html>
"""


class ControllerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/cmd":
            action = parse_qs(parsed.query).get("action", [""])[0]
            if action not in ALLOWED_COMMANDS:
                self.send_error(400, "Invalid movement command")
                return
            try:
                sock.sendto(action.encode("ascii"), (ESP32_IP, ESP32_PORT))
                print(f"[WEB] Sent {action} to ESP32 {ESP32_IP}:{ESP32_PORT}")
                self.send_response(202)
                body = b"Command accepted"
            except OSError as exc:
                print(f"[WEB] UDP send failed: {exc}")
                self.send_response(503)
                body = b"Command could not be sent"
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        print("[WEB] " + fmt % args)


if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", HOST_PORT), ControllerHandler)
    print("=" * 58)
    print(" RC Car web controller is running")
    print(f" On hotspot Wi-Fi: http://{HOST_IP}:{HOST_PORT}/")
    print(f" On this laptop:   http://localhost:{HOST_PORT}/")
    print("=" * 58)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping web controller…")
    finally:
        server.server_close()
