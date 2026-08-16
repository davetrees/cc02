#!/usr/bin/env python3
"""CC-02 Mac-side PILOT.

Holds a CONSTANT throttle and continuously drives the car via the Pi debug
port (http://pi.local:8001/exec). Exposes:
  - a dashboard at http://localhost:2020  (set avg speed, see logs, ESTOP)
  - a thin API the autonomous steering brain calls:
      POST /steer   {"steer": -1..1}      set steering (POSITIVE=LEFT)
      POST /estop                          emergency stop (latches)
      POST /resume                         clear estop
      POST /speed   {"us": 0..75}          set constant speed (us deviation)
      GET  /status                         json state
      GET  /logs                           recent log lines

The car goes FORWARD on NEGATIVE user_throttle (drivetrain inversion), so
speed_us is converted to a negative throttle fraction here.
"""
import json
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PI_EXEC = "http://pi.local:8001/exec"
PUSH_HZ = 8.0
SPEED_MAX_US = 75          # hard ceiling per owner
SPEED_DEFAULT_US = 50

S = {
    "steer": 0.0,
    "speed_us": SPEED_DEFAULT_US,
    "estop": False,
    "connected": False,
    "last_ok": 0.0,
    "esc": None,
    "roll": None,
}
LOGS = deque(maxlen=200)
_lock = threading.Lock()


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    LOGS.append(line)
    print(line, flush=True)


def pi_exec(code, timeout=1.5):
    body = json.dumps({"code": code}).encode()
    req = urllib.request.Request(PI_EXEC, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def control_loop():
    period = 1.0 / PUSH_HZ
    while True:
        t0 = time.time()
        with _lock:
            estop = S["estop"]
            steer = max(-1.0, min(1.0, float(S["steer"])))
            speed_us = max(0, min(SPEED_MAX_US, int(S["speed_us"])))
        cruise = -(speed_us / 500.0)   # forward = negative
        try:
            if estop:
                code = ("st.estop=True\n"
                        "st.user_steer=0.0\nst.user_throttle=0.0\n"
                        "st.last_input_time=__import__('time').monotonic()\n"
                        "t=st.telem or {}\nprint('OUT',t.get('esc_us'),round(t.get('roll',0)*57.3,1))")
            else:
                code = ("import time as _t\n"
                        "st.estop=False\n"
                        "if st.mode!=0: st.set_mode(0)\n"
                        f"st.user_steer={steer:.4f}\nst.user_throttle={cruise:.4f}\n"
                        "st.last_input_time=_t.monotonic()\nst.last_input_src='pilot'\n"
                        "t=st.telem or {}\nprint('OUT',t.get('esc_us'),round(t.get('roll',0)*57.3,1))")
            r = pi_exec(code)
            ok = r.get("ok")
            with _lock:
                S["connected"] = bool(ok)
                if ok:
                    S["last_ok"] = time.time()
                    for ln in (r.get("stdout") or "").splitlines():
                        if ln.startswith("OUT"):
                            p = ln.split()
                            if len(p) >= 3:
                                S["esc"], S["roll"] = p[1], p[2]
                if not ok:
                    log(f"push error: {r.get('error','?')[:80]}")
        except Exception as e:
            with _lock:
                S["connected"] = False
            log(f"push exception: {e}")
        dt = period - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)


PAGE = """<!doctype html><meta charset=utf-8><title>CC-02 Pilot</title>
<style>
body{background:#111;color:#eee;font-family:system-ui;margin:0;padding:16px}
h1{font-size:20px;margin:0 0 12px}
.row{display:flex;gap:12px;align-items:center;margin:10px 0;flex-wrap:wrap}
button{font-size:18px;padding:14px 20px;border:0;border-radius:8px;color:#fff;cursor:pointer}
#estop{background:#c22;font-size:26px;padding:22px 40px;font-weight:700}
#resume{background:#282}
.set{background:#2a3a4a}
input{font-size:20px;width:90px;padding:8px;border-radius:6px;border:1px solid #444;background:#1a1a1a;color:#eee}
.badge{padding:4px 10px;border-radius:6px;background:#333}
.ok{background:#282}.bad{background:#a22}
#log{background:#000;height:300px;overflow:auto;padding:8px;font:12px/1.4 monospace;border-radius:6px;white-space:pre-wrap}
.big{font-size:28px;font-weight:700}
</style>
<h1>CC-02 Pilot</h1>
<div class=row>
  <button id=estop onclick=estop()>E-STOP</button>
  <button id=resume class=resume onclick=resume()>RESUME</button>
  <span id=st class=badge>...</span>
</div>
<div class=row>
  avg speed <input id=spd type=number min=0 max=75 step=5 value=50> &micro;s
  <button class=set onclick=setspd()>Set</button>
  <span>(max 75)</span>
</div>
<div class=row>
  <span>steer <b id=steer class=big>0.00</b></span>
  <span>esc <b id=esc>-</b>&micro;s</span>
  <span>roll <b id=roll>-</b>&deg;</span>
</div>
<div id=log></div>
<script>
function post(p,b){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})})}
function estop(){post('/estop')}
function resume(){post('/resume')}
function setspd(){post('/speed',{us:+document.getElementById('spd').value})}
async function tick(){
  try{
    let s=await (await fetch('/status')).json();
    let el=document.getElementById('st');
    el.textContent=(s.estop?'ESTOP':(s.connected?'DRIVING':'NO LINK'))+' | '+s.speed_us+'us';
    el.className='badge '+(s.estop?'bad':(s.connected?'ok':'bad'));
    document.getElementById('steer').textContent=(+s.steer).toFixed(2);
    document.getElementById('esc').textContent=s.esc||'-';
    document.getElementById('roll').textContent=s.roll||'-';
    let lg=await (await fetch('/logs')).text();
    let l=document.getElementById('log');let b=l.scrollTop+l.clientHeight>=l.scrollHeight-20;
    l.textContent=lg;if(b)l.scrollTop=l.scrollHeight;
  }catch(e){}
}
setInterval(tick,500);tick();
</script>
"""


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def log_message(self, *a):
        pass

    def _body(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode() or "{}")
        except Exception:
            return {}

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE, "text/html")
        elif self.path == "/status":
            with _lock:
                st = dict(S)
            st["link_age"] = round(time.time() - st["last_ok"], 1)
            self._send(200, json.dumps(st))
        elif self.path == "/logs":
            self._send(200, "\n".join(LOGS), "text/plain")
        else:
            self._send(404, "{}")

    def do_POST(self):
        d = self._body()
        if self.path == "/steer":
            with _lock:
                S["steer"] = max(-1.0, min(1.0, float(d.get("steer", 0))))
            self._send(200, json.dumps({"ok": True, "steer": S["steer"]}))
        elif self.path == "/estop":
            with _lock:
                S["estop"] = True
            log("ESTOP")
            self._send(200, json.dumps({"ok": True, "estop": True}))
        elif self.path == "/resume":
            with _lock:
                S["estop"] = False
            log("RESUME")
            self._send(200, json.dumps({"ok": True, "estop": False}))
        elif self.path == "/speed":
            with _lock:
                S["speed_us"] = max(0, min(SPEED_MAX_US, int(d.get("us", SPEED_DEFAULT_US))))
            log(f"speed set {S['speed_us']}us")
            self._send(200, json.dumps({"ok": True, "speed_us": S["speed_us"]}))
        else:
            self._send(404, "{}")


if __name__ == "__main__":
    log("pilot starting; constant-throttle driver + :2020 dashboard")
    threading.Thread(target=control_loop, daemon=True).start()
    ThreadingHTTPServer(("127.0.0.1", 2020), H).serve_forever()
