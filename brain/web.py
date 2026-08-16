"""aiohttp web panel on 0.0.0.0:8080: control page, MJPEG stream, WebSocket."""
import asyncio
import json
import math
import time

import cv2
from aiohttp import web, WSMsgType

import logger as loggermod
import protocol as P


def make_app(state, camera, vision, mapping):
    app = web.Application()
    app['state'] = state
    app['camera'] = camera
    app['vision'] = vision
    app['mapping'] = mapping
    app.router.add_get('/', index)
    app.router.add_get('/stream.mjpg', stream)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/map.json', map_json)
    app.router.add_get('/logs/latest.csv', latest_csv)
    return app


async def index(request):
    return web.Response(text=PAGE, content_type='text/html')


async def map_json(request):
    return web.json_response(request.app['mapping'].snapshot())


async def latest_csv(request):
    p = loggermod.CsvLogger.latest()
    if not p:
        return web.Response(status=404, text='no logs yet')
    return web.FileResponse(p)


async def stream(request):
    resp = web.StreamResponse(headers={
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
        'Cache-Control': 'no-cache'})
    await resp.prepare(request)
    if request.method == 'HEAD':
        return resp
    cam = request.app['camera']
    vis = request.app['vision']
    try:
        while True:
            frame = cam.get_frame()
            if frame is not None:
                frame = vis.annotate(frame)
                ok, jpg = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    b = jpg.tobytes()
                    await resp.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(b)}\r\n\r\n'.encode()
                        + b + b'\r\n')
            await asyncio.sleep(0.1)  # ~10 fps
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def ws_handler(request):
    st = request.app['state']
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    st.ws_clients += 1
    sender = asyncio.ensure_future(_telem_sender(ws, st, request.app['mapping']))
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                d = json.loads(msg.data)
            except ValueError:
                continue
            if 'steer' in d or 'throttle' in d:
                s = max(-1.0, min(1.0, float(d.get('steer', 0))))
                t = max(-1.0, min(1.0, float(d.get('throttle', 0))))
                # WEB-CLIENT-ONLY inversion (virtual sticks/keys/browser
                # gamepad). The Pi-paired physical gamepad (gamepad.py) and
                # the global inversion in main.cmd_tuple are NOT affected.
                if st.config.get('web_invert_steer', True):
                    s = -s
                if st.config.get('web_invert_throttle', True):
                    t = -t
                st.user_steer = s
                st.user_throttle = t
                st.user_direct = bool(d.get("direct", 0))
                st.last_input_time = time.monotonic()
                st.last_input_src = 'ws'
            if 'mode' in d:
                m = {'MANUAL': 0, 'ASSIST': 1, 'AUTO': 2, 'RTH': 3}.get(d['mode'])
                if m is not None:
                    st.set_mode(m)
            if 'estop' in d:
                st.estop = bool(d['estop'])
                st.log(f'web: estop {"ON" if st.estop else "OFF"}')
            if d.get('cal'):
                # LEVEL/ZERO: serial_link raises DISP DF_CAL_REQUEST for ~1s
                st.cal_request_time = time.monotonic()
                st.log('web: LEVEL/ZERO requested (AH re-zero)')
            if 'config' in d and isinstance(d['config'], dict):
                st.config.update(d['config'])
                st.save_config()
    finally:
        sender.cancel()
        st.ws_clients = max(0, st.ws_clients - 1)
    return ws


async def _telem_sender(ws, st, mapping):
    try:
        while not ws.closed:
            t = st.telem or {}
            age = (time.monotonic() - st.last_telem_time) if st.last_telem_time else -1
            out = {
                'mode': 'ESTOP' if st.estop else P.MODE_NAMES.get(st.mode, '?'),
                'estop': st.estop,
                'link': {'connected': st.link_connected,
                         'hz': round(st.telem_hz, 1),
                         'age_s': round(age, 2) if age >= 0 else None},
                'roll_deg': round(math.degrees(t.get('roll', 0)), 1),
                'pitch_deg': round(math.degrees(t.get('pitch', 0)), 1),
                'steer_us': st.out_steer_us,
                'throttle_us': st.out_throttle_us,
                'servo_us_in': t.get('servo_us', 0),
                'esc_us_in': t.get('esc_us', 0),
                'batt_mv': t.get('batt_mv', 0),
                'yolo_fps': round(st.yolo_fps, 1),
                'det_count': st.det_count,
                'collision': st.collision,
                'crumbs': len(mapping.crumbs),
                'rth_remaining': st.rth_remaining,
                'pro_controller': bool(st.gamepad_connected),
                'auto': {'state': st.auto_state,
                         'target': st.auto_target,
                         'steer_us': st.auto_steer_us,
                         'costs': st.auto_costs,
                         'accel_var': (round(st.auto_accel_var, 5)
                                       if st.auto_accel_var is not None else None)},
                'config': st.config,
            }
            await ws.send_str(json.dumps(out))
            await asyncio.sleep(0.1)  # 10 Hz
    except (asyncio.CancelledError, ConnectionResetError):
        pass


PAGE = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>CC-02 Brain</title>
<style>
body{background:#111;color:#ddd;font-family:system-ui,sans-serif;margin:0;padding:8px;
 user-select:none;-webkit-user-select:none}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:1400px;margin:auto}
.card{background:#1c1c1e;border-radius:8px;padding:10px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
#mode{font-size:2.2em;font-weight:bold;text-align:center;padding:6px;border-radius:8px;
 background:#333}
#mode.MANUAL{background:#2b5b2b}#mode.ASSIST{background:#2b4b6b}
#mode.AUTO{background:#6b5b1b}#mode.RTH{background:#6b3b6b}#mode.ESTOP{background:#8b1b1b}
button{font-size:1em;padding:10px 14px;margin:3px;border:0;border-radius:6px;
 background:#3a3a3c;color:#eee;cursor:pointer}
button.active{background:#0a84ff}
#estopBtn{background:#c62828;font-size:1.5em;font-weight:bold;width:100%;padding:18px}
#estopBtn.on{background:#ff1744;animation:blink 0.6s infinite alternate}
@keyframes blink{from{opacity:1}to{opacity:.5}}
.joy{width:150px;height:150px;background:#2a2a2c;border-radius:12px;position:relative;
 touch-action:none;display:inline-block;margin:4px}
.knob{width:44px;height:44px;background:#0a84ff;border-radius:50%;position:absolute;
 left:53px;top:53px;pointer-events:none}
.joyrow{display:flex;justify-content:space-around;flex-wrap:wrap}
img#cam{width:100%;border-radius:6px;background:#000}
canvas#map{width:100%;height:260px;background:#000;border-radius:6px}
.kv{display:grid;grid-template-columns:auto auto auto auto;gap:2px 14px;font-size:.95em}
.kv b{color:#8af}
label{display:block;margin:4px 0}
input[type=range]{width:150px;vertical-align:middle}
select{background:#2a2a2c;color:#eee;border-radius:4px}
.warn{color:#ffb300;font-size:.85em}
.badge{padding:2px 8px;border-radius:10px;background:#444;font-size:.85em}
.badge.ok{background:#2b5b2b}.badge.bad{background:#8b1b1b}
small{color:#888}
</style></head><body>
<div class="grid">
 <div class="card">
  <div id="mode">--</div>
  <div style="text-align:center;margin-top:6px">
   <button id="bMANUAL" onclick="setMode('MANUAL')">MANUAL</button>
   <button id="bASSIST" onclick="setMode('ASSIST')">ASSIST</button>
   <button id="bAUTO" onclick="setMode('AUTO')">AUTO</button>
   <button id="bRTH" onclick="setMode('RTH')" title="best-effort dead reckoning">RTH*</button>
   <button onclick="send({cal:true})" title="re-zero the artificial horizon (car level and still)">LEVEL/ZERO</button>
  </div>
  <div class="warn" style="text-align:center">RTH is BEST-EFFORT breadcrumb replay (dead reckoning, not SLAM)</div>
  <button id="estopBtn" onclick="toggleEstop()">ESTOP</button>
  <div class="joyrow">
   <div><div class="joy" id="joyT"><div class="knob" id="knobT"></div></div>
        <div style="text-align:center"><small>THROTTLE (vertical)</small></div></div>
   <div><div class="joy" id="joyS"><div class="knob" id="knobS"></div></div>
        <div style="text-align:center"><small>STEER (horizontal)</small></div></div>
  </div>
  <div><small>Keys: W/S or Up/Down = throttle, A/D or Left/Right = steer (hold to drive).
  Gamepad: left stick steer, right trigger/stick throttle.</small>
  <span id="gp" class="badge">no gamepad</span>
  <span id="proCtl" class="badge">Pro Controller: &ndash;</span></div>
 </div>

 <div class="card">
  <img id="cam" src="/stream.mjpg" alt="camera">
  <div class="kv" id="telem"></div>
  <div><span id="wsStat" class="badge">ws: connecting</span>
   <span id="linkStat" class="badge">serial: ?</span>
   <a href="/logs/latest.csv" style="color:#8af">download latest log CSV</a></div>
 </div>

 <div class="card">
  <b>Settings</b>
  <label>Max speed <input type="number" id="maxSpeedUs" min="0" max="500" step="5" value="150"
   style="width:70px">&micro;s <small>(0-500, caps throttle deviation)</small></label>
  <label>AUTO cruise speed <input type="number" id="cruiseUs" min="0" max="500" step="5" value="120"
   style="width:70px">&micro;s <small>(forward dev at full clearance)</small></label>
  <label>AUTO blocked thr <input type="number" id="blockThr" min="0" max="1" step="0.05" value="0.25"
   style="width:65px">
   steer hyst <input type="number" id="hyst" min="0" max="1" step="0.05" value="0.15"
   style="width:65px"></label>
  <label><input type="checkbox" id="antiTip" checked> Anti-tip enable —
   roll <input type="number" id="tipRoll" value="45" min="10" max="90" style="width:50px">°
   pitch <input type="number" id="tipPitch" value="45" min="10" max="90" style="width:50px">°</label>
  <label><input type="checkbox" id="yoloEn" checked> YOLO enable —
   conf <input type="range" id="yoloConf" min="10" max="90" value="35">
   <span id="yoloConfV">0.35</span></label>
  <label>Classes <select id="classes" multiple size="4">
   <option selected>person</option><option selected>car</option>
   <option selected>chair</option><option selected>dog</option>
   <option selected>cat</option><option selected>bottle</option>
   <option selected>backpack</option></select></label>
  <label><input type="checkbox" id="colStop" checked> Collision-stop enable —
   area thr <input type="range" id="colThr" min="5" max="60" value="20">
   <span id="colThrV">0.20</span></label>
 </div>

 <div class="card">
  <b>Breadcrumb map (dead-reckoning, BEST-EFFORT — not SLAM)</b>
  <canvas id="map" width="600" height="260"></canvas>
 </div>
</div>
<script>
var ws=null, tele={}, steer=0, throttle=0, inputActive=false, estop=false;
var keys={};
function connect(){
  ws=new WebSocket((location.protocol=='https:'?'wss://':'ws://')+location.host+'/ws');
  ws.onopen=function(){document.getElementById('wsStat').textContent='ws: connected';
    document.getElementById('wsStat').className='badge ok';};
  ws.onclose=function(){document.getElementById('wsStat').textContent='ws: closed, retrying';
    document.getElementById('wsStat').className='badge bad';setTimeout(connect,1000);};
  ws.onmessage=function(e){tele=JSON.parse(e.data);initCfg(tele.config);render();};
}
// populate settings from persisted config once per page load
var cfgInit=false;
function initCfg(c){
  if(cfgInit||!c)return;cfgInit=true;
  function set(id,v){if(v!=null)document.getElementById(id).value=v;}
  set('maxSpeedUs',Math.round((c.max_speed!=null?c.max_speed:0.3)*500));
  set('cruiseUs',c.auto_cruise_us);set('blockThr',c.auto_block_thr);set('hyst',c.auto_hyst);
  document.getElementById('antiTip').checked=!!c.anti_tip_enable;
  set('tipRoll',c.tip_roll_deg);set('tipPitch',c.tip_pitch_deg);
  document.getElementById('yoloEn').checked=!!c.yolo_enable;
  if(c.yolo_conf!=null){set('yoloConf',Math.round(c.yolo_conf*100));
    document.getElementById('yoloConfV').textContent=c.yolo_conf.toFixed(2);}
  document.getElementById('colStop').checked=!!c.collision_stop;
  if(c.collision_area_threshold!=null){set('colThr',Math.round(c.collision_area_threshold*100));
    document.getElementById('colThrV').textContent=c.collision_area_threshold.toFixed(2);}
  if(c.yolo_classes)Array.prototype.forEach.call(document.getElementById('classes').options,
    function(o){o.selected=c.yolo_classes.indexOf(o.value)>=0;});
}
connect();
function send(o){if(ws&&ws.readyState==1)ws.send(JSON.stringify(o));}
function setMode(m){send({mode:m});}
function toggleEstop(){estop=!tele.estop;send({estop:estop});}
function render(){
  var m=document.getElementById('mode');m.textContent=tele.mode;m.className=tele.mode;
  ['MANUAL','ASSIST','AUTO','RTH'].forEach(function(x){
    document.getElementById('b'+x).className=(tele.mode==x)?'active':'';});
  var eb=document.getElementById('estopBtn');
  eb.textContent=tele.estop?'ESTOP ACTIVE — tap to clear':'ESTOP';
  eb.className=tele.estop?'on':'';
  var l=tele.link||{};
  var pc=document.getElementById('proCtl');
  pc.textContent='Pro Controller: '+(tele.pro_controller?'connected':'–');
  pc.className='badge '+(tele.pro_controller?'ok':'');
  var ls=document.getElementById('linkStat');
  ls.textContent='serial: '+(l.connected?('up '+l.hz+'Hz'):'DOWN (reconnecting)');
  ls.className='badge '+(l.connected?'ok':'bad');
  document.getElementById('telem').innerHTML=
    '<span>roll</span><b>'+tele.roll_deg+'&deg;</b><span>pitch</span><b>'+tele.pitch_deg+'&deg;</b>'+
    '<span>steer out</span><b>'+tele.steer_us+'&micro;s</b><span>thr out</span><b>'+tele.throttle_us+'&micro;s</b>'+
    '<span>yolo</span><b>'+tele.yolo_fps+' fps</b><span>dets</span><b>'+tele.det_count+'</b>'+
    '<span>collision</span><b style="color:'+(tele.collision?'#f55':'#5f5')+'">'+tele.collision+'</b>'+
    '<span>battery</span><b>'+tele.batt_mv+' mV</b>'+
    '<span>crumbs</span><b>'+tele.crumbs+'</b><span>telem age</span><b>'+
    (tele.link&&tele.link.age_s!=null?tele.link.age_s+'s':'-')+'</b>'+
    '<span>auto</span><b>'+(tele.auto?tele.auto.state+' &rarr;col'+tele.auto.target:'-')+'</b>'+
    '<span>accel var</span><b>'+(tele.auto&&tele.auto.accel_var!=null?tele.auto.accel_var:'-')+'</b>';
}
// ---- joysticks ----
function joy(el,knob,cb){
  var active=false;
  function pos(e){
    var r=el.getBoundingClientRect();
    var x=Math.max(-1,Math.min(1,((e.clientX-r.left)/r.width)*2-1));
    var y=Math.max(-1,Math.min(1,((e.clientY-r.top)/r.height)*2-1));
    knob.style.left=(x*53+53)+'px';knob.style.top=(y*53+53)+'px';
    cb(x,-y,true);
  }
  el.addEventListener('pointerdown',function(e){active=true;el.setPointerCapture(e.pointerId);pos(e);});
  el.addEventListener('pointermove',function(e){if(active)pos(e);});
  function up(){active=false;knob.style.left='53px';knob.style.top='53px';cb(0,0,false);}
  el.addEventListener('pointerup',up);el.addEventListener('pointercancel',up);
}
var joyLX=0,joyLY=0,joyLA=false,joyRX=0,joyRY=0,joyRA=false;
// LEFT stick (joyS): DIRECT non-ramped combined - x=steer, y=throttle
joy(document.getElementById('joyS'),document.getElementById('knobS'),
  function(x,y,a){joyLX=x;joyLY=y;joyLA=a;});
// RIGHT stick (joyT): ramped combined - x=steer, y=throttle
joy(document.getElementById('joyT'),document.getElementById('knobT'),
  function(x,y,a){joyRX=x;joyRY=y;joyRA=a;});
// ---- keyboard ----
window.addEventListener('keydown',function(e){keys[e.key.toLowerCase()]=true;
  if([' ','arrowup','arrowdown','arrowleft','arrowright'].indexOf(e.key.toLowerCase())>=0)e.preventDefault();});
window.addEventListener('keyup',function(e){keys[e.key.toLowerCase()]=false;});
function kbActive(){return keys['w']||keys['s']||keys['a']||keys['d']||
  keys['arrowup']||keys['arrowdown']||keys['arrowleft']||keys['arrowright'];}
// ---- gamepad ----
var gpAttached=false,gpSteer=0,gpThr=0;
window.addEventListener('gamepadconnected',function(){gpAttached=true;});
window.addEventListener('gamepaddisconnected',function(){gpAttached=false;});
function pollGamepad(){
  var gps=navigator.getGamepads?navigator.getGamepads():[];
  var gp=null;for(var i=0;i<gps.length;i++)if(gps[i]){gp=gps[i];break;}
  gpAttached=!!gp;
  var el=document.getElementById('gp');
  el.textContent=gpAttached?('gamepad: '+gp.id.substring(0,20)):'no gamepad';
  el.className='badge '+(gpAttached?'ok':'');
  if(gp){
    gpSteer=Math.abs(gp.axes[0])>0.08?gp.axes[0]:0;
    var rt=gp.buttons[7]?gp.buttons[7].value:0;
    var lt=gp.buttons[6]?gp.buttons[6].value:0;
    gpThr=rt-lt;
    if(Math.abs(gpThr)<0.02&&gp.axes.length>3)
      gpThr=Math.abs(gp.axes[3])>0.08?-gp.axes[3]:0;
  }
  requestAnimationFrame(pollGamepad);
}
requestAnimationFrame(pollGamepad);
// ---- control send loop 25Hz ----
setInterval(function(){
  var s=0,t=0,act=false;
  var direct=0;
  if(joyLA){s=joyLX;t=joyLY;direct=1;act=true;}        // LEFT = DIRECT
  else if(joyRA){s=joyRX;t=joyRY;act=true;}             // RIGHT = ramped
  if(kbActive()){
    s=(keys['d']||keys['arrowright']?1:0)-(keys['a']||keys['arrowleft']?1:0);
    t=(keys['w']||keys['arrowup']?1:0)-(keys['s']||keys['arrowdown']?1:0);
    act=true;}
  if(gpAttached){if(!act){s=gpSteer;t=gpThr;}act=true;}
  if(act){send({steer:s,throttle:t,direct:direct});inputActive=true;}
  else if(inputActive){send({steer:0,throttle:0,direct:0});inputActive=false;}
},40);
// ---- settings ----
function cfg(){
  var cls=Array.prototype.filter.call(document.getElementById('classes').options,
    function(o){return o.selected;}).map(function(o){return o.value;});
  send({config:{
    max_speed:Math.max(0,Math.min(500,+document.getElementById('maxSpeedUs').value))/500,
    auto_cruise_us:Math.max(0,Math.min(500,+document.getElementById('cruiseUs').value)),
    auto_block_thr:Math.max(0,Math.min(1,+document.getElementById('blockThr').value)),
    auto_hyst:Math.max(0,Math.min(1,+document.getElementById('hyst').value)),
    anti_tip_enable:document.getElementById('antiTip').checked,
    tip_roll_deg:+document.getElementById('tipRoll').value,
    tip_pitch_deg:+document.getElementById('tipPitch').value,
    yolo_enable:document.getElementById('yoloEn').checked,
    yolo_conf:document.getElementById('yoloConf').value/100,
    yolo_classes:cls,
    collision_stop:document.getElementById('colStop').checked,
    collision_area_threshold:document.getElementById('colThr').value/100}});
  document.getElementById('yoloConfV').textContent=(document.getElementById('yoloConf').value/100).toFixed(2);
  document.getElementById('colThrV').textContent=(document.getElementById('colThr').value/100).toFixed(2);
}
// numeric fields: 'change' only (typing partial values must not send
// transient configs, e.g. "0." -> block_thr 0); sliders stay live on 'input'
['maxSpeedUs','cruiseUs','blockThr','hyst','antiTip','tipRoll','tipPitch','yoloEn',
 'classes','colStop']
 .forEach(function(id){document.getElementById(id).addEventListener('change',cfg);});
['yoloConf','colThr']
 .forEach(function(id){document.getElementById(id).addEventListener('change',cfg);
   document.getElementById(id).addEventListener('input',cfg);});
// ---- map ----
function drawMap(){
  fetch('/map.json').then(function(r){return r.json();}).then(function(d){
    var c=document.getElementById('map'),ctx=c.getContext('2d');
    ctx.fillStyle='#000';ctx.fillRect(0,0,c.width,c.height);
    var pts=d.crumbs||[];if(!pts.length){return;}
    var xs=pts.map(function(p){return p.x;}),ys=pts.map(function(p){return p.y;});
    var minx=Math.min.apply(0,xs)-1,maxx=Math.max.apply(0,xs)+1;
    var miny=Math.min.apply(0,ys)-1,maxy=Math.max.apply(0,ys)+1;
    var sc=Math.min(c.width/(maxx-minx),c.height/(maxy-miny));
    function X(x){return (x-minx)*sc;} function Y(y){return c.height-(y-miny)*sc;}
    ctx.strokeStyle='#0a84ff';ctx.beginPath();
    pts.forEach(function(p,i){i?ctx.lineTo(X(p.x),Y(p.y)):ctx.moveTo(X(p.x),Y(p.y));});
    ctx.stroke();
    var last=pts[pts.length-1];
    ctx.fillStyle='#5f5';ctx.beginPath();ctx.arc(X(last.x),Y(last.y),5,0,7);ctx.fill();
    ctx.fillStyle='#f55';
    (d.dets||[]).forEach(function(p){ctx.fillRect(X(p.x)-3,Y(p.y)-3,6,6);});
  }).catch(function(){});
}
setInterval(drawMap,2000);drawMap();
</script></body></html>
"""
