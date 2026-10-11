#include "web.h"
#include "robot.h"
#include "movement.h"
#include "buzzer.h"
#include "face.h"
#include "follow.h"
#include "arduino_secrets.h"
#include <WiFiS3.h>

static WiFiServer server(80);
static bool wifiOk = false;
static bool staMode = false;
static unsigned long lastReconnect = 0;
static char ipText[16] = "0.0.0.0";
static int rssi = 0;
static unsigned long lastRssi = 0;

static void cacheIp() {
  IPAddress ip = WiFi.localIP();
  snprintf(ipText, sizeof(ipText), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

// Phone / laptop control page. Keyboard: WASD or arrows, Space = stop.
static const char PAGE[] = R"html(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>RobotCar</title><style>
body{margin:0;padding:12px 16px;font-family:system-ui,sans-serif;background:#111;color:#eee;text-align:center;
touch-action:none;user-select:none;-webkit-user-select:none}
.top{display:flex;justify-content:space-between;align-items:center;font-size:13px;color:#999;max-width:360px;margin:0 auto}
#dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#d33;margin-right:6px;vertical-align:middle}
#dot.ok{background:#3c6}
#mode{padding:4px 10px;border-radius:8px;background:#333;color:#fff;font-weight:bold;font-size:14px}
#mode.auto{background:#36c}
#dist{font-size:56px;font-weight:bold;margin:8px 0 6px;font-variant-numeric:tabular-nums}
.warn{color:#f55}.near{color:#fb3}
#bar{height:10px;background:#222;border-radius:5px;overflow:hidden;max-width:320px;margin:0 auto}
#fill{height:100%;width:100%;background:#3c6;transition:width .15s}
#msg{height:20px;margin-top:8px;color:#f55;font-weight:bold;font-size:15px}
#pad{width:240px;height:240px;border-radius:50%;background:#1d1d1d;border:3px solid #444;margin:10px auto;position:relative}
#pad.blocked{border-color:#f55}
#knob{width:84px;height:84px;border-radius:50%;background:#4a9;position:absolute;left:78px;top:78px}
#wheels{font-size:13px;color:#999;font-variant-numeric:tabular-nums}
.row{display:flex;gap:8px;justify-content:center;max-width:360px;margin:10px auto 0}
button{flex:1;font-size:15px;padding:14px 0;border:0;border-radius:10px;color:#fff;background:#555}
#stop{background:#d33;font-weight:bold}#auto{background:#36c}#fol,#mode.follow{background:#8a3cc9}
label{display:block;font-size:13px;color:#999;margin-top:12px}
input[type=range]{width:220px;vertical-align:middle}
</style></head><body>
<div class=top><span><span id=dot></span><span id=net>connecting...</span></span><span id=mode>-</span></div>
<div id=dist>-</div><div id=bar><div id=fill></div></div>
<div id=msg></div>
<div id=pad><div id=knob></div></div>
<div id=wheels>L 0% &middot; R 0%</div>
<div class=row><button id=stop>STOP</button><button id=auto>AUTO</button><button id=fol>FOLLOW</button><button id=beep>BEEP</button></div>
<label>Max speed <span id=spdv>70</span>%<br><input id=spd type=range min=30 max=100 value=70></label>
<script>
const $=id=>document.getElementById(id);
const pad=$('pad'),knob=$('knob'),R=120,K=42,M=R-K;
let jx=0,jy=0,touch=false,sendZero=false,keys={},lastB=0,fails=0,cur='';
const FS={scan:'looking around',turn:'turning to signal',drive:'driving to you',near:'found you!',avoid:'going around obstacle'};
function setKnob(dx,dy){knob.style.left=(R-K+dx)+'px';knob.style.top=(R-K+dy)+'px';}
function move(e){const b=pad.getBoundingClientRect();let dx=e.clientX-b.left-b.width/2,dy=e.clientY-b.top-b.height/2;
const d=Math.hypot(dx,dy);if(d>M){dx*=M/d;dy*=M/d;}setKnob(dx,dy);jx=dx/M*100;jy=-dy/M*100;}
function release(){if(touch)sendZero=true;touch=false;jx=jy=0;setKnob(0,0);}
pad.onpointerdown=e=>{touch=true;pad.setPointerCapture(e.pointerId);move(e);};
pad.onpointermove=e=>{if(touch)move(e);};
pad.onpointerup=pad.onpointercancel=release;
const KM={ArrowUp:'u',KeyW:'u',ArrowDown:'d',KeyS:'d',ArrowLeft:'l',KeyA:'l',ArrowRight:'r',KeyD:'r'};
onkeydown=e=>{if(e.code==='Space'){e.preventDefault();stop();return;}const k=KM[e.code];if(k){keys[k]=1;e.preventDefault();}};
onkeyup=e=>{const k=KM[e.code];if(k){delete keys[k];if(!Object.keys(keys).length)sendZero=true;}};
onblur=()=>{if(Object.keys(keys).length)sendZero=true;keys={};release();};
$('spd').oninput=()=>$('spdv').textContent=$('spd').value;
function cmd(){let x=jx,y=jy;
 if(!touch){if(!Object.keys(keys).length)return null;
  y=((keys.u?1:0)-(keys.d?1:0))*100;x=((keys.r?1:0)-(keys.l?1:0))*(y?50:100);}
 const s=$('spd').value/100;return[Math.round(x*s),Math.round(y*s)];}
function show(s,ms){
 $('dot').className='ok';
 $('net').textContent=(s.net==='sta'?'Wi-Fi '+s.rssi+' dBm':'AP '+s.ip)+' · '+Math.round(ms)+' ms';
 cur=s.m;const m=$('mode');m.textContent=s.m;m.className=s.m.toLowerCase();
 $('auto').textContent=s.m==='AUTO'?'MANUAL':'AUTO';$('fol').textContent=s.m==='FOLLOW'?'MANUAL':'FOLLOW';
 const clear=s.d>=999,cls=clear?'':s.d<s.sc?'warn':s.d<s.wc?'near':'';
 const d=$('dist');d.textContent=clear?'clear':Math.round(s.d)+' cm';d.className=cls;
 const f=$('fill');f.style.width=(clear?100:Math.min(100,s.d))+'%';
 f.style.background=cls==='warn'?'#f55':cls==='near'?'#fb3':'#3c6';
 const g=$('msg');g.style.color=s.b?'#f55':'#c9a0f0';
 g.textContent=s.b?'\u26d4 Obstacle ahead - turn or reverse':s.m==='FOLLOW'?'Follow: '+(FS[s.fs]||s.fs)+' \u00b7 '+s.rssi+' dBm':'';
 pad.className=s.b?'blocked':'';
 if(s.b&&!lastB&&navigator.vibrate)navigator.vibrate(150);lastB=s.b;
 $('wheels').textContent='L '+s.l+'% · R '+s.r+'%';}
function offline(){$('dot').className='';$('net').textContent='no connection';}
async function req(u){const t=performance.now(),c=new AbortController(),to=setTimeout(()=>c.abort(),1500);
 try{const r=await fetch(u,{cache:'no-store',signal:c.signal});show(await r.json(),performance.now()-t);fails=0;}
 catch(e){if(++fails>1)offline();}finally{clearTimeout(to);}}
function stop(){keys={};release();sendZero=false;req('/stop');}
// The robot answers one request at a time: poll slowly when idle, not at all when hidden
async function tick(){const c=cmd();
 if(c)await req('/m?x='+c[0]+'&y='+c[1]);
 else if(sendZero){sendZero=false;await req('/m?x=0&y=0');}
 else if(!document.hidden)await req('/status');
 setTimeout(tick,c?30:document.hidden?1000:500);}
$('stop').onclick=stop;
$('auto').onclick=()=>req(cur==='AUTO'?'/mode?m=manual':'/mode?m=auto');
$('fol').onclick=()=>req(cur==='FOLLOW'?'/mode?m=manual':'/mode?m=follow');
$('beep').onclick=()=>req('/beep');
tick();
</script></body></html>)html";

static int queryInt(const String& path, const char* key, int def) {
  String k = String(key) + "=";
  int i = path.indexOf(k);
  if (i < 0) return def;
  return path.substring(i + k.length()).toInt();
}

// Every WiFiClient call is a round-trip to the Wi-Fi chip, so build each
// response in memory and send it with as few writes as possible.
static void sendAll(WiFiClient& c, const char* data, size_t len) {
  const size_t CHUNK = 1024;
  for (size_t i = 0; i < len; i += CHUNK)
    c.write((const uint8_t*)data + i, min(CHUNK, len - i));
}

static void sendStatus(WiFiClient& c) {
  char body[260];
  snprintf(body, sizeof(body),
    "{\"m\":\"%s\",\"d\":%.1f,\"b\":%d,\"l\":%d,\"r\":%d,\"net\":\"%s\",\"ip\":\"%s\","
    "\"rssi\":%d,\"up\":%lu,\"sc\":%d,\"wc\":%d,\"fs\":\"%s\"}",
    modeName(), dist, moveBlocked() ? 1 : 0, moveLeft(), moveRight(),
    staMode ? "sta" : "ap", ipText, staMode ? rssi : 0, millis(), (int)STOP_CM, (int)WARN_CM,
    mode == FOLLOW ? followState() : "");
  char resp[400];
  int n = snprintf(resp, sizeof(resp),
    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\n"
    "Cache-Control: no-store\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
    (int)strlen(body), body);
  sendAll(c, resp, n);
}

static void sendPage(WiFiClient& c) {
  char head[160];
  int n = snprintf(head, sizeof(head),
    "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    "Content-Length: %d\r\nConnection: close\r\n\r\n", (int)strlen(PAGE));
  sendAll(c, head, n);
  sendAll(c, PAGE, strlen(PAGE));
}

static void route(WiFiClient& c, const String& path) {
  if (path == "/") { sendPage(c); return; }

  if (path.startsWith("/m?")) {
    if (mode == MANUAL) moveJoystick(queryInt(path, "x", 0), queryInt(path, "y", 0), WEB_TIMEOUT_MS);
  } else if (path.startsWith("/drive")) {
    if (mode == MANUAL) {
      long ms = constrain(queryInt(path, "ms", WEB_TIMEOUT_MS), 1, 10000);
      moveDrive(queryInt(path, "l", 0), queryInt(path, "r", 0), ms);
    }
  } else if (path == "/stop") {
    setMode(MANUAL);
  } else if (path.startsWith("/mode")) {
    if (path.indexOf("m=auto") >= 0)        setMode(AUTO);
    else if (path.indexOf("m=follow") >= 0) setMode(FOLLOW);
    else if (path.indexOf("m=manual") >= 0) setMode(MANUAL);
    else                                    setMode(mode == MANUAL ? AUTO : MANUAL);
  } else if (path.startsWith("/beep")) {
    buzzerBeep(constrain(queryInt(path, "ms", 150), 10, 2000), queryInt(path, "f", 2000));
  } else if (path != "/status") {
    const char NF[] = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    sendAll(c, NF, strlen(NF));
    return;
  }
  sendStatus(c);
}

static bool joinNetwork() {
  Serial.print("Joining Wi-Fi \""); Serial.print(STA_SSID); Serial.println("\"...");
  WiFi.begin(STA_SSID, STA_PASS);
  unsigned long t0 = millis();
  while (millis() - t0 < STA_CONNECT_MS) {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) return true;
    delay(250);
  }
  WiFi.disconnect();
  return false;
}

static void startAP() {
  WiFi.config(IPAddress(192, 168, 4, 1));
  if (WiFi.beginAP(AP_SSID, AP_PASS) != WL_AP_LISTENING) {
    Serial.println("Failed to start access point");
    return;
  }
  delay(2000);
}

bool wifiIsSta() { return staMode; }

int wifiRssi() {
  if (!staMode) return 0;
  rssi = WiFi.RSSI();
  lastRssi = millis();
  return rssi;
}

void webBegin() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Wi-Fi module not found!");
    return;
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    Serial.println("Update Wi-Fi firmware: Tools -> Firmware Updater");

  staMode = strlen(STA_SSID) > 0 && joinNetwork();
  if (!staMode) {
    if (strlen(STA_SSID) > 0) Serial.println("Could not join, starting own access point");
    startAP();
  }
  server.begin();
  wifiOk = true;

  cacheIp();
  IPAddress ip = WiFi.localIP();
  if (staMode) { Serial.print("Joined \""); Serial.print(STA_SSID); Serial.print("\"");  }
  else         { Serial.print("Access point \""); Serial.print(AP_SSID); Serial.print("\""); }
  Serial.print(", open http://"); Serial.println(ip);
  showNumber(ip[3]);   // last part of the IP stays on the LED matrix until the robot moves
}

void webPoll() {
  if (!wifiOk) return;

  if (staMode && WiFi.status() != WL_CONNECTED && millis() - lastReconnect > 10000) {
    lastReconnect = millis();
    if (mode == AUTO) setMode(MANUAL);
    moveStop();   // joining blocks the loop, don't keep driving blind
    Serial.println("Wi-Fi lost, reconnecting");
    if (joinNetwork()) { server.begin(); cacheIp(); showNumber(WiFi.localIP()[3]); }
    return;
  }

  if (staMode && millis() - lastRssi > 1000) wifiRssi();

  WiFiClient c = server.available();
  if (!c) return;
  // Read the request in bulk (byte-by-byte reads are very slow on WiFiS3)
  char buf[257];
  int n = 0;
  unsigned long t0 = millis();
  while (n == 0 && millis() - t0 < 300) {
    int a = c.available();
    if (a > 0) n = c.read((uint8_t*)buf, min(a, (int)sizeof(buf) - 1));
  }
  buf[max(n, 0)] = 0;
  char* eol = strpbrk(buf, "\r\n");
  if (eol) *eol = 0;
  String line = buf;
  uint8_t junk[256];
  int a;
  while ((a = c.available()) > 0) c.read(junk, min(a, (int)sizeof(junk)));   // rest of headers

  // "GET /m?x=10&y=20 HTTP/1.1" -> "/m?x=10&y=20"
  int s = line.indexOf(' '), e = line.indexOf(' ', s + 1);
  String path = (s >= 0 && e > s) ? line.substring(s + 1, e) : "/";
  route(c, path);
  delay(1);
  c.stop();
}
