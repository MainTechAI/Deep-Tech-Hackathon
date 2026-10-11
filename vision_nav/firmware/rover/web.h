#pragma once

// Wi-Fi + HTTP API. All endpoints answer with status JSON:
//   {"m":"MANUAL","d":34.5,"b":0,"l":0,"r":0,"net":"sta","ip":"172.20.10.5",
//    "rssi":-55,"up":12345,"sc":20,"wc":45}
//   m = mode, d = front distance cm (999 = clear), b = forward blocked, l/r = wheel output %,
//   net = "sta" (joined a network) or "ap" (own access point), rssi = signal dBm (sta only),
//   up = uptime ms, sc/wc = stop / warning distance cm, fs = follow state (FOLLOW mode)
//
// GET /                      joystick page
// GET /status                status only
// GET /m?x=..&y=..           joystick -100..100, stops after WEB_TIMEOUT_MS without updates
// GET /drive?l=..&r=..&ms=.. wheels -100..100 for ms (default WEB_TIMEOUT_MS, max 10000)
// GET /stop                  stop + MANUAL mode
// GET /mode?m=auto|follow|manual  switch mode (/mode without m toggles AUTO/MANUAL)
// GET /beep?ms=..&f=..       beep (default 150 ms, 2000 Hz)
//
// Drive commands are ignored in AUTO and FOLLOW modes. Front safety always applies.

void webBegin();
bool wifiIsSta();   // joined a network (needed for FOLLOW)
int  wifiRssi();    // fresh signal strength from the network's access point, dBm
int  wifiLastRssi(); // last measured value, no Wi-Fi chip round-trip
void webPoll();   // also reconnects if the network drops (robot stops meanwhile)
