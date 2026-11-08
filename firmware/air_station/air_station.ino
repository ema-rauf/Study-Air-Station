/*
  Study Air Station firmware
  ESP32 DevKit V1 + SCD40 + PMS5003 + ST7789 TFT + microSD

  Libraries (Library Manager): Adafruit GFX Library, Adafruit ST7789 Library.
  Board package: esp32 by Espressif (Arduino-ESP32 2.x or 3.x), board "ESP32 Dev Module".
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ---------------------------------------------------------------- settings
// Fill these in before flashing.
const char *WIFI_SSID = "your-wifi-name";
const char *WIFI_PASS = "your-wifi-password";
const char *DASH_USER = "admin";
const char *DASH_PASS = "change-me";       // the dashboard stays off until this is changed
const char *TG_TOKEN  = "";                // Telegram bot token, empty disables alerts
const char *TG_CHAT   = "";                // Telegram chat id
const char *TZ_INFO   = "PKT-5";           // POSIX time zone string

const float SCD_TEMP_OFFSET_C = 4.0;       // SCD40 default; lower it if the case runs hot
const uint16_t SCD_ALTITUDE_M = 0;         // metres above sea level

// Default thresholds, changeable from the dashboard and kept in flash
const int DEF_CO2_WARN  = 1000;            // ppm
const int DEF_CO2_ALERT = 1500;            // ppm
const int DEF_PM_WARN   = 25;              // ug/m3 PM2.5
const int DEF_PM_ALERT  = 55;              // ug/m3 PM2.5
const int DEF_FAN_CO2   = 1000;            // fan on at or above this CO2
const int DEF_FAN_PM    = 25;              // fan on at or above this PM2.5
const int FAN_CO2_OFF_DELTA = 200;         // fan off when CO2 is this far below the on point
const int FAN_PM_OFF_DELTA  = 10;

const unsigned long SILENCE_MS      = 10UL * 60UL * 1000UL;   // silence ends by itself
const unsigned long FAN_MIN_DWELL   = 60UL * 1000UL;          // minimum on or off time
const unsigned long LOG_PERIOD_MS   = 60UL * 1000UL;
const unsigned long HIST_PERIOD_MS  = 300UL * 1000UL;         // 288 points = 24 h
const unsigned long TG_REPEAT_MS    = 30UL * 60UL * 1000UL;   // repeat while still in alert

// ---------------------------------------------------------------- pins
const int PIN_SDA = 21, PIN_SCL = 22;
const int PIN_PMS_RX = 16;                 // ESP32 RX2, wired to PMS5003 TX
const int PIN_PMS_TX = 17;                 // ESP32 TX2, wired to PMS5003 RX
const int PIN_SCK = 33, PIN_MOSI = 25, PIN_MISO = 34;
const int PIN_TFT_CS = 14, PIN_TFT_DC = 27, PIN_TFT_RST = 26;
const int PIN_SD_CS = 32;
const int PIN_FAN = 23, PIN_BUZZER = 4;
const int PIN_LED_R = 19, PIN_LED_G = 18, PIN_LED_B = 5;
const int PIN_BTN_SILENCE = 36, PIN_BTN_PAGE = 39;

// ---------------------------------------------------------------- state
SPIClass spiBus(VSPI);
Adafruit_ST7789 tft(&spiBus, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
WebServer server(80);
Preferences prefs;

struct Config {
  int co2Warn, co2Alert, pmWarn, pmAlert, fanCo2, fanPm;
  int fanMode;                             // 0 auto, 1 on, 2 off
} cfg;

struct Reading {
  int co2 = 0;
  float temp = 0, hum = 0;
  int pm1 = 0, pm25 = 0, pm10 = 0;
  bool co2Ok = false, pmOk = false;
  unsigned long co2At = 0, pmAt = 0;
} rd;

enum Level { LV_OK = 0, LV_WARN = 1, LV_ALERT = 2 };
Level co2Level = LV_OK, pmLevel = LV_OK, level = LV_OK;

struct Button {
  int pin;
  bool down;
  bool longDone;
  unsigned long since;
  Button(int p) : pin(p), down(false), longDone(false), since(0) {}
};
Button btnSilence(PIN_BTN_SILENCE), btnPage(PIN_BTN_PAGE);

bool fanOn = false;
unsigned long fanChangedAt = 0;
unsigned long silencedAt = 0;
bool silenced = false;
bool sdOk = false;
bool serverOn = false;
bool timeSet = false;
bool tgSentAlert = false;
unsigned long lastBeepAt = 0;
int beepsLeft = 0;
unsigned long beepToggleAt = 0;
bool beepState = false;

const int HIST_N = 288;
uint16_t hist[HIST_N];
int histCount = 0, histHead = 0;
unsigned long histSum = 0, histSamples = 0;

int page = 0;
bool pageDirty = true;
String toast;
unsigned long toastUntil = 0;

// ---------------------------------------------------------------- SCD40
uint8_t crc8(const uint8_t *d, int n) {
  uint8_t c = 0xFF;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1);
  }
  return c;
}

bool scdCmd(uint16_t cmd) {
  Wire.beginTransmission(0x62);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  return Wire.endTransmission() == 0;
}

bool scdCmdArg(uint16_t cmd, uint16_t arg) {
  uint8_t a[2] = {(uint8_t)(arg >> 8), (uint8_t)(arg & 0xFF)};
  Wire.beginTransmission(0x62);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.write(a, 2);
  Wire.write(crc8(a, 2));
  return Wire.endTransmission() == 0;
}

bool scdRead(uint8_t *buf, int n) {
  if (Wire.requestFrom(0x62, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  for (int i = 0; i < n; i += 3)
    if (crc8(buf + i, 2) != buf[i + 2]) return false;
  return true;
}

void scdBegin() {
  scdCmd(0x3F86);                          // stop periodic measurement
  delay(500);
  scdCmdArg(0x241D, (uint16_t)(SCD_TEMP_OFFSET_C * 65535.0 / 175.0));
  scdCmdArg(0x2427, SCD_ALTITUDE_M);
  scdCmd(0x21B1);                          // start periodic measurement
}

void scdPoll() {
  static unsigned long last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  uint8_t b[9];
  if (!scdCmd(0xE4B8)) return;             // get data ready status
  delay(2);
  if (!scdRead(b, 3)) return;
  if ((((uint16_t)b[0] << 8 | b[1]) & 0x07FF) == 0) return;
  if (!scdCmd(0xEC05)) return;             // read measurement
  delay(2);
  if (!scdRead(b, 9)) return;
  uint16_t co2 = (uint16_t)b[0] << 8 | b[1];
  uint16_t t = (uint16_t)b[3] << 8 | b[4];
  uint16_t h = (uint16_t)b[6] << 8 | b[7];
  if (co2 == 0) return;
  rd.co2 = co2;
  rd.temp = -45.0 + 175.0 * t / 65535.0;
  rd.hum = 100.0 * h / 65535.0;
  rd.co2Ok = true;
  rd.co2At = millis();
}

// ---------------------------------------------------------------- PMS5003
void pmsPoll() {
  static uint8_t buf[32];
  static int n = 0;
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    if (n == 0 && c != 0x42) continue;
    if (n == 1 && c != 0x4D) {            // resync, the stray byte may itself start a frame
      n = 0;
      if (c == 0x42) buf[n++] = c;
      continue;
    }
    buf[n++] = c;
    if (n == 32) {
      n = 0;
      uint16_t sum = 0;
      for (int i = 0; i < 30; i++) sum += buf[i];
      if (sum != ((uint16_t)buf[30] << 8 | buf[31])) continue;
      rd.pm1 = buf[10] << 8 | buf[11];     // atmospheric environment values
      rd.pm25 = buf[12] << 8 | buf[13];
      rd.pm10 = buf[14] << 8 | buf[15];
      rd.pmOk = true;
      rd.pmAt = millis();
    }
  }
  if (rd.pmOk && millis() - rd.pmAt > 10000) rd.pmOk = false;
}

// ---------------------------------------------------------------- config
void loadConfig() {
  prefs.begin("air", false);
  cfg.co2Warn = prefs.getInt("co2w", DEF_CO2_WARN);
  cfg.co2Alert = prefs.getInt("co2a", DEF_CO2_ALERT);
  cfg.pmWarn = prefs.getInt("pmw", DEF_PM_WARN);
  cfg.pmAlert = prefs.getInt("pma", DEF_PM_ALERT);
  cfg.fanCo2 = prefs.getInt("fanc", DEF_FAN_CO2);
  cfg.fanPm = prefs.getInt("fanp", DEF_FAN_PM);
  cfg.fanMode = prefs.getInt("fanm", 0);
}

void saveConfig() {
  prefs.putInt("co2w", cfg.co2Warn);
  prefs.putInt("co2a", cfg.co2Alert);
  prefs.putInt("pmw", cfg.pmWarn);
  prefs.putInt("pma", cfg.pmAlert);
  prefs.putInt("fanc", cfg.fanCo2);
  prefs.putInt("fanp", cfg.fanPm);
  prefs.putInt("fanm", cfg.fanMode);
}

// ---------------------------------------------------------------- logic
// A level is entered at its threshold and left only 10 percent below it.
Level classify(int v, int warn, int alert, Level prev) {
  int w = warn, a = alert;
  if (prev >= LV_WARN) w = warn * 9 / 10;
  if (prev >= LV_ALERT) a = alert * 9 / 10;
  if (v >= a) return LV_ALERT;
  if (v >= w) return LV_WARN;
  return LV_OK;
}

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

int score() {
  int s = 100;
  if (rd.co2Ok) s = min(s, clampi(100 - (rd.co2 - 600) * 100 / 1400, 0, 100));
  if (rd.pmOk) s = min(s, clampi(100 - rd.pm25 * 100 / 75, 0, 100));
  return s;
}

uint16_t levelColor(Level l) {
  return l == LV_OK ? ST77XX_GREEN : (l == LV_WARN ? ST77XX_YELLOW : ST77XX_RED);
}

void updateLevels() {
  if (rd.co2Ok) co2Level = classify(rd.co2, cfg.co2Warn, cfg.co2Alert, co2Level);
  if (rd.pmOk) pmLevel = classify(rd.pm25, cfg.pmWarn, cfg.pmAlert, pmLevel);
  level = co2Level > pmLevel ? co2Level : pmLevel;
}

void updateFan() {
  bool want = fanOn;
  if (cfg.fanMode == 1) want = true;
  else if (cfg.fanMode == 2) want = false;
  else {
    bool high = (rd.co2Ok && rd.co2 >= cfg.fanCo2) || (rd.pmOk && rd.pm25 >= cfg.fanPm);
    bool low = (!rd.co2Ok || rd.co2 < cfg.fanCo2 - FAN_CO2_OFF_DELTA) &&
               (!rd.pmOk || rd.pm25 < cfg.fanPm - FAN_PM_OFF_DELTA);
    if (!fanOn && high) want = true;
    if (fanOn && low) want = false;
  }
  if (want != fanOn && (cfg.fanMode != 0 || millis() - fanChangedAt >= FAN_MIN_DWELL || fanChangedAt == 0)) {
    fanOn = want;
    fanChangedAt = millis();
    digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
  }
}

void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r);
  digitalWrite(PIN_LED_G, g);
  digitalWrite(PIN_LED_B, b);
}

void updateLed() {
  if (!rd.co2Ok && !rd.pmOk) { setLed(0, 0, 1); return; }    // blue while warming up
  if (level == LV_ALERT) { setLed((millis() / 400) % 2, 0, 0); return; }
  if (level == LV_WARN) { setLed(1, 1, 0); return; }
  setLed(0, 1, 0);
}

void startBeeps(int n) { beepsLeft = n * 2; beepToggleAt = millis(); beepState = false; }

void updateBuzzer() {
  unsigned long now = millis();
  if (silenced && now - silencedAt >= SILENCE_MS) {
    silenced = false;
    toast = "Silence ended";
    toastUntil = now + 3000;
  }
  if (level == LV_ALERT && !silenced && beepsLeft == 0 && (lastBeepAt == 0 || now - lastBeepAt > 30000)) {
    startBeeps(3);
    lastBeepAt = now;
  }
  if (level != LV_ALERT) lastBeepAt = 0;
  if (beepsLeft > 0 && now >= beepToggleAt) {
    beepState = !beepState;
    digitalWrite(PIN_BUZZER, beepState);
    beepsLeft--;
    beepToggleAt = now + 150;
  }
  if (beepsLeft == 0 && beepState) { beepState = false; digitalWrite(PIN_BUZZER, LOW); }
}

void setSilence(bool on) {
  silenced = on;
  silencedAt = millis();
  if (on) { beepsLeft = 0; digitalWrite(PIN_BUZZER, LOW); beepState = false; }
  toast = on ? "Silenced for 10 min" : "Silence cancelled";
  toastUntil = millis() + 3000;
}

void cycleFanMode() {
  cfg.fanMode = (cfg.fanMode + 1) % 3;
  saveConfig();
  const char *names[] = {"Fan: auto", "Fan: on", "Fan: off"};
  toast = names[cfg.fanMode];
  toastUntil = millis() + 3000;
}

// ---------------------------------------------------------------- time, history, log
bool nowLocal(struct tm &t) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;
  localtime_r(&now, &t);
  return true;
}

void timeString(char *out, size_t n, bool withDate) {
  struct tm t;
  if (!nowLocal(t)) { snprintf(out, n, "--:--"); return; }
  strftime(out, n, withDate ? "%Y-%m-%d %H:%M:%S" : "%H:%M", &t);
}

void historyTick() {
  static unsigned long last = 0;
  if (rd.co2Ok) { histSum += rd.co2; histSamples++; }
  if (millis() - last < HIST_PERIOD_MS) return;
  last = millis();
  if (histSamples == 0) return;
  hist[histHead] = histSum / histSamples;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
  histSum = 0;
  histSamples = 0;
}

uint16_t histAt(int i) {                   // 0 is the oldest point
  int start = (histHead - histCount + HIST_N) % HIST_N;
  return hist[(start + i) % HIST_N];
}

String logPath() {
  struct tm t;
  if (!nowLocal(t)) return "/log_boot.csv";
  char p[24];
  strftime(p, sizeof(p), "/log_%Y%m%d.csv", &t);
  return String(p);
}

void sdMount() {
  sdOk = SD.begin(PIN_SD_CS, spiBus, 4000000);
}

void logTick() {
  static unsigned long last = 0;
  if (millis() - last < LOG_PERIOD_MS) return;
  last = millis();
  if (!sdOk) sdMount();
  if (!sdOk || (!rd.co2Ok && !rd.pmOk)) return;
  String path = logPath();
  bool fresh = !SD.exists(path);
  File f = SD.open(path, FILE_APPEND);
  if (!f) { sdOk = false; return; }
  if (fresh) f.println("time,co2_ppm,temp_c,rh_pct,pm1,pm25,pm10,fan");
  char ts[24];
  struct tm tm;
  if (nowLocal(tm)) strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
  else snprintf(ts, sizeof(ts), "t+%lus", millis() / 1000);
  f.printf("%s,%d,%.1f,%.1f,%d,%d,%d,%d\n", ts, rd.co2, rd.temp, rd.hum, rd.pm1, rd.pm25, rd.pm10, fanOn ? 1 : 0);
  f.close();
}

// ---------------------------------------------------------------- Telegram
bool telegramSend(const String &text) {
  if (!strlen(TG_TOKEN) || !strlen(TG_CHAT) || WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();                    // no certificate pinning, see README
  client.setTimeout(6);
  HTTPClient http;
  http.setTimeout(6000);
  String url = String("https://api.telegram.org/bot") + TG_TOKEN + "/sendMessage";
  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String body = String("chat_id=") + TG_CHAT + "&text=" + text;
  body.replace(" ", "%20");
  body.replace("\n", "%0A");
  int code = http.POST(body);
  http.end();
  return code == 200;
}

// The first alert goes out at once. While the alert lasts it repeats every 30 minutes,
// and a failed send is retried after one minute.
void alertTick() {
  static unsigned long nextTry = 0;
  unsigned long now = millis();
  if (level == LV_ALERT) {
    if (nextTry != 0 && now < nextTry) return;
    String msg = String("Air quality alert\nCO2 ") + rd.co2 + " ppm, PM2.5 " + rd.pm25 + " ug/m3.\nOpen a window.";
    if (telegramSend(msg)) { tgSentAlert = true; nextTry = now + TG_REPEAT_MS; }
    else nextTry = now + 60000UL;
  } else if (level == LV_OK) {
    if (tgSentAlert) {
      telegramSend(String("Air quality back to normal.\nCO2 ") + rd.co2 + " ppm, PM2.5 " + rd.pm25 + " ug/m3.");
      tgSentAlert = false;
    }
    nextTry = 0;
  }
}

// ---------------------------------------------------------------- web
const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Study Air Station</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#12161c;color:#e8ecf1}
main{max-width:720px;margin:auto;padding:16px}
h1{font-size:20px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.card{background:#1c222b;border-radius:8px;padding:12px}
.card b{display:block;font-size:26px}
.card span{font-size:12px;color:#8b98a8}
#score{font-size:40px}
canvas{width:100%;height:160px;background:#1c222b;border-radius:8px;margin-top:10px}
button,input{font:inherit;padding:6px 10px;border-radius:6px;border:1px solid #394454;background:#232b36;color:inherit}
form{display:grid;grid-template-columns:1fr 90px;gap:6px;margin-top:6px}
a{color:#7fb4ff}
</style></head><body><main>
<h1>Study Air Station <small id="t"></small></h1>
<div class="grid">
<div class="card"><span>Air score</span><b id="score">-</b></div>
<div class="card"><span>CO2 ppm</span><b id="co2">-</b></div>
<div class="card"><span>PM2.5 ug/m3</span><b id="pm25">-</b></div>
<div class="card"><span>Temperature C</span><b id="temp">-</b></div>
<div class="card"><span>Humidity %</span><b id="hum">-</b></div>
<div class="card"><span>PM1.0 / PM10</span><b id="pm">-</b></div>
</div>
<p>Fan <b id="fan"></b> <button onclick="post('/api/fan?mode=0')">Auto</button>
<button onclick="post('/api/fan?mode=1')">On</button><button onclick="post('/api/fan?mode=2')">Off</button>
&nbsp; <button onclick="post('/api/silence?on=1')">Silence 10 min</button>
<button onclick="post('/api/silence?on=0')">Unsilence</button></p>
<canvas id="c" width="720" height="160"></canvas>
<p><a href="/log.csv">Download today's log</a></p>
<h2 style="font-size:16px">Thresholds</h2>
<form id="cf" onsubmit="return save(event)">
<label>CO2 warn (ppm)</label><input name="co2w" type="number">
<label>CO2 alert (ppm)</label><input name="co2a" type="number">
<label>PM2.5 warn</label><input name="pmw" type="number">
<label>PM2.5 alert</label><input name="pma" type="number">
<label>Fan on at CO2</label><input name="fanc" type="number">
<label>Fan on at PM2.5</label><input name="fanp" type="number">
<button>Save</button></form>
</main><script>
const $=i=>document.getElementById(i);
function post(u){fetch(u,{method:'POST'}).then(load)}
function save(e){e.preventDefault();fetch('/api/config?'+new URLSearchParams(new FormData($('cf'))),{method:'POST'}).then(load);return false}
let filled=false;
function load(){fetch('/api/status').then(r=>r.json()).then(d=>{
$('t').textContent=d.time;$('score').textContent=d.score;$('co2').textContent=d.co2_ok?d.co2:'-';
$('pm25').textContent=d.pm_ok?d.pm25:'-';$('temp').textContent=d.co2_ok?d.temp.toFixed(1):'-';
$('hum').textContent=d.co2_ok?Math.round(d.hum):'-';$('pm').textContent=d.pm_ok?d.pm1+' / '+d.pm10:'-';
$('fan').textContent=(d.fan?'running':'stopped')+' ('+['auto','forced on','forced off'][d.fan_mode]+')';
if(!filled){filled=true;for(const k in d.cfg)document.querySelector('[name='+k+']').value=d.cfg[k]}});
fetch('/api/history').then(r=>r.json()).then(h=>{const c=$('c'),x=c.getContext('2d');
x.clearRect(0,0,c.width,c.height);if(h.length<2)return;const mx=Math.max(2000,...h),mn=400;
x.strokeStyle='#7fb4ff';x.lineWidth=2;x.beginPath();
h.forEach((v,i)=>{const px=i*(c.width-1)/(h.length-1),py=c.height-(v-mn)/(mx-mn)*(c.height-10)-5;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
x.fillStyle='#8b98a8';x.fillText('CO2 last 24 h, '+mn+' to '+mx+' ppm',8,14)});}
load();setInterval(load,5000);
</script></body></html>)HTML";

bool auth() {
  if (server.authenticate(DASH_USER, DASH_PASS)) return true;
  server.requestAuthentication();
  return false;
}

void handleStatus() {
  if (!auth()) return;
  char ts[24];
  timeString(ts, sizeof(ts), true);
  String j = "{";
  j += "\"time\":\"" + String(ts) + "\",";
  j += "\"score\":" + String(score()) + ",";
  j += "\"co2_ok\":" + String(rd.co2Ok ? "true" : "false") + ",\"co2\":" + String(rd.co2) + ",";
  j += "\"temp\":" + String(rd.temp, 1) + ",\"hum\":" + String(rd.hum, 1) + ",";
  j += "\"pm_ok\":" + String(rd.pmOk ? "true" : "false") + ",\"pm1\":" + String(rd.pm1) + ",\"pm25\":" + String(rd.pm25) + ",\"pm10\":" + String(rd.pm10) + ",";
  j += "\"fan\":" + String(fanOn ? "true" : "false") + ",\"fan_mode\":" + String(cfg.fanMode) + ",";
  j += "\"silenced\":" + String(silenced ? "true" : "false") + ",";
  j += "\"cfg\":{\"co2w\":" + String(cfg.co2Warn) + ",\"co2a\":" + String(cfg.co2Alert) + ",\"pmw\":" + String(cfg.pmWarn) +
       ",\"pma\":" + String(cfg.pmAlert) + ",\"fanc\":" + String(cfg.fanCo2) + ",\"fanp\":" + String(cfg.fanPm) + "}}";
  server.send(200, "application/json", j);
}

void handleHistory() {
  if (!auth()) return;
  String j = "[";
  for (int i = 0; i < histCount; i++) { if (i) j += ","; j += String(histAt(i)); }
  j += "]";
  server.send(200, "application/json", j);
}

void handleFan() {
  if (!auth()) return;
  int m = server.arg("mode").toInt();
  if (m < 0 || m > 2) { server.send(400, "text/plain", "bad mode"); return; }
  cfg.fanMode = m;
  saveConfig();
  server.send(200, "text/plain", "ok");
}

void handleSilence() {
  if (!auth()) return;
  setSilence(server.arg("on") == "1");
  server.send(200, "text/plain", "ok");
}

void handleConfig() {
  if (!auth()) return;
  struct { const char *key; int *dst; int lo, hi; } f[] = {
    {"co2w", &cfg.co2Warn, 400, 5000}, {"co2a", &cfg.co2Alert, 400, 5000},
    {"pmw", &cfg.pmWarn, 1, 500}, {"pma", &cfg.pmAlert, 1, 500},
    {"fanc", &cfg.fanCo2, 400, 5000}, {"fanp", &cfg.fanPm, 1, 500}};
  for (auto &e : f)
    if (server.hasArg(e.key)) *e.dst = clampi(server.arg(e.key).toInt(), e.lo, e.hi);
  if (cfg.co2Alert <= cfg.co2Warn) cfg.co2Alert = cfg.co2Warn + 100;
  if (cfg.pmAlert <= cfg.pmWarn) cfg.pmAlert = cfg.pmWarn + 5;
  saveConfig();
  server.send(200, "text/plain", "ok");
}

void handleLog() {
  if (!auth()) return;
  if (!sdOk) { server.send(503, "text/plain", "no SD card"); return; }
  File f = SD.open(logPath(), FILE_READ);
  if (!f) { server.send(404, "text/plain", "no log yet"); return; }
  server.streamFile(f, "text/csv");
  f.close();
}

void handleRoot() {
  if (!auth()) return;
  server.send_P(200, "text/html", PAGE_HTML);
}

void startServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.on("/api/fan", HTTP_POST, handleFan);
  server.on("/api/silence", HTTP_POST, handleSilence);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/log.csv", HTTP_GET, handleLog);
  server.begin();
  serverOn = true;
}

void wifiTick() {
  static unsigned long lastTry = 0;
  static bool started = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (!timeSet) {
      configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
      timeSet = true;
    }
    if (!serverOn && strcmp(DASH_PASS, "change-me") != 0) startServer();
    return;
  }
  if (!started || millis() - lastTry > 20000) {
    started = true;
    lastTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ---------------------------------------------------------------- display
void field(int x, int y, int w, int h, const String &s, uint8_t size, uint16_t fg) {
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(fg, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.print(s);
}

void drawFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6, 6);
  if (page == 0) tft.print("Air quality");
  if (page == 1) tft.print("CO2, last 24 h");
  if (page == 2) tft.print("System");
  tft.drawFastHLine(0, 28, 320, 0x4208);
  if (page == 0) {
    tft.setTextSize(1);
    tft.setTextColor(0x8410, ST77XX_BLACK);
    tft.setCursor(8, 40);  tft.print("CO2 ppm");
    tft.setCursor(170, 40); tft.print("PM2.5 ug/m3");
    tft.setCursor(8, 140); tft.print("Temp C");
    tft.setCursor(90, 140); tft.print("Humidity %");
    tft.setCursor(190, 140); tft.print("PM1.0 / PM10");
    tft.setCursor(8, 196); tft.print("Air score");
  }
  pageDirty = false;
}

void drawChart() {
  const int x0 = 30, y0 = 40, w = 280, h = 160;
  tft.fillRect(0, 32, 320, 190, ST77XX_BLACK);
  tft.drawRect(x0, y0, w, h, 0x4208);
  if (histCount < 2) {
    field(60, 110, 220, 16, "Collecting data...", 2, 0x8410);
    return;
  }
  int mx = 2000;
  for (int i = 0; i < histCount; i++) if (histAt(i) > mx) mx = histAt(i);
  int mn = 400;
  int cw = cfg.co2Warn;
  if (cw > mn && cw < mx) tft.drawFastHLine(x0, y0 + h - (cw - mn) * h / (mx - mn), w, ST77XX_YELLOW);
  int px = 0, py = 0;
  for (int i = 0; i < histCount; i++) {
    int x = x0 + 1 + (i + HIST_N - histCount) * (w - 2) / (HIST_N - 1);
    int v = clampi(histAt(i), mn, mx);
    int y = y0 + h - 1 - (v - mn) * (h - 2) / (mx - mn);
    if (i) tft.drawLine(px, py, x, y, ST77XX_CYAN);
    px = x; py = y;
  }
  tft.setTextSize(1);
  tft.setTextColor(0x8410, ST77XX_BLACK);
  tft.setCursor(0, y0);          tft.print(mx);
  tft.setCursor(0, y0 + h - 8);  tft.print(mn);
  tft.setCursor(x0, y0 + h + 6); tft.print("-24 h");
  tft.setCursor(x0 + w - 18, y0 + h + 6); tft.print("now");
}

void drawPage0() {
  char b[24];
  if (rd.co2Ok) snprintf(b, sizeof(b), "%d", rd.co2); else snprintf(b, sizeof(b), "--");
  field(8, 54, 150, 40, b, 5, levelColor(co2Level));
  if (rd.pmOk) snprintf(b, sizeof(b), "%d", rd.pm25); else snprintf(b, sizeof(b), "--");
  field(170, 54, 150, 40, b, 5, levelColor(pmLevel));
  if (rd.co2Ok) snprintf(b, sizeof(b), "%.1f", rd.temp); else snprintf(b, sizeof(b), "--");
  field(8, 154, 70, 16, b, 2, ST77XX_WHITE);
  if (rd.co2Ok) snprintf(b, sizeof(b), "%d", (int)(rd.hum + 0.5)); else snprintf(b, sizeof(b), "--");
  field(90, 154, 80, 16, b, 2, ST77XX_WHITE);
  if (rd.pmOk) snprintf(b, sizeof(b), "%d/%d", rd.pm1, rd.pm10); else snprintf(b, sizeof(b), "--");
  field(190, 154, 130, 16, b, 2, ST77XX_WHITE);
  int s = score();
  tft.fillRect(8, 208, 304, 10, 0x2104);
  tft.fillRect(8, 208, 304 * s / 100, 10, levelColor(level));
  snprintf(b, sizeof(b), "%d", s);
  field(80, 192, 60, 16, b, 2, ST77XX_WHITE);
  const char *msg = level == LV_ALERT ? "Ventilate now" : (level == LV_WARN ? "Getting stuffy" : "Air is fine");
  field(150, 192, 170, 16, msg, 2, levelColor(level));
}

void drawPage2() {
  char b[48];
  String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("offline");
  snprintf(b, sizeof(b), "IP    %s", ip.c_str());
  field(8, 40, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "WiFi  %s  %d dBm", WiFi.status() == WL_CONNECTED ? WIFI_SSID : "-", WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  field(8, 56, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "SD    %s", sdOk ? "logging" : "no card");
  field(8, 72, 310, 14, b, 1, sdOk ? ST77XX_WHITE : ST77XX_ORANGE);
  const char *fm[] = {"auto", "forced on", "forced off"};
  snprintf(b, sizeof(b), "Fan   %s, %s", fanOn ? "running" : "stopped", fm[cfg.fanMode]);
  field(8, 88, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "Web   %s", serverOn ? "dashboard on" : "off (set DASH_PASS)");
  field(8, 104, 310, 14, b, 1, serverOn ? ST77XX_WHITE : ST77XX_ORANGE);
  snprintf(b, sizeof(b), "CO2 warn/alert  %d / %d ppm", cfg.co2Warn, cfg.co2Alert);
  field(8, 128, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "PM2.5 warn/alert  %d / %d", cfg.pmWarn, cfg.pmAlert);
  field(8, 144, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "Silence  %s", silenced ? "active" : "off");
  field(8, 160, 310, 14, b, 1, ST77XX_WHITE);
  snprintf(b, sizeof(b), "Uptime  %lu min", millis() / 60000UL);
  field(8, 176, 310, 14, b, 1, ST77XX_WHITE);
}

void drawStatusBar() {
  char t[24];
  timeString(t, sizeof(t), false);
  field(250, 8, 70, 16, t, 2, ST77XX_WHITE);
  if (millis() < toastUntil) {
    field(0, 224, 320, 16, toast, 2, ST77XX_YELLOW);
  } else {
    String s = String(fanOn ? "Fan on" : "Fan off") + (silenced ? "  Silenced" : "") + (sdOk ? "  SD" : "");
    field(0, 224, 320, 16, s, 1, 0x8410);
  }
}

void displayTick() {
  static unsigned long last = 0;
  static unsigned long lastChart = 0;
  if (pageDirty) { drawFrame(); last = 0; lastChart = 0; }
  if (page == 1) {
    if (millis() - lastChart >= 30000 || lastChart == 0) { drawChart(); lastChart = millis(); }
  }
  if (millis() - last < 1000) return;
  last = millis();
  if (page == 0) drawPage0();
  if (page == 2) drawPage2();
  drawStatusBar();
}

// ---------------------------------------------------------------- buttons
// returns 1 on a short press (on release) and 2 once a long press passes 1.2 s
int buttonPoll(Button &b) {
  bool pressed = digitalRead(b.pin) == LOW;
  unsigned long now = millis();
  if (pressed && !b.down) { b.down = true; b.longDone = false; b.since = now; return 0; }
  if (pressed && b.down && !b.longDone && now - b.since > 1200) { b.longDone = true; return 2; }
  if (!pressed && b.down) {
    b.down = false;
    if (!b.longDone && now - b.since > 40) return 1;
  }
  return 0;
}

void buttonsTick() {
  int s = buttonPoll(btnSilence);
  if (s == 1) setSilence(!silenced);
  int p = buttonPoll(btnPage);
  if (p == 1) { page = (page + 1) % 3; pageDirty = true; }
  if (p == 2) cycleFanMode();
}

// ---------------------------------------------------------------- setup and loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_FAN, OUTPUT);     digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BUZZER, OUTPUT);  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BTN_SILENCE, INPUT);        // external pull-ups on the board
  pinMode(PIN_BTN_PAGE, INPUT);
  setLed(0, 0, 1);
  loadConfig();

  spiBus.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 100);
  tft.print("Study Air Station");
  tft.setTextSize(1);
  tft.setCursor(8, 124);
  tft.print("Sensors warming up");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  scdBegin();
  Serial2.begin(9600, SERIAL_8N1, PIN_PMS_RX, PIN_PMS_TX);
  sdMount();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("air-station");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  delay(1500);
  pageDirty = true;
}

void loop() {
  scdPoll();
  pmsPoll();
  updateLevels();
  updateFan();
  updateLed();
  updateBuzzer();
  historyTick();
  logTick();
  wifiTick();
  if (serverOn) server.handleClient();
  buttonsTick();
  displayTick();
  alertTick();
  delay(5);
}
