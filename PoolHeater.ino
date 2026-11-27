/*
  Pool Heater Probe Override  -  firmware v1.7.1
  Board: Waveshare ESP32-S3-Zero   (Arduino IDE: "ESP32S3 Dev Module", USB CDC On Boot: Enabled,
         Flash 4MB, PSRAM: QSPI PSRAM, Partition: Default 4MB with spiffs)

  What it does
   - Reads the pool probe (NTC thermistor) on GP1 through a divider (R1 to 3.3 V).
   - Gives the heater controller a fake probe:  K1 (GP6) = HOT resistor  -> heater satisfied (off)
                                                K2 (GP5) = COLD resistor -> heater calls for heat
     Both relays off = open sensor -> heater shows P1 and stops (fail-safe if this board dies).
   - Web page at http://poolheater.local  (or the IP shown in Serial Monitor): status, settings,
     schedule, calibration, Wi-Fi setup, event log, 24 h temperature graph.
   - Over-the-air updates from the Arduino IDE (Tools > Port > "poolheater at ...").
   - If it can't join Wi-Fi it opens its own network "PoolHeater-Setup" (password: poolheater);
     connect your phone to it and browse to http://192.168.4.1 to enter your Wi-Fi details.

  Before the first upload: copy secrets.example.h to secrets.h (same folder) and put your
  Wi-Fi name, password and update password in it. secrets.h is never uploaded to GitHub.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>
#include <esp_task_wdt.h>
#include <LittleFS.h>

// One history record (defined up here so the Arduino IDE's automatic function declarations can see it)
struct HRec { uint32_t t; float avg, mn, mx; uint8_t heat; };

// ===================== YOUR SETTINGS =====================
// Passwords live in secrets.h (kept out of GitHub by .gitignore)
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "secrets.h is missing: copy secrets.example.h to secrets.h in the PoolHeater folder and fill in your Wi-Fi details"
#endif
const char* DEFAULT_WIFI_SSID = WIFI_SSID;            // 2.4 GHz network; can be changed later on the web page
const char* DEFAULT_WIFI_PASS = WIFI_PASSWORD;
const char* OTA_PASSWORD      = OTA_UPDATE_PASSWORD;  // asked for by the Arduino IDE on network uploads
const char* HOSTNAME          = "poolheater";          // -> http://poolheater.local
const char* TIMEZONE          = "EST5EDT,M3.2.0,M11.1.0"; // US Eastern with daylight saving

// Measured hardware values
const float R1_OHMS      = 6600.0;   // divider resistor (two in series), measured
const float VCC_MV       = 3300.0;   // 3V3 rail; small errors are absorbed by calibration
// Probe curve from your 2-point calibration (ice bath 13820 ohm = 32.0 F, warm water 2890 ohm = 96.3 F).
// Used when no calibration points are stored on the board.
const float PROBE_R25    = 4444.0;   // ohms at 25 C / 77 F
const float PROBE_BETA   = 3696.0;
// =========================================================

const char* FW_VERSION = "1.7.1";

// Pins
const int PIN_PROBE = 1;    // GP1  (ADC1)
const int PIN_HOT   = 6;    // GP6 -> K1 -> HOT resistor  (heater off)
const int PIN_COLD  = 5;    // GP5 -> K2 -> COLD resistor (heater on)
const int PIN_RGB   = 21;   // onboard WS2812

WebServer server(80);
Preferences prefs;

// ---------- settings (saved in flash) ----------
enum Mode { MODE_AUTO = 0, MODE_OFF = 1, MODE_ON = 2 };
struct Settings {
  float setpointF  = 82.0;
  float diffF      = 0.5;     // Neg Differential: heat restarts at setpoint - diffF
  int   confirmMin = 2;       // Confirm time: readings must hold this long before switching
  int   minOffMin  = 5;
  bool  schedOn    = false;
  int   schedStart = 10 * 60; // minutes after midnight
  int   schedEnd   = 18 * 60;
  int   mode       = MODE_AUTO;
  float offsetF    = 0.0;     // installed offset (pool wall vs thermometer)
  int   calN       = 0;       // bench calibration points
  float calR[3]    = {0, 0, 0};
  float calF[3]    = {0, 0, 0};
  String ssid, pass;
} S;

// ---------- probe model ----------
struct Model {
  bool  sh = false;          // true: Steinhart-Hart A,B,C ; false: Beta
  double A = 0, B = 0, C = 0;
  double beta = PROBE_BETA, r25 = PROBE_R25;
  double ptOffsetF = 0;      // used for 1-point calibration
  String desc = "default curve";
} M;

// ---------- live state ----------
float  probeMv = 0, probeOhms = NAN, tempF = NAN, rawTempF = NAN;
String probeFault = "starting";
bool   heatOn = false;            // true = COLD relay on (heater asked to run)
bool   relaysStarted = false;
unsigned long lastChange = 0;
bool   everSwitched = false;
String reason = "starting";
bool   apMode = false, otaStarted = false;
unsigned long staConnectedAt = 0, bootMs = 0, lastConnectedMs = 0, weakSince = 0, forceOnSince = 0;
unsigned long atOrAboveSince = 0;  // start of the current unbroken run of readings >= setpoint (0 = none)
unsigned long atOrBelowSince = 0;  // start of the current unbroken run of readings <= setpoint - differential
bool wifiSaved = false;          // true = Wi-Fi details saved from the web page (else the code's lines)
int  wifiAttempt = 0;
const unsigned long FORCE_ON_LIMIT_MS = 2UL * 60 * 60 * 1000;   // Force on reverts to Auto after 2 hours

// ---------- log + history ----------
const int LOGN = 40;
String logBuf[LOGN]; int logHead = 0, logCount = 0;

bool timeValid() { return time(nullptr) > 1700000000; }

String nowStr() {
  if (!timeValid()) {
    unsigned long s = millis() / 1000;
    char b[24]; snprintf(b, sizeof b, "+%lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60); return b;
  }
  time_t t = time(nullptr); struct tm tmv; localtime_r(&t, &tmv);
  char b[24]; strftime(b, sizeof b, "%m-%d %H:%M:%S", &tmv); return b;
}

void logEvent(const String& msg) {
  String line = nowStr() + "  " + msg;
  Serial.println(line);
  logBuf[logHead] = line; logHead = (logHead + 1) % LOGN; if (logCount < LOGN) logCount++;
}

// ===================== settings storage =====================
void loadSettings() {
  prefs.begin("pool", true);
  S.setpointF  = prefs.getFloat("sp", S.setpointF);
  S.diffF      = prefs.getFloat("hy", S.diffF);
  S.confirmMin = prefs.getInt("cfm", S.confirmMin);
  S.minOffMin  = prefs.getInt("moff", S.minOffMin);
  S.schedOn    = prefs.getBool("son", S.schedOn);
  S.schedStart = prefs.getInt("ss", S.schedStart);
  S.schedEnd   = prefs.getInt("se", S.schedEnd);
  S.mode       = prefs.getInt("mode", S.mode);
  S.offsetF    = prefs.getFloat("off", S.offsetF);
  S.calN       = prefs.getInt("cn", 0);
  for (int i = 0; i < 3; i++) {
    S.calR[i] = prefs.getFloat(("cr" + String(i)).c_str(), 0);
    S.calF[i] = prefs.getFloat(("cf" + String(i)).c_str(), 0);
  }
  wifiSaved = prefs.isKey("ssid");
  S.ssid = wifiSaved ? prefs.getString("ssid", DEFAULT_WIFI_SSID) : String(DEFAULT_WIFI_SSID);
  S.pass = wifiSaved ? prefs.getString("pass", DEFAULT_WIFI_PASS) : String(DEFAULT_WIFI_PASS);
  prefs.end();
  if (S.calN < 0 || S.calN > 3) S.calN = 0;
}

void saveSettings() {
  prefs.begin("pool", false);
  prefs.putFloat("sp", S.setpointF);  prefs.putFloat("hy", S.diffF);
  prefs.putInt("cfm", S.confirmMin);  prefs.putInt("moff", S.minOffMin);
  prefs.putBool("son", S.schedOn);    prefs.putInt("ss", S.schedStart); prefs.putInt("se", S.schedEnd);
  prefs.putInt("mode", S.mode);       prefs.putFloat("off", S.offsetF);
  prefs.putInt("cn", S.calN);
  for (int i = 0; i < 3; i++) {
    prefs.putFloat(("cr" + String(i)).c_str(), S.calR[i]);
    prefs.putFloat(("cf" + String(i)).c_str(), S.calF[i]);
  }
  prefs.end();
}

void saveWifi() {
  prefs.begin("pool", false);
  prefs.putString("ssid", S.ssid); prefs.putString("pass", S.pass);
  prefs.end();
  wifiSaved = true;
}

void forgetWifi() {
  prefs.begin("pool", false); prefs.remove("ssid"); prefs.remove("pass"); prefs.end();
  wifiSaved = false; S.ssid = DEFAULT_WIFI_SSID; S.pass = DEFAULT_WIFI_PASS;
}

// If the Wi-Fi lines in the code were changed since the last upload, they win over anything saved on the board.
bool codeWifiChanged() {
  String sig = String(DEFAULT_WIFI_SSID) + "\n" + DEFAULT_WIFI_PASS;
  prefs.begin("pool", false);
  bool changed = prefs.getString("csig", "") != sig;
  if (changed) { prefs.remove("ssid"); prefs.remove("pass"); prefs.putString("csig", sig); }
  prefs.end();
  return changed;
}

// ===================== probe math =====================
double FtoK(double f) { return (f - 32.0) / 1.8 + 273.15; }
double KtoF(double k) { return (k - 273.15) * 1.8 + 32.0; }
double betaTempF(double ohms, double beta, double r25) {
  return KtoF(1.0 / (1.0 / 298.15 + log(ohms / r25) / beta));
}

void buildModel() {
  M = Model();
  int n = S.calN;
  if (n >= 3) {
    // Steinhart-Hart: 1/T = A + B lnR + C lnR^3  (solve 3x3 with Cramer's rule)
    double L[3], Y[3];
    for (int i = 0; i < 3; i++) { L[i] = log(S.calR[i]); Y[i] = 1.0 / FtoK(S.calF[i]); }
    double a[3][3] = {{1, L[0], L[0]*L[0]*L[0]}, {1, L[1], L[1]*L[1]*L[1]}, {1, L[2], L[2]*L[2]*L[2]}};
    auto det3 = [](double m[3][3]) {
      return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
           + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    };
    double D = det3(a);
    double fmin = min(S.calF[0], min(S.calF[1], S.calF[2])), fmax = max(S.calF[0], max(S.calF[1], S.calF[2]));
    if (fabs(D) > 1e-12 && fmax - fmin >= 15.0) {
      double coef[3];
      for (int k = 0; k < 3; k++) {
        double m[3][3];
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) m[r][c] = (c == k) ? Y[r] : a[r][c];
        coef[k] = det3(m) / D;
      }
      // sanity: curve must fall with rising resistance over 1k..40k
      bool ok = true; double prev = 1e9;
      for (double r = 1000; r <= 40000; r *= 1.25) {
        double lr = log(r); double t = 1.0 / (coef[0] + coef[1]*lr + coef[2]*lr*lr*lr);
        if (!(t > 200 && t < 400) || t >= prev) { ok = false; break; } prev = t;
      }
      if (ok) {
        M.sh = true; M.A = coef[0]; M.B = coef[1]; M.C = coef[2];
        M.desc = "3-point Steinhart-Hart fit"; return;
      }
    }
    n = 2;  // fall back to the two most widely spaced points below
  }
  if (n == 2) {
    int i = 0, j = 1;
    if (S.calN >= 3) {   // pick widest pair
      double best = -1;
      for (int p = 0; p < 3; p++) for (int q = p + 1; q < 3; q++)
        if (fabs(S.calF[p] - S.calF[q]) > best) { best = fabs(S.calF[p] - S.calF[q]); i = p; j = q; }
    }
    double T1 = FtoK(S.calF[i]), T2 = FtoK(S.calF[j]);
    if (fabs(S.calF[i] - S.calF[j]) >= 10.0 && S.calR[i] > 0 && S.calR[j] > 0) {
      double b = log(S.calR[i] / S.calR[j]) / (1.0 / T1 - 1.0 / T2);
      if (b > 2500 && b < 5500) {
        M.beta = b; M.r25 = S.calR[i] * exp(b * (1.0 / 298.15 - 1.0 / T1));
        M.desc = "2-point fit, B=" + String((int)b) + ", R25=" + String((int)M.r25);
        return;
      }
    }
    n = 1;  // points too close: use newest as an offset
  }
  if (n == 1) {
    int k = S.calN - 1;
    M.ptOffsetF = S.calF[k] - betaTempF(S.calR[k], PROBE_BETA, PROBE_R25);
    M.desc = "default curve + 1-point offset (" + String(M.ptOffsetF, 1) + " F)";
    return;
  }
  M.desc = "default curve (B=" + String((int)PROBE_BETA) + ", R25=" + String((int)PROBE_R25) + ")";
}

double modelTempF(double ohms) {
  if (M.sh) { double l = log(ohms); return KtoF(1.0 / (M.A + M.B*l + M.C*l*l*l)); }
  return betaTempF(ohms, M.beta, M.r25) + M.ptOffsetF;
}

void histAccumulate(float t);

// Track unbroken runs of readings, compared at 0.1 F exactly as shown on the page
void resetRuns() { atOrAboveSince = 0; atOrBelowSince = 0; }
void updateRuns() {
  float t = roundf(tempF * 10.0f) / 10.0f;
  unsigned long now = millis(); if (now == 0) now = 1;
  if (t >= S.setpointF) { if (!atOrAboveSince) atOrAboveSince = now; } else atOrAboveSince = 0;
  if (t <= S.setpointF - S.diffF) { if (!atOrBelowSince) atOrBelowSince = now; } else atOrBelowSince = 0;
}

// read probe: average 32 quick ADC samples (removes electrical noise only)
void readProbe() {
  uint32_t sum = 0; const int N = 32;
  for (int i = 0; i < N; i++) sum += analogReadMilliVolts(PIN_PROBE);
  float mv = sum / (float)N;
  probeMv = mv;   // no extra smoothing: every reading is used as-is

  if (probeMv > 2850)         { probeFault = "probe open (not connected?)"; probeOhms = NAN; tempF = NAN; resetRuns(); return; }
  if (probeMv < 120)          { probeFault = "probe shorted";               probeOhms = NAN; tempF = NAN; resetRuns(); return; }
  probeFault = "";
  probeOhms = R1_OHMS * probeMv / (VCC_MV - probeMv);
  rawTempF  = modelTempF(probeOhms);
  tempF     = rawTempF + S.offsetF;
  updateRuns();
  histAccumulate(tempF);
}

// ===================== relays =====================
void ledColor(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_RGB, g, r, b); }   // this board's LED has red/green swapped

// heaterRun=true -> COLD relay (heater runs); false -> HOT relay (heater satisfied)
void applyRelays(bool heaterRun) {
  int on  = heaterRun ? PIN_COLD : PIN_HOT;
  int off = heaterRun ? PIN_HOT  : PIN_COLD;
  digitalWrite(on, HIGH);      // make before break: both briefly on reads "hotter" = safe
  delay(60);
  digitalWrite(off, LOW);
}

void setHeat(bool run, const String& why) {
  if (relaysStarted && run == heatOn) return;
  applyRelays(run);
  if (relaysStarted) logEvent(String(run ? "HEAT ON  - " : "HEAT OFF - ") + why);
  heatOn = run; relaysStarted = true; lastChange = millis(); everSwitched = true;
}

int minutesNow() {
  time_t t = time(nullptr); struct tm tmv; localtime_r(&t, &tmv);
  return tmv.tm_hour * 60 + tmv.tm_min;
}

bool scheduleAllows(String& why) {
  if (!S.schedOn) return true;
  if (!timeValid()) { why = "time not known yet - schedule ignored"; return true; }
  int m = minutesNow();
  bool in = (S.schedStart <= S.schedEnd) ? (m >= S.schedStart && m < S.schedEnd)
                                         : (m >= S.schedStart || m < S.schedEnd);  // overnight window
  if (!in) why = "outside heating schedule";
  return in;
}

String mmss(unsigned long ms) {
  unsigned long s = ms / 1000; char b[12]; snprintf(b, sizeof b, "%lu:%02lu", s / 60, s % 60); return b;
}

void controlStep() {
  String note;
  bool want = heatOn;
  bool immediate = false;   // skip minimum on/off times

  if (probeFault.length()) { want = false; immediate = true; reason = "Probe fault: " + probeFault + " - heater held off"; }
  else if (tempF >= 104.0) { want = false; immediate = true; reason = "Safety: water above 104 F - heater held off"; }
  else if (S.mode == MODE_OFF) { want = false; immediate = true; reason = "Forced OFF from web page"; }
  else if (S.mode == MODE_ON && millis() - forceOnSince >= FORCE_ON_LIMIT_MS) {
    S.mode = MODE_AUTO; saveSettings(); logEvent("Force on time limit (2 h) reached - back to AUTO");
    reason = "Back to Auto after the 2-hour Force on limit";
  }
  else if (S.mode == MODE_ON)  { want = true;  reason = "Forced ON from web page (back to Auto in " + mmss(FORCE_ON_LIMIT_MS - (millis() - forceOnSince)) + ")"; }
  else {
    String sw;
    bool allowed = scheduleAllows(sw);
    if (!allowed) { want = false; immediate = true; reason = "Off: " + sw; }
    else {
      unsigned long conf = (unsigned long)S.confirmMin * 60000UL, now = millis();
      float lo = S.setpointF - S.diffF;
      if (heatOn) {
        if (atOrAboveSince && now - atOrAboveSince >= conf) {
          want = false;
          reason = "Stopping: all readings >= " + String(S.setpointF, 1) + " F for " + mmss(conf);
        } else if (atOrAboveSince) {
          reason = "Heating: all readings >= " + String(S.setpointF, 1) + " F for " + mmss(now - atOrAboveSince) +
                   " of " + mmss(conf) + " - stops at " + mmss(conf);
        } else {
          reason = "Heating: water " + String(tempF, 1) + " F, below setpoint " + String(S.setpointF, 1) + " F (confirm timer waiting)";
        }
      } else {
        if (atOrBelowSince && now - atOrBelowSince >= conf) {
          want = true;
          reason = "Starting: all readings <= " + String(lo, 1) + " F for " + mmss(conf);
        } else if (atOrBelowSince) {
          reason = "Idle: all readings <= " + String(lo, 1) + " F for " + mmss(now - atOrBelowSince) +
                   " of " + mmss(conf) + " - starts at " + mmss(conf);
        } else {
          reason = "Idle: water " + String(tempF, 1) + " F, heat restarts after " + mmss(conf) + " at or below " + String(lo, 1) + " F";
        }
      }
      if (sw.length()) reason += " - " + sw;
    }
  }

  if (!relaysStarted) { setHeat(want, "startup"); return; }
  if (want == heatOn) return;

  unsigned long since = millis() - lastChange;
  if (!immediate && everSwitched) {
    if (want && !heatOn && since < (unsigned long)S.minOffMin * 60000UL) {
      reason += " - waiting minimum off time (" + mmss(S.minOffMin * 60000UL - since) + " left)"; return;
    }
  }
  setHeat(want, reason);
}

void updateLed() {
  static bool blink = false; blink = !blink;
  if (probeFault.length())           ledColor(blink ? 30 : 0, 0, 0);       // red blink
  else if (apMode)                   ledColor(0, 0, blink ? 25 : 4);       // blue pulse: setup network
  else if (WiFi.status() != WL_CONNECTED) ledColor(blink ? 20 : 0, 0, 20); // purple blink: no Wi-Fi
  else if (heatOn)                   ledColor(25, 8, 0);                   // orange: heating
  else                               ledColor(0, 12, 0);                   // green: idle
}

// ===================== long-term history (flash, LittleFS) =====================
// One 8-byte record every 5 minutes: time, average, low, high, % of the time heating.
// About 14 months fit in the free 1.5 MB flash area; when it is full the oldest record is overwritten.
const uint32_t HIST_STEP   = 300;            // seconds per record
const uint32_t HIST_EPOCH  = 1704067200UL;   // 2024-01-01 00:00 UTC (record times are minutes after this)
const uint32_t HIST_MAXCAP = 120000;         // 120000 x 5 min = about 416 days
const char*    HIST_FILE   = "/hist.bin";
uint32_t histCap = 0, histHead = 0, histCnt = 0, histLastT = 0;   // histHead = next physical slot to write
bool histOk = false;
uint32_t accSlot = 0, accN = 0; double accSum = 0; float accMin = 1e9, accMax = -1e9;
unsigned long accHeatMs = 0, accLastMs = 0;


void packRec(const HRec& r, uint8_t* b) {
  uint32_t tm = (r.t - HIST_EPOCH) / 60;
  uint16_t a = (uint16_t)(int16_t)lroundf(r.avg * 10);
  int dmin = constrain((int)lroundf((r.avg - r.mn) * 10), 0, 255);
  int dmax = constrain((int)lroundf((r.mx - r.avg) * 10), 0, 255);
  b[0] = tm & 0xFF; b[1] = (tm >> 8) & 0xFF; b[2] = (tm >> 16) & 0xFF;
  b[3] = a & 0xFF;  b[4] = a >> 8;
  b[5] = dmin; b[6] = dmax; b[7] = r.heat;
}
void unpackRec(const uint8_t* b, HRec& r) {
  uint32_t tm = b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
  int16_t a = (int16_t)(b[3] | (b[4] << 8));
  r.t = HIST_EPOCH + tm * 60; r.avg = a / 10.0f;
  r.mn = r.avg - b[5] / 10.0f; r.mx = r.avg + b[6] / 10.0f; r.heat = b[7];
}
uint32_t physOf(uint32_t logical) { return (histHead + histCap - histCnt + logical) % histCap; }
bool readRec(File& f, uint32_t logical, HRec& r) {
  uint8_t b[8];
  if (!f.seek(physOf(logical) * 8)) return false;
  if (f.read(b, 8) != 8) return false;
  unpackRec(b, r); return true;
}
void histSaveIndex() { prefs.begin("pool", false); prefs.putUInt("hhead", histHead); prefs.putUInt("hcnt", histCnt); prefs.end(); }

void histReset() {
  LittleFS.remove(HIST_FILE); histHead = 0; histCnt = 0; histLastT = 0; histSaveIndex();
}

void histInit() {
  if (!LittleFS.begin(true)) { logEvent("History storage not available (check Partition Scheme: Default 4MB with spiffs)"); return; }
  prefs.begin("pool", false);
  histCap = prefs.getUInt("hcap", 0);
  if (histCap == 0) {                      // first run: size the ring to fit, then never change it
    size_t total = LittleFS.totalBytes();
    uint32_t fit = (uint32_t)(((total > 65536) ? (total - 65536) : 0) * 0.8 / 8);
    histCap = min(HIST_MAXCAP, fit);
    prefs.putUInt("hcap", histCap);
  }
  histHead = prefs.getUInt("hhead", 0); histCnt = prefs.getUInt("hcnt", 0);
  prefs.end();
  if (histCap < 1000) { logEvent("History storage too small"); return; }
  size_t sz = 0;
  { File f = LittleFS.open(HIST_FILE, "r"); if (f) { sz = f.size(); f.close(); } }
  bool okIdx = histHead < histCap && histCnt <= histCap &&
               (histCnt < histCap ? (histHead == histCnt && sz / 8 >= histCnt) : (sz / 8 >= histCap));
  if (!okIdx) { if (histCnt || sz) logEvent("History index didn't match the file - starting a new history"); histReset(); }
  histOk = true;
  if (histCnt) { File f = LittleFS.open(HIST_FILE, "r"); HRec r; if (f && readRec(f, histCnt - 1, r)) histLastT = r.t; if (f) f.close(); }
  logEvent("History: " + String(histCnt) + " records (" + String(histCnt * HIST_STEP / 86400.0, 1) + " days) stored, room for " +
           String(histCap * HIST_STEP / 86400.0, 0) + " days");
}

void histAppend(const HRec& r) {
  uint8_t b[8]; packRec(r, b);
  size_t sz = 0;
  { File t = LittleFS.open(HIST_FILE, "r"); if (t) { sz = t.size(); t.close(); } }
  File f;
  if ((size_t)histHead * 8 >= sz) f = LittleFS.open(HIST_FILE, "a");          // still filling: append
  else { f = LittleFS.open(HIST_FILE, "r+"); if (f) f.seek(histHead * 8); }  // full: overwrite the oldest
  if (!f) { logEvent("History write failed"); return; }
  size_t w = f.write(b, 8); f.close();
  if (w != 8) { logEvent("History write failed"); return; }
  histHead = (histHead + 1) % histCap; if (histCnt < histCap) histCnt++;
  histLastT = r.t; histSaveIndex();
}

void accReset() { accN = 0; accSum = 0; accMin = 1e9; accMax = -1e9; accHeatMs = 0; }

void histAccumulate(float t) {
  if (isnan(t)) return;
  accN++; accSum += t; if (t < accMin) accMin = t; if (t > accMax) accMax = t;
}

// called every loop: adds up heating time and writes a record at each 5-minute boundary
void histTask() {
  unsigned long ms = millis();
  if (heatOn) accHeatMs += ms - accLastMs;
  accLastMs = ms;
  if (!histOk || !timeValid()) return;
  uint32_t slot = (uint32_t)time(nullptr) / HIST_STEP;
  if (accSlot == 0 || slot < accSlot) { accSlot = slot; accReset(); return; }
  if (slot == accSlot) return;
  if (accN > 0) {
    HRec r; r.t = accSlot * HIST_STEP; r.avg = accSum / accN; r.mn = accMin; r.mx = accMax;
    r.heat = (uint8_t)min(100UL, accHeatMs * 100UL / (HIST_STEP * 1000UL));
    if (r.t > histLastT) histAppend(r);
  }
  accSlot = slot; accReset();
}

// ===================== web =====================
String jsonEsc(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (char c : s) { if (c == '"' || c == '\\') { o += '\\'; o += c; } else if (c >= 32) o += c; }
  return o;
}
String hhmm(int m) { char b[6]; snprintf(b, sizeof b, "%02d:%02d", m / 60, m % 60); return b; }
int parseHHMM(const String& s, int def) {
  int c = s.indexOf(':'); if (c < 1) return def;
  int h = s.substring(0, c).toInt(), m = s.substring(c + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return def; return h * 60 + m;
}
String num(float v, int d = 1) { return isnan(v) ? "null" : String(v, d); }

void handleStatus() {
  String j = "{";
  j += "\"fw\":\"" + String(FW_VERSION) + "\",";
  j += "\"t\":" + num(tempF, 2) + ",\"traw\":" + num(rawTempF, 2) + ",";
  j += "\"ohms\":" + num(probeOhms, 0) + ",\"mv\":" + num(probeMv, 0) + ",";
  j += "\"fault\":\"" + jsonEsc(probeFault) + "\",";
  j += "\"heat\":" + String(heatOn ? "true" : "false") + ",";
  j += "\"relay\":\"" + String(heatOn ? "COLD (K2)" : "HOT (K1)") + "\",";
  j += "\"reason\":\"" + jsonEsc(reason) + "\",";
  j += "\"mode\":" + String(S.mode) + ",\"sp\":" + String(S.setpointF, 1) + ",\"df\":" + String(S.diffF, 1) + ",";
  j += "\"cf\":" + String(S.confirmMin) + ",\"moff\":" + String(S.minOffMin) + ",";
  j += "\"son\":" + String(S.schedOn ? "true" : "false") + ",\"ss\":\"" + hhmm(S.schedStart) + "\",\"se\":\"" + hhmm(S.schedEnd) + "\",";
  j += "\"off\":" + String(S.offsetF, 2) + ",\"model\":\"" + jsonEsc(M.desc) + "\",\"cal\":[";
  for (int i = 0; i < S.calN; i++) { if (i) j += ","; j += "[" + String(S.calR[i], 0) + "," + String(S.calF[i], 2) + "]"; }
  j += "],\"tv\":" + String(timeValid() ? "true" : "false") + ",\"time\":\"" + nowStr() + "\",";
  j += "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  j += "\"bssid\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.BSSIDstr() : String("")) + "\",";
  j += "\"wsrc\":\"" + String(wifiSaved ? "saved from this page" : "from the code") + "\",";
  j += "\"ssid\":\"" + jsonEsc(S.ssid) + "\",\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + ",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\",\"ap\":" + String(apMode ? "true" : "false") + ",";
  j += "\"up\":" + String(millis() / 1000) + ",\"r1\":" + String(R1_OHMS, 0) + ",\"log\":[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead - 1 - i + LOGN) % LOGN; if (i) j += ","; j += "\"" + jsonEsc(logBuf[idx]) + "\"";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleHistInfo() {
  String j = "{\"ok\":" + String(histOk ? "true" : "false") + ",\"step\":" + String(HIST_STEP) +
             ",\"cap\":" + String(histCap) + ",\"count\":" + String(histCnt) +
             ",\"days\":" + String(histCap * HIST_STEP / 86400.0, 0);
  if (histOk && histCnt) {
    File f = LittleFS.open(HIST_FILE, "r"); HRec a, b;
    if (f && readRec(f, 0, a) && readRec(f, histCnt - 1, b)) j += ",\"first\":" + String(a.t) + ",\"last\":" + String(b.t);
    if (f) f.close();
  }
  j += "}";
  server.send(200, "application/json", j);
}

// /api/hist?from=EPOCH&to=EPOCH&n=POINTS : records in the range, grouped into about n buckets
// Each point: [bucket start, average, lowest, highest, % heating]
void handleHist() {
  uint32_t from = strtoul(server.arg("from").c_str(), nullptr, 10), to = strtoul(server.arg("to").c_str(), nullptr, 10);
  int n = constrain(server.arg("n").toInt(), 10, 3000);
  if (to <= from) to = from + 3600;
  File f;
  if (histOk && histCnt) f = LittleFS.open(HIST_FILE, "r");
  if (!f) { server.send(200, "application/json", "{\"w\":300,\"p\":[]}"); return; }

  auto lowerBound = [&](uint32_t key) -> uint32_t {    // first record with time >= key
    uint32_t lo = 0, hi = histCnt; HRec r;
    while (lo < hi) { uint32_t mid = (lo + hi) / 2; if (readRec(f, mid, r) && r.t < key) lo = mid + 1; else hi = mid; }
    return lo;
  };
  uint32_t i0 = lowerBound(from), i1 = lowerBound(to + 1);
  uint32_t w = max((uint32_t)HIST_STEP, (to - from) / (uint32_t)n);

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String out = "{\"w\":" + String(w) + ",\"p\":[";
  bool first = true;
  uint32_t bk = UINT32_MAX, bn = 0, bh = 0; double bs = 0; float bmn = 0, bmx = 0;
  auto flush = [&]() {
    if (!bn) return;
    char tmp[72];
    snprintf(tmp, sizeof tmp, "%s[%lu,%.1f,%.1f,%.1f,%lu]", first ? "" : ",", (unsigned long)(from + bk * w),
             bs / bn, bmn, bmx, (unsigned long)(bh / bn));
    out += tmp; first = false;
    if (out.length() > 1800) { server.sendContent(out); out = ""; }
  };
  const uint32_t CH = 128; static uint8_t buf[CH * 8];
  uint32_t i = i0;
  while (i < i1) {
    uint32_t p = physOf(i), k = min(CH, min(i1 - i, histCap - p));
    f.seek(p * 8);
    if (f.read(buf, k * 8) != k * 8) break;
    for (uint32_t q = 0; q < k; q++) {
      HRec r; unpackRec(buf + q * 8, r);
      uint32_t b = (r.t - from) / w;
      if (b != bk) { flush(); bk = b; bn = 0; bh = 0; bs = 0; bmn = r.mn; bmx = r.mx; }
      bs += r.avg; bn++; bh += r.heat;
      if (r.mn < bmn) bmn = r.mn;
      if (r.mx > bmx) bmx = r.mx;
    }
    i += k; esp_task_wdt_reset();
  }
  flush(); f.close();
  out += "]}";
  server.sendContent(out);
  server.sendContent("");
}

void ok(const String& msg) { server.send(200, "application/json", "{\"ok\":true,\"msg\":\"" + jsonEsc(msg) + "\"}"); }
void bad(const String& msg) { server.send(400, "application/json", "{\"ok\":false,\"msg\":\"" + jsonEsc(msg) + "\"}"); }

void handleSettings() {
  float sp = server.arg("sp").toFloat(), df = server.arg("df").toFloat();
  int cf = server.arg("cf").toInt(), moff = server.arg("moff").toInt();
  if (sp < 50 || sp > 104) return bad("Setpoint must be 50-104 F");
  if (df < 0.1 || df > 5)  return bad("Neg Differential must be 0.1-5 F");
  if (cf < 0 || cf > 30)   return bad("Confirm time must be 0-30 minutes");
  if (moff < 0 || moff > 60) return bad("Min. off time must be 0-60 minutes");
  S.setpointF = sp; S.diffF = df; S.confirmMin = cf; S.minOffMin = moff;
  resetRuns();
  S.schedOn = server.arg("son") == "1";
  S.schedStart = parseHHMM(server.arg("ss"), S.schedStart);
  S.schedEnd   = parseHHMM(server.arg("se"), S.schedEnd);
  saveSettings();
  logEvent("Settings saved: setpoint " + String(sp, 1) + " F, neg differential " + String(df, 1) + " F, confirm " +
           String(cf) + " min, min off " + String(moff) + " min, schedule " +
           (S.schedOn ? hhmm(S.schedStart) + "-" + hhmm(S.schedEnd) : String("off")));
  ok("Saved");
}

void handleMode() {
  int m = server.arg("mode").toInt();
  if (m < 0 || m > 2) return bad("bad mode");
  S.mode = m; if (m == MODE_ON) forceOnSince = millis(); saveSettings();
  logEvent(String("Mode set to ") + (m == MODE_AUTO ? "AUTO" : m == MODE_OFF ? "FORCED OFF" : "FORCED ON"));
  ok("Mode changed");
}

void handleCalAdd() {
  if (isnan(probeOhms)) return bad("No probe reading");
  float f = server.arg("f").toFloat();
  if (f < 25 || f > 130) return bad("Enter the thermometer reading in F (25-130)");
  if (S.calN == 3) { for (int i = 0; i < 2; i++) { S.calR[i] = S.calR[i + 1]; S.calF[i] = S.calF[i + 1]; } S.calN = 2; }
  S.calR[S.calN] = probeOhms; S.calF[S.calN] = f; S.calN++;
  buildModel(); saveSettings();
  logEvent("Calibration point added: " + String(probeOhms, 0) + " ohm = " + String(f, 1) + " F -> " + M.desc);
  ok("Point added: " + M.desc);
}

void handleCalClear() {
  S.calN = 0; buildModel(); saveSettings(); logEvent("Calibration points cleared"); ok("Cleared");
}

void handleOffset() {
  if (server.hasArg("match")) {
    if (isnan(rawTempF)) return bad("No probe reading");
    float f = server.arg("match").toFloat();
    if (f < 25 || f > 130) return bad("Enter the thermometer reading in F");
    S.offsetF = f - rawTempF;
  } else {
    float o = server.arg("off").toFloat();
    if (o < -15 || o > 15) return bad("Offset must be -15 to +15 F");
    S.offsetF = o;
  }
  saveSettings(); logEvent("Installed offset set to " + String(S.offsetF, 2) + " F");
  ok("Offset " + String(S.offsetF, 2) + " F");
}

void handleWifi() {
  String ss = server.arg("ssid"), pw = server.arg("pass");
  if (ss.length() < 1 || ss.length() > 32) return bad("Network name required");
  if (pw.length() == 0 && ss == S.ssid) pw = S.pass;            // blank password = keep the current one
  if (pw.length() > 0 && pw.length() < 8) return bad("Wi-Fi passwords are at least 8 characters");
  S.ssid = ss; S.pass = pw;
  saveWifi(); logEvent("Wi-Fi settings saved for \"" + ss + "\" - restarting");
  ok("Saved. Restarting to join " + ss + "...");
  delay(800); ESP.restart();
}

// ---------- the web page ----------
const char PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Pool Heater</title>
<style>
:root{--bg:#0f1a20;--card:#18262e;--ink:#e6eef0;--mut:#93a6ad;--acc:#4cc3b9;--warm:#f0954f;--bad:#ef6b6b;--rule:#2c3d46}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
main{max-width:720px;margin:0 auto;padding:16px;display:grid;gap:14px}
.card{background:var(--card);border:1px solid var(--rule);border-radius:10px;padding:14px;display:grid;gap:10px}
h1{font-size:20px;margin:0}h2{font-size:15px;margin:0;color:var(--mut);text-transform:uppercase;letter-spacing:.06em}
.big{font-size:56px;font-weight:700;line-height:1}.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
.chip{padding:3px 10px;border-radius:12px;font-weight:600;font-size:14px}.on{background:#5a3518;color:var(--warm)}
.idle{background:#16413c;color:var(--acc)}.fault{background:#4a1f22;color:var(--bad)}
.mut{color:var(--mut);font-size:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
label{display:grid;gap:4px;font-size:14px;color:var(--mut)}input{font:inherit;padding:8px;border-radius:6px;border:1px solid var(--rule);background:#0d161b;color:var(--ink);width:100%}
input[type=checkbox]{width:auto}button{font:inherit;padding:9px 14px;border-radius:6px;border:1px solid var(--acc);background:#123a37;color:var(--ink);cursor:pointer}
button.sel{background:var(--acc);color:#062421;font-weight:700}button:focus-visible,input:focus-visible{outline:2px solid var(--acc)}
#hc{width:100%;height:280px;display:block;cursor:grab;touch-action:pan-y;background:#0d161b;border-radius:6px}#hs{width:100%;accent-color:var(--acc)}
#log{font:13px/1.5 ui-monospace,Consolas,monospace;max-height:220px;overflow:auto;white-space:pre-wrap;color:var(--mut)}
svg{width:100%;height:150px;display:block}#msg{min-height:1.2em;color:var(--acc)}table{border-collapse:collapse;font-size:14px}td{padding:2px 10px 2px 0}
</style></head><body><main>
<div class="row" style="justify-content:space-between"><h1>Pool Heater</h1><span class="mut" id="clock"></span></div>
<div class="card"><div class="row" style="justify-content:space-between">
<div><div class="big" id="temp">--</div><div class="mut">pool water, &deg;F</div></div>
<div style="text-align:right"><span class="chip idle" id="state">...</span><div class="mut" id="relay"></div></div></div>
<div id="reason" class="mut"></div>
<div class="row"><button data-m="0">Auto</button><button data-m="1">Force off</button><button data-m="2">Force on</button></div>
<div id="msg"></div></div>

<div class="card"><h2>Temperature history</h2>
<div class="row"><button class="hb" data-s="86400">Day</button><button class="hb" data-s="604800">Week</button><button class="hb" data-s="2592000">Month</button><button class="hb" data-s="7776000">3 months</button><button class="hb" data-s="31536000">Year</button><button class="hb" data-s="0">All</button></div>
<canvas id="hc"></canvas>
<input id="hs" type="range" min="0" max="1000" value="1000" aria-label="Move through time">
<div class="mut" id="hhover">&nbsp;</div>
<div class="mut" id="hinfo">Loading history...</div>
<div class="mut">Mouse wheel over the graph zooms in and out. Drag the graph or the slider to move through time. Line: average. Light band: low to high. Orange shading: share of time heating. Dashed: setpoint and restart level.</div></div>

<div class="card"><h2>Settings</h2><div class="grid">
<label>Setpoint &deg;F<input id="sp" type="number" step="0.1" min="50" max="104"></label>
<label>Neg Differential &deg;F<input id="df" type="number" step="0.1" min="0.1" max="5"></label>
<label>Confirm time (min)<input id="cf" type="number" min="0" max="30"></label>
<label>Min. off time (min)<input id="moff" type="number" min="0" max="60"></label></div>
<label class="row" style="color:var(--ink)"><input id="son" type="checkbox"> Heat only during this time window</label>
<div class="grid"><label>From<input id="ss" type="time"></label><label>Until<input id="se" type="time"></label></div>
<div class="mut" id="rules"></div>
<div><button id="save">Save settings</button></div></div>

<div class="card"><h2>Calibration</h2>
<div class="mut">Probe: <b id="ohms">--</b> &Omega; (<span id="mv">--</span> mV) &middot; curve: <span id="model"></span> &middot; installed offset <span id="offv2"></span> &deg;F</div>
<div id="calbtnrow"><button id="calopen">Calibrate&hellip;</button></div>
<div id="calbox" hidden style="display:grid;gap:10px">
<div class="mut"><b>Bench points</b> (probe and thermometer together in water, wait 2-3 min to settle). Up to 3; the oldest is replaced.</div>
<table id="calt"></table>
<div class="row"><label style="flex:1">Thermometer &deg;F<input id="calf" type="number" step="0.1"></label>
<button id="caladd">Add point</button><button id="calclr">Clear points</button></div>
<div class="mut"><b>Installed offset</b> (after mounting on the pool wall): <span id="offv"></span> &deg;F</div>
<div class="row"><label style="flex:1">Pool thermometer &deg;F<input id="matchf" type="number" step="0.1"></label>
<button id="match">Match reading</button><button id="offclr">Reset offset</button></div>
<div><button id="caldone">Done</button></div></div></div>

<div class="card"><h2>Wi-Fi</h2><div class="mut" id="wifi"></div><div class="mut" id="wsrc"></div>
<div id="wbtnrow"><button id="wopen">Change Wi-Fi&hellip;</button></div>
<div id="wbox" style="display:none;gap:10px">
<div class="grid"><label>Network name<input id="ssid" autocomplete="off"></label><label>Password (leave blank to keep the current one)<input id="pass" type="password" autocomplete="off"></label></div>
<div class="row"><button id="wsave">Save and restart</button><button id="wforget">Use the code's Wi-Fi lines</button><button id="wcancel">Cancel</button></div></div></div>

<div class="card"><h2>Event log</h2><div id="log"></div><div class="mut" id="fw"></div></div>
</main><script>
const $=id=>document.getElementById(id);let filled=false,last=null;
async function post(url,data){const r=await fetch(url,{method:'POST',body:new URLSearchParams(data)});
 let j={};try{j=await r.json()}catch(e){} $('msg').textContent=j.msg||(r.ok?'Done':'Error');$('msg').style.color=r.ok?'':'var(--bad)';refresh();}
function fill(s){$('sp').value=s.sp;$('df').value=s.df;$('cf').value=s.cf;$('moff').value=s.moff;$('son').checked=s.son;$('ss').value=s.ss;$('se').value=s.se;$('ssid').value=s.ssid;filled=true;}
async function refresh(){try{const s=await (await fetch('/api/status')).json();
 if(!filled){fill(s);drawH();}
 $('temp').textContent=s.t==null?'--':s.t.toFixed(1);
 const st=$('state');if(s.fault){st.textContent='Probe fault';st.className='chip fault'}else if(s.heat){st.textContent='Heating';st.className='chip on'}else{st.textContent='Idle';st.className='chip idle'}
 $('relay').textContent='Heater sees: '+s.relay;last=s;
 $('rules').textContent=`Heater stops after ${s.cf} min with every reading at or above ${s.sp.toFixed(1)} \u00b0F. It restarts after ${s.cf} min with every reading at or below ${(s.sp-s.df).toFixed(1)} \u00b0F.`;$('reason').textContent=s.reason;
 document.querySelectorAll('[data-m]').forEach(b=>b.classList.toggle('sel',+b.dataset.m===s.mode));
 $('clock').textContent=s.tv?s.time:'time not set yet';
 $('ohms').textContent=s.ohms==null?'--':s.ohms;$('mv').textContent=s.mv==null?'--':s.mv;$('model').textContent=s.model;
 $('calt').innerHTML=s.cal.length?'<tr><td>#</td><td>Probe &Omega;</td><td>&deg;F</td></tr>'+s.cal.map((p,i)=>`<tr><td>${i+1}</td><td>${p[0]}</td><td>${p[1]}</td></tr>`).join(''):'<tr><td class="mut">No bench points - using your measured default curve.</td></tr>';
 $('offv').textContent=$('offv2').textContent=(s.off>=0?'+':'')+s.off.toFixed(2);
 $('wifi').textContent=s.ap?'Setup network active (PoolHeater-Setup). Enter your Wi-Fi below.':(s.wifi?`Connected to ${s.ssid} (access point ${s.bssid}), signal ${s.rssi} dBm (${s.rssi>=-67?'solid':s.rssi>=-75?'OK':'weak'}), IP ${s.ip}`:'Not connected');
 $('wsrc').textContent='Wi-Fi details: '+s.wsrc;
 $('log').textContent=s.log.join('\n');$('fw').textContent='Firmware '+s.fw+' - up '+Math.floor(s.up/3600)+' h '+Math.floor(s.up/60)%60+' min - R1 '+s.r1+' ohm';
}catch(e){$('reason').textContent='Lost contact with the controller...'}}
const HC=$('hc'),HS=$('hs'),PAD={l:46,r:10,t:10,b:24};
let HI={count:0,first:0,last:0,step:300,days:0},V={end:0,span:86400,live:true},HD=null,hT=0,hA=null,drag=null;
function hTotal(){return Math.max(HI.last-HI.first,0)}
function clampV(){const hiEnd=HI.last+HI.step,mx=Math.max(hTotal()+HI.step,86400);
 V.span=Math.min(Math.max(V.span,3600),mx);const loEnd=HI.first+V.span;
 if(V.end>hiEnd)V.end=hiEnd;if(V.end<loEnd)V.end=Math.min(loEnd,hiEnd);V.live=V.end>=hiEnd-1;
 const room=hiEnd-HI.first-V.span;HS.disabled=room<=0;HS.value=room>0?Math.round((V.end-HI.first-V.span)/room*1000):1000}
function fmtD(t){return new Date(t*1000).toLocaleDateString(undefined,{month:'short',day:'numeric',year:'numeric'})}
function fmtSpan(s){return s>=86400?(s/86400).toFixed(1)+' days':s>=3600?(s/3600).toFixed(1)+' h':Math.round(s/60)+' min'}
async function hInfo(){try{const i=await (await fetch('/api/hinfo')).json();HI=i;
 if(!i.count){$('hinfo').textContent=i.ok?'No history yet. A record is saved every 5 minutes once the clock is set.':'History storage is not available.';drawH();return}
 if(V.live||!V.end)V.end=i.last+i.step;clampV();
 $('hinfo').textContent=`Stored: ${i.count.toLocaleString()} records, ${fmtD(i.first)} to ${fmtD(i.last)}. Room for about ${i.days} days; the oldest records are replaced first.`;
 hLoad()}catch(e){}}
function hSched(){drawH();clearTimeout(hT);hT=setTimeout(hLoad,150)}
async function hLoad(){if(!HI.count)return;if(hA)hA.abort();hA=new AbortController();
 const from=Math.floor(V.end-V.span),to=Math.ceil(V.end),n=Math.max(50,Math.round(HC.clientWidth-PAD.l-PAD.r));
 try{const d=await (await fetch(`/api/hist?from=${from}&to=${to}&n=${n}`,{signal:hA.signal})).json();HD=d;drawH()}catch(e){}}
function niceStep(x){if(!(x>0))return 1;const p=Math.pow(10,Math.floor(Math.log10(x))),f=x/p;return Math.max(0.1,(f<1.5?1:f<3.5?2:f<7.5?5:10)*p)}
function timeAxis(c,X,t0,x0,x1,y0,y1){c.textAlign='center';c.fillStyle='#93a6ad';c.strokeStyle='#22323a';c.lineWidth=1;
 const span=V.span,want=Math.max(2,Math.floor((x1-x0)/90)),ticks=[];
 if(span/86400>75){const ms=[1,2,3,6,12].find(m=>span/(m*30.44*86400)<=want)||12;const d=new Date(t0*1000);d.setDate(1);d.setHours(0,0,0,0);
  while(d.getMonth()%ms)d.setMonth(d.getMonth()+1);
  for(let g=0;g<200&&d.getTime()/1000<=V.end;g++){const t=d.getTime()/1000;if(t>=t0)ticks.push([t,d.toLocaleDateString(undefined,d.getMonth()===0?{month:'short',year:'numeric'}:{month:'short'})]);d.setMonth(d.getMonth()+ms)}
 }else{const steps=[3600,7200,10800,21600,43200,86400,172800,604800,1209600],st=steps.find(s=>span/s<=want)||1209600;
  const d=new Date(t0*1000);d.setMinutes(0,0,0);
  if(st>=86400)d.setHours(0);else{const h=st/3600;d.setHours(Math.floor(d.getHours()/h)*h)}
  for(let g=0;g<400;g++){const t=d.getTime()/1000;if(t>V.end)break;if(t>=t0){const lab=st<86400&&d.getHours()!==0?d.toLocaleTimeString(undefined,{hour:'numeric'}):d.toLocaleDateString(undefined,{month:'short',day:'numeric'});ticks.push([t,lab])}
   if(st<86400)d.setHours(d.getHours()+st/3600);else d.setDate(d.getDate()+st/86400)}}
 ticks.forEach(([t,l])=>{const x=Math.round(X(t))+.5;c.beginPath();c.moveTo(x,y0);c.lineTo(x,y1);c.stroke();c.fillText(l,x,y1+16)})}
function drawH(){const dpr=window.devicePixelRatio||1,W=HC.clientWidth||600,H=280;
 if(HC.width!==Math.round(W*dpr)||HC.height!==Math.round(H*dpr)){HC.width=Math.round(W*dpr);HC.height=Math.round(H*dpr)}
 const c=HC.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,W,H);
 const x0=PAD.l,x1=W-PAD.r,y0=PAD.t,y1=H-PAD.b,t0=V.end-V.span,X=t=>x0+(t-t0)/V.span*(x1-x0);
 c.font='12px system-ui,sans-serif';c.fillStyle='#93a6ad';c.textAlign='left';
 if(!HI.count||!HD){c.fillText(HI.count?'Loading...':'No history yet',x0+8,y0+20);return}
 const P=HD.p.filter(p=>p[0]+HD.w>t0&&p[0]<V.end);
 if(!P.length){c.fillText('No data in this range',x0+8,y0+20);timeAxis(c,X,t0,x0,x1,y0,y1);return}
 let lo=Infinity,hi=-Infinity;P.forEach(p=>{lo=Math.min(lo,p[2]);hi=Math.max(hi,p[3])});
 if(hi-lo<1){const m=(hi+lo)/2;lo=m-0.5;hi=m+0.5}const pd=(hi-lo)*0.08;lo-=pd;hi+=pd;
 const Y=v=>y1-(v-lo)/(hi-lo)*(y1-y0);
 P.forEach(p=>{if(p[4]>0){const a=Math.max(x0,X(p[0])),b=Math.min(x1,X(p[0]+HD.w));c.fillStyle=`rgba(240,149,79,${(p[4]/100*0.3).toFixed(3)})`;c.fillRect(a,y0,Math.max(1,b-a),y1-y0)}});
 const ys=niceStep((hi-lo)/5);c.strokeStyle='#22323a';c.lineWidth=1;c.fillStyle='#93a6ad';c.textAlign='right';
 for(let v=Math.ceil(lo/ys)*ys;v<=hi;v+=ys){const y=Math.round(Y(v))+.5;c.beginPath();c.moveTo(x0,y);c.lineTo(x1,y);c.stroke();c.fillText(v.toFixed(ys<1?1:0)+'°',x0-5,y+4)}
 timeAxis(c,X,t0,x0,x1,y0,y1);
 if(last)[[last.sp,'#f0954f'],[last.sp-last.df,'#93a6ad']].forEach(([v,col])=>{if(v>lo&&v<hi){const y=Math.round(Y(v))+.5;c.strokeStyle=col;c.setLineDash([6,5]);c.beginPath();c.moveTo(x0,y);c.lineTo(x1,y);c.stroke();c.setLineDash([])}});
 const gap=HD.w*2.5,segs=[];let cur=[];P.forEach((p,i)=>{if(i&&p[0]-P[i-1][0]>gap){segs.push(cur);cur=[]}cur.push(p)});segs.push(cur);
 const mid=p=>X(p[0]+HD.w/2);c.save();c.beginPath();c.rect(x0,y0,x1-x0,y1-y0);c.clip();
 segs.forEach(sg=>{if(sg.length===1){const p=sg[0];c.fillStyle='#4cc3b9';c.beginPath();c.arc(mid(p),Y(p[1]),2.5,0,7);c.fill();return}
  c.fillStyle='rgba(76,195,185,.22)';c.beginPath();sg.forEach((p,i)=>{i?c.lineTo(mid(p),Y(p[3])):c.moveTo(mid(p),Y(p[3]))});for(let i=sg.length-1;i>=0;i--)c.lineTo(mid(sg[i]),Y(sg[i][2]));c.closePath();c.fill();
  c.strokeStyle='#4cc3b9';c.lineWidth=1.6;c.beginPath();sg.forEach((p,i)=>{i?c.lineTo(mid(p),Y(p[1])):c.moveTo(mid(p),Y(p[1]))});c.stroke()});
 c.restore();
 if(V.hx!=null&&V.hx>=x0&&V.hx<=x1){c.strokeStyle='#e6eef0';c.globalAlpha=.4;c.beginPath();c.moveTo(V.hx+.5,y0);c.lineTo(V.hx+.5,y1);c.stroke();c.globalAlpha=1}}
function hover(px){V.hx=px;drawH();if(!HD||!HD.p.length)return;const W=HC.clientWidth,t=V.end-V.span+(px-PAD.l)/(W-PAD.l-PAD.r)*V.span;
 let best=null,bd=Infinity;HD.p.forEach(p=>{const d=Math.abs(p[0]+HD.w/2-t);if(d<bd){bd=d;best=p}});
 if(!best||bd>HD.w*2){$('hhover').innerHTML='&nbsp;';return}
 const d=new Date(best[0]*1000);$('hhover').textContent=`${d.toLocaleString(undefined,{weekday:'short',month:'short',day:'numeric',year:'numeric',hour:'numeric',minute:'2-digit'})}${HD.w>300?' + '+fmtSpan(HD.w):''}: average ${best[1].toFixed(1)} °F, low ${best[2].toFixed(1)}, high ${best[3].toFixed(1)}, heating ${best[4]}% of the time`}
HC.addEventListener('wheel',e=>{if(!HI.count)return;e.preventDefault();const r=HC.getBoundingClientRect(),fx=Math.min(1,Math.max(0,(e.clientX-r.left-PAD.l)/(r.width-PAD.l-PAD.r))),ta=V.end-V.span+fx*V.span;
 V.span*=e.deltaY>0?1.25:0.8;V.end=ta+(1-fx)*V.span;clampV();hSched()},{passive:false});
HC.addEventListener('pointerdown',e=>{if(!HI.count)return;drag={x:e.clientX,end:V.end};HC.setPointerCapture(e.pointerId);HC.style.cursor='grabbing'});
HC.addEventListener('pointermove',e=>{const r=HC.getBoundingClientRect();if(drag){V.end=drag.end-(e.clientX-drag.x)/(r.width-PAD.l-PAD.r)*V.span;clampV();hSched();return}hover(e.clientX-r.left)});
['pointerup','pointercancel'].forEach(ev=>HC.addEventListener(ev,()=>{drag=null;HC.style.cursor='grab'}));
HC.addEventListener('pointerleave',()=>{V.hx=null;drawH();$('hhover').innerHTML='&nbsp;'});
HS.oninput=()=>{const room=HI.last+HI.step-HI.first-V.span;V.end=HI.first+V.span+HS.value/1000*room;clampV();hSched()};
document.querySelectorAll('.hb').forEach(b=>b.onclick=()=>{const sp=+b.dataset.s;V.span=sp||hTotal()+HI.step;V.end=HI.last+HI.step;clampV();hSched()});
window.addEventListener('resize',()=>hSched());
document.querySelectorAll('[data-m]').forEach(b=>b.onclick=()=>post('/api/mode',{mode:b.dataset.m}));
$('save').onclick=()=>post('/api/settings',{sp:$('sp').value,df:$('df').value,cf:$('cf').value,moff:$('moff').value,son:$('son').checked?1:0,ss:$('ss').value,se:$('se').value});
const sure=(q,fn)=>()=>{if(confirm(q))fn()};
function calShow(on){$('calbox').hidden=!on;$('calbox').style.display=on?'grid':'none';$('calbtnrow').style.display=on?'none':''}
calShow(false);
$('calopen').onclick=sure('Calibration changes how the pool temperature is measured. Only continue if you have the probe in an ice bath or next to an accurate thermometer. Open calibration?',()=>calShow(true));
$('caldone').onclick=()=>calShow(false);
$('caladd').onclick=sure('Add a calibration point? If 3 points are already stored, the oldest one is replaced.',()=>post('/api/cal/add',{f:$('calf').value}));
$('calclr').onclick=sure('Delete ALL calibration points? The temperature will go back to the built-in curve.',()=>post('/api/cal/clear',{}));
$('match').onclick=sure('Set the installed offset so the reading matches your pool thermometer?',()=>post('/api/offset',{match:$('matchf').value}));
$('offclr').onclick=sure('Reset the installed offset to 0?',()=>post('/api/offset',{off:0}));
function wShow(on){$('wbox').style.display=on?'grid':'none';$('wbtnrow').style.display=on?'none':''}
$('wopen').onclick=()=>wShow(true);$('wcancel').onclick=()=>wShow(false);
$('wsave').onclick=sure('Save these Wi-Fi details and restart? If they are wrong, the controller will fall back to the Wi-Fi lines in its code.',()=>post('/api/wifi',{ssid:$('ssid').value,pass:$('pass').value}));
$('wforget').onclick=sure('Remove the Wi-Fi details saved on the board and use the ones in the code? It will restart.',()=>post('/api/wifi/forget',{}));
refresh();hInfo();setInterval(refresh,1000);setInterval(hInfo,60000);
</script></body></html>)rawliteral";

void handleWifiForget() {
  forgetWifi(); logEvent("Saved Wi-Fi details removed - using the code's Wi-Fi lines; restarting");
  ok("Using the Wi-Fi details from the code. Restarting...");
  delay(800); ESP.restart();
}

void setupWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/hinfo", HTTP_GET, handleHistInfo);
  server.on("/api/hist", HTTP_GET, handleHist);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/cal/add", HTTP_POST, handleCalAdd);
  server.on("/api/cal/clear", HTTP_POST, handleCalClear);
  server.on("/api/offset", HTTP_POST, handleOffset);
  server.on("/api/wifi", HTTP_POST, handleWifi);
  server.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
  server.onNotFound([]() {           // send stray requests (e.g. phone captive checks in setup mode) to the page
    server.sendHeader("Location", "/"); server.send(302, "text/plain", "");
  });
  server.begin();
}

// ===================== Wi-Fi / OTA =====================
// With a repeater or mesh using the same network name, join the STRONGEST access point, not the first one found
void wifiConnect() {
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  // If details saved from the page keep failing, every other attempt uses the code's lines instead
  bool useCode = wifiSaved && (wifiAttempt % 2 == 1) &&
                 (S.ssid != DEFAULT_WIFI_SSID || S.pass != DEFAULT_WIFI_PASS);
  const char* ss = useCode ? DEFAULT_WIFI_SSID : S.ssid.c_str();
  const char* pw = useCode ? DEFAULT_WIFI_PASS : S.pass.c_str();
  Serial.printf("Wi-Fi: trying \"%s\" (%s)\n", ss, useCode ? "code's lines" : (wifiSaved ? "saved on board" : "code's lines"));
  wifiAttempt++;
  WiFi.begin(ss, pw);
}

void startOta() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { logEvent("OTA update starting"); });
  ArduinoOTA.onProgress([](unsigned int, unsigned int) { esp_task_wdt_reset(); });   // keep watchdog happy during upload
  ArduinoOTA.onError([](ota_error_t e) { logEvent("OTA error " + String((int)e)); });
  ArduinoOTA.begin();
  MDNS.addService("http", "tcp", 80);
  otaStarted = true;
}

void wifiTask() {
  static bool wasConnected = false;
  static unsigned long lastTry = 0;
  bool c = WiFi.status() == WL_CONNECTED;
  if (c && !wasConnected) {
    staConnectedAt = millis();
    logEvent("Wi-Fi connected to " + S.ssid + " via " + WiFi.BSSIDstr() + ", IP " + WiFi.localIP().toString() + ", signal " + String(WiFi.RSSI()) + " dBm");
    Serial.printf("Open http://%s.local or http://%s\n", HOSTNAME, WiFi.localIP().toString().c_str());
    if (wifiSaved && WiFi.SSID() == DEFAULT_WIFI_SSID && S.ssid != DEFAULT_WIFI_SSID) {
      forgetWifi(); logEvent("Connected with the code's Wi-Fi lines - removed the saved details that weren't working");
    }
    wifiAttempt = 0;
    if (!otaStarted) startOta();
  }
  if (!c && wasConnected) logEvent("Wi-Fi lost - retrying");
  wasConnected = c;
  if (c) lastConnectedMs = millis();

  // weak signal for 2 minutes -> reconnect, which picks the strongest access point (repeater vs main router)
  if (c && WiFi.RSSI() < -75) {
    if (!weakSince) weakSince = millis();
    else if (millis() - weakSince > 120000) {
      logEvent("Weak signal (" + String(WiFi.RSSI()) + " dBm) - reconnecting to the strongest access point");
      weakSince = 0; WiFi.disconnect(); delay(100); wifiConnect(); lastTry = millis();
    }
  } else weakSince = 0;

  // no Wi-Fi for 3 minutes -> open setup network (keeps retrying the home network too)
  if (!c && !apMode && millis() - lastConnectedMs > 180000) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("PoolHeater-Setup", "poolheater");
    apMode = true;
    logEvent("Setup network PoolHeater-Setup started - browse to http://192.168.4.1");
  }
  // connected for 2 minutes -> close setup network
  if (c && apMode && millis() - staConnectedAt > 120000) {
    WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); apMode = false;
    logEvent("Setup network closed");
  }
  if (!c && millis() - lastTry > 30000) { lastTry = millis(); wifiConnect(); }
}

// ===================== setup / loop =====================
void setup() {
  // relays first: both off, then HOT on as soon as possible so the heater never sees an open sensor for long
  pinMode(PIN_HOT, OUTPUT);  digitalWrite(PIN_HOT, LOW);
  pinMode(PIN_COLD, OUTPUT); digitalWrite(PIN_COLD, LOW);
  digitalWrite(PIN_HOT, HIGH);          // heater satisfied (off) while we start up
  heatOn = false; relaysStarted = true; lastChange = millis(); everSwitched = false;

  setCpuFrequencyMhz(80);   // slowest speed that keeps Wi-Fi: less power and heat

  bootMs = millis();
  Serial.begin(115200);
  unsigned long t0 = millis(); while (!Serial && millis() - t0 < 1500) { }
  ledColor(0, 0, 10);

  analogReadResolution(12);   // default attenuation reads 0 - ~3.1 V

  // watchdog: if the program ever freezes for 10 s the chip restarts (and starts with the heater OFF)
  esp_task_wdt_config_t wdt = { .timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);

  bool codeChanged = codeWifiChanged();
  loadSettings();
  buildModel();
  const char* rr[] = {"unknown", "power-on", "external", "software", "crash", "interrupt watchdog",
                      "task watchdog", "other watchdog", "deep sleep", "brownout (power dip)", "SDIO"};
  int r = (int)esp_reset_reason();
  logEvent(String("Pool heater controller v") + FW_VERSION + " starting (reset: " + (r >= 0 && r <= 10 ? rr[r] : "other") + "), curve: " + M.desc);
  if (codeChanged) logEvent("Wi-Fi lines in the code are new - using them");
  histInit();
  if (S.mode == MODE_ON) { S.mode = MODE_AUTO; saveSettings(); logEvent("Was in Force on before restart - back to AUTO"); }
  for (int i = 0; i < 6; i++) { readProbe(); delay(20); }

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  lastConnectedMs = millis();
  wifiConnect();
  configTzTime(TIMEZONE, "pool.ntp.org", "time.nist.gov");
  setupWeb();
}

void loop() {
  static unsigned long lastProbe = 0, lastCtl = 0, lastLed = 0;
  static bool timeLogged = false;
  unsigned long now = millis();

  esp_task_wdt_reset();
  server.handleClient();
  if (otaStarted) ArduinoOTA.handle();
  wifiTask();

  if (now - lastProbe >= 250)  { lastProbe = now; readProbe(); }
  if (now - lastCtl   >= 1000) { lastCtl = now; controlStep(); }
  if (now - lastLed   >= 500)  { lastLed = now; updateLed(); }
  if (!timeLogged && timeValid()) { timeLogged = true; logEvent("Clock set from internet"); }
  histTask();
  delay(2);
}
