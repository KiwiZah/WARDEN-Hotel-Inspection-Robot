/*
 * WARDEN Sensor Hub + Assistant TFT  --  ESP32 DevKitC V4
 * =====================================================================
 * Sensors / servo / LED / RGB / serial protocol unchanged + working.
 * AMG8833 read raw+chunked (no buffer overflow). Plus ILI9341 + XPT2046
 * touch "assistant mode" that pushes guest requests to the dashboard.
 *
 * TFT PINS (rewired off the sensor bus):
 *   SDI/MOSI + T_DIN : GPIO32   (NOT 23 = LED gate)
 *   T_CS             : GPIO17   (NOT 21 = I2C SDA)
 *   T_IRQ            : GPIO35   (NOT 22 = I2C SCL; wired, but firmware POLLS so it's unused)
 *   SCK / T_CLK      : GPIO18      SDO/MISO / T_DO : GPIO19
 *   TFT_CS : GPIO15   DC : GPIO2   RST : GPIO4   VCC,LED->3V3  GND->GND
 *
 * Sensor/actuator pins (do NOT touch): I2C 21/22, LED 23, servo 13,
 *   RGB 25/26/27, MQ-135 34.
 *
 * Libraries: Adafruit BME280, Adafruit BusIO, ESP32Servo,
 *            Adafruit GFX, Adafruit ILI9341, XPT2046_Touchscreen
 *
 * Protocol: PING/LED/SERVO/STATUS/ENV/THERMAL/RF (unchanged) +
 *   EVENT -> {"alert":"none|navigation|staff|emergency"}   (Pi polls this)
 */

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

const int PIN_LED_GATE = 23;
const int PIN_SERVO    = 13;
const int PIN_R = 25, PIN_G = 26, PIN_B = 27;
const int PIN_MQ135 = 34;
const int SERVO_US_MIN = 500, SERVO_US_MAX = 2500;

#define TFT_SCK 18
#define TFT_MISO 19
#define TFT_MOSI 32
#define TFT_CS 15
#define TFT_DC  2
#define TFT_RST 4
#define TOUCH_CS 17
#define TFT_IRQ  35      // touch IRQ wired to 35; firmware polls, so this stays unused

Adafruit_BME280 bme;
Servo           panServo;
bool   haveBME = false, haveAMG = false;
uint8_t amgAddr = 0;

SPIClass tftSPI(VSPI);
Adafruit_ILI9341 tft(&tftSPI, TFT_DC, TFT_CS, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);            // polling (robust; ignores T_IRQ)
// To make the firmware USE the IRQ pin instead, replace the line above with:
//   XPT2046_Touchscreen ts(TOUCH_CS, TFT_IRQ);

// touch calibration (raw ADC at screen edges, measured on this panel)
//   vertical   = raw X : top 1189 -> bottom 3412
//   horizontal = raw Y : left 3625 -> right 485
#define TS_TOP     1189    // raw X at top edge      (screen y = 0)
#define TS_BOTTOM  3412    // raw X at bottom edge    (screen y = 240)
#define TS_LEFT    3625    // raw Y at left edge      (screen x = 0)
#define TS_RIGHT   485     // raw Y at right edge     (screen x = 320)

#define C_BG     0x0841
#define C_CARD   0x18E3
#define C_ACCENT 0x07FF
#define C_TEXT   0xFFFF
#define C_DIM    0x8410
#define C_RED    0xF1A7
#define C_AMBER  0xFD20

enum Screen { HOME, SUB };
Screen screen = HOME;
String currentAlert = "none";          // guest request pushed to the dashboard
uint32_t lastTouchMs = 0;

// ================= RGB / SERVO =================
void setStatus(int r, int g, int b) {
  digitalWrite(PIN_R, r ? HIGH : LOW);
  digitalWrite(PIN_G, g ? HIGH : LOW);
  digitalWrite(PIN_B, b ? HIGH : LOW);
}
void servoDeg(int deg) { panServo.write(constrain(deg, 0, 180)); }

// ================= AMG8833 (raw, chunked) =================
bool amgWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(amgAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool amgBegin() {
  amgAddr = 0; uint8_t a[2] = {0x69, 0x68};
  for (int i = 0; i < 2; i++) { Wire.beginTransmission(a[i]); if (Wire.endTransmission() == 0) { amgAddr = a[i]; break; } }
  if (!amgAddr) return false;
  amgWrite(0x00, 0x00); amgWrite(0x01, 0x3F); delay(50);
  amgWrite(0x02, 0x00); amgWrite(0x03, 0x00); delay(100);
  return true;
}
bool amgReadGrid(float *out) {
  for (int chunk = 0; chunk < 4; chunk++) {
    Wire.beginTransmission(amgAddr);
    Wire.write((uint8_t)(0x80 + chunk * 32));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)amgAddr, 32) != 32) return false;
    for (int i = 0; i < 16; i++) {
      if (Wire.available() < 2) return false;
      uint8_t lo = Wire.read(), hi = Wire.read();
      int16_t raw = (int16_t)((hi << 8) | lo);
      if (raw & 0x0800) raw |= 0xF000;
      out[chunk * 16 + i] = raw * 0.25f;
    }
  }
  return true;
}

// ================= TFT ASSISTANT UI =================
void drawButton(int x, int y, int w, int h, const char *label, uint16_t accent) {
  tft.fillRoundRect(x, y, w, h, 8, C_CARD);
  tft.drawRoundRect(x, y, w, h, 8, accent);
  tft.setTextColor(C_TEXT); tft.setTextSize(2);
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds(label, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor(x + (w - bw) / 2, y + (h - bh) / 2);
  tft.print(label);
}
void drawHeader(const char *title) {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_ACCENT); tft.setTextSize(2); tft.setCursor(12, 12); tft.print("WARDEN");
  tft.setTextColor(C_DIM); tft.setCursor(110, 16); tft.print(title);
  tft.drawFastHLine(12, 38, 296, C_DIM);
}
void drawHome() {
  screen = HOME;
  drawHeader("ASSISTANT");
  tft.setTextColor(C_TEXT); tft.setTextSize(2); tft.setCursor(12, 50); tft.print("How can I help?");
  drawButton(12, 80,  296, 44, "Navigation Help", C_ACCENT);
  drawButton(12, 134, 296, 44, "Call Staff",      C_AMBER);
  drawButton(12, 188, 296, 44, "Emergency Alert", C_RED);
}
void drawSub(const char *title, const char *big, const char *sub, uint16_t col) {
  screen = SUB;
  drawHeader(title);
  if (col == C_RED) tft.fillRect(0, 44, 320, 6, C_RED);    // emergency emphasis
  tft.setTextColor(col); tft.setTextSize(3); tft.setCursor(12, 74); tft.print(big);
  tft.setTextColor(C_TEXT); tft.setTextSize(2); tft.setCursor(12, 120); tft.print(sub);
  tft.setTextColor(C_DIM); tft.setTextSize(2); tft.setCursor(12, 150); tft.print("Awaiting host poll");
  drawButton(12, 188, 296, 44, "< Done", C_DIM);
}
bool hit(int tx, int ty, int x, int y, int w, int h) { return tx >= x && tx <= x + w && ty >= y && ty <= y + h; }

void handleTouch() {
  if (!ts.touched()) return;
  if (millis() - lastTouchMs < 350) return;
  lastTouchMs = millis();
  TS_Point p = ts.getPoint();                                  // raw (ts.setRotation(0))
  int tx = constrain(map(p.y, TS_LEFT, TS_RIGHT, 0, 320), 0, 319);   // screen X from raw Y
  int ty = constrain(map(p.x, TS_TOP,  TS_BOTTOM, 0, 240), 0, 239);  // screen Y from raw X
  // No unsolicited serial debug output: host protocol is request/response only.

  if (screen == HOME) {
    if (hit(tx, ty, 12, 80, 296, 44)) {
      currentAlert = "navigation";
      drawSub("NAVIGATION", "Request saved", "Check host dashboard.", C_ACCENT);
    } else if (hit(tx, ty, 12, 134, 296, 44)) {
      currentAlert = "staff";
      drawSub("CALL STAFF", "Request saved", "Check host dashboard.", C_AMBER);
    } else if (hit(tx, ty, 12, 188, 296, 44)) {
      currentAlert = "emergency";
      drawSub("EMERGENCY", "Alert selected", "Seek help immediately.", C_RED);
    }
  } else {                                     // SUB -> Done clears the alert
    if (hit(tx, ty, 12, 188, 296, 44)) { currentAlert = "none"; drawHome(); }
  }
}

// ================= SERIAL =================
void cmdEnv() {
  Serial.print("{");
  if (haveBME) Serial.printf("\"t\":%.2f,\"h\":%.2f,\"p\":%.2f,", bme.readTemperature(), bme.readHumidity(), bme.readPressure()/100.0f);
  else         Serial.print("\"t\":null,\"h\":null,\"p\":null,");
  Serial.printf("\"gas\":%d}\n", analogRead(PIN_MQ135)); // raw ADC, never ppm
}
void cmdThermal() {
  if (!haveAMG) { Serial.println("{\"min\":null,\"max\":null,\"mean\":null,\"grid\":[]}"); return; }
  float px[64];
  if (!amgReadGrid(px)) { Serial.println("{\"min\":null,\"max\":null,\"mean\":null,\"grid\":[]}"); return; }
  float mn = 999, mx = -999, sum = 0;
  for (int i = 0; i < 64; i++) { sum += px[i]; if (px[i] < mn) mn = px[i]; if (px[i] > mx) mx = px[i]; }
  Serial.printf("{\"min\":%.2f,\"max\":%.2f,\"mean\":%.2f,\"grid\":[", mn, mx, sum/64.0f);
  for (int i = 0; i < 64; i++) Serial.printf("%d%s", (int)round(px[i]*100), i < 63 ? "," : "");
  Serial.println("]}");
}
void cmdRF() {
  int n = WiFi.scanNetworks(false, true);
  int strong = 0, top[8], cnt = 0;
  if (n < 0) { Serial.println("{\"n\":null,\"strong\":null,\"top\":[]}"); return; }
  for (int i = 0; i < n; i++) { int r = WiFi.RSSI(i); if (r > -55) strong++; if (cnt < 8) top[cnt++] = r; }
  Serial.printf("{\"n\":%d,\"strong\":%d,\"top\":[", n, strong);
  for (int i = 0; i < cnt; i++) Serial.printf("%d%s", top[i], i < cnt-1 ? "," : "");
  Serial.println("]}");
  WiFi.scanDelete();
}
String buf;
void handle(String line) {
  line.trim();
  if (line == "PING")            { Serial.println("PONG"); return; }
  if (line == "LED ON")          { digitalWrite(PIN_LED_GATE, HIGH); Serial.println("OK"); return; }
  if (line == "LED OFF")         { digitalWrite(PIN_LED_GATE, LOW);  Serial.println("OK"); return; }
  if (line == "ENV")             { cmdEnv();     return; }
  if (line == "THERMAL")         { cmdThermal(); return; }
  if (line == "RF")              { cmdRF();      return; }
  if (line == "EVENT")           { Serial.printf("{\"alert\":\"%s\"}\n", currentAlert.c_str()); return; }
  if (line.startsWith("SERVO ")) { servoDeg(line.substring(6).toInt()); Serial.println("OK"); return; }
  if (line.startsWith("STATUS ")) {
    int a, b, c;
    if (sscanf(line.c_str(), "STATUS %d %d %d", &a, &b, &c) == 3) { setStatus(a, b, c); Serial.println("OK"); }
    else Serial.println("ERR");
    return;
  }
  Serial.println("ERR");
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_GATE, OUTPUT); digitalWrite(PIN_LED_GATE, LOW);
  pinMode(PIN_R, OUTPUT); pinMode(PIN_G, OUTPUT); pinMode(PIN_B, OUTPUT);
  setStatus(0, 0, 1);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ135, ADC_11db);

  Wire.begin(21, 22); delay(50);
  for (int i = 0; i < 3 && !haveBME; i++) { haveBME = bme.begin(0x76) || bme.begin(0x77); if (!haveBME) delay(50); }
  for (int i = 0; i < 3 && !haveAMG; i++) { haveAMG = amgBegin(); if (!haveAMG) delay(50); }

  panServo.setPeriodHertz(50);
  panServo.attach(PIN_SERVO, SERVO_US_MIN, SERVO_US_MAX);
  servoDeg(90);

  WiFi.mode(WIFI_STA); WiFi.disconnect(false); delay(100); // retain Wi-Fi radio for real scans

  tftSPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(); tft.setRotation(1);
  ts.begin(tftSPI); ts.setRotation(0);     // RAW; mapping done manually in handleTouch
  drawHome();

  setStatus(0, 1, 0);
}
void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { handle(buf); buf = ""; }
    else if (ch != '\r') buf += ch;
  }
  handleTouch();
}
