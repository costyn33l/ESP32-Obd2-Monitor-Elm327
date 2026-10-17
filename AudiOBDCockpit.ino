/*
 * AudiOBDCockpit.ino
 * Virtual-cockpit-inspired dashboard fed by a Bluetooth Classic ELM327 adapter.
 *
 * Target : classic ESP32 (ESP32-WROOM-32E) - NOT ESP32-S3
 * Display: 2.8" ILI9341V, landscape 320x240 (tft.setRotation(1))  - TFT_eSPI
 * Touch  : XPT2046 on its own pins via XPT2046_Touchscreen (HSPI)
 * OBD    : BluetoothSerial (SPP master) + ELMduino, in a separate task (ObdLink.cpp)
 *
 * Pages (tap the tabs in the bottom strip):
 *   DRIVE : tachometer, speed, coolant / load / voltage / fuel, throttle
 *   DATA  : all ten PIDs with value, STALE / UNSUP / NO DATA tags
 *   LINK  : Bluetooth / ELM327 / ECU status, PID support, touch read-out, RECONNECT
 *
 * Values that are unavailable are shown as "N/A" - never as 0 and never simulated.
 *
 * NOT COMPILED OR TESTED by the author. See README.txt.
 */

#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include "Config.h"
#include "ObdLink.h"

// ---------------------------------------------------------------------------
// Compile-time guard: wrong TFT_eSPI setup => clear error
// ---------------------------------------------------------------------------
#ifndef ILI9341_DRIVER
#error "TFT_eSPI is not using the ILI9341 driver. Install the provided User_Setup.h (see README.txt)."
#endif
#if (TFT_CS != 15) || (TFT_DC != 2) || (TFT_MOSI != 13) || (TFT_MISO != 12) || (TFT_SCLK != 14)
#error "TFT_eSPI pins do not match this board. Install the provided User_Setup.h (see README.txt)."
#endif
#if !defined(TFT_BL)
#error "TFT_BL (GPIO 21) is not defined. Install the provided User_Setup.h (see README.txt)."
#endif
#if !defined(LOAD_FONT2) || !defined(LOAD_FONT4) || !defined(LOAD_FONT6)
#error "Fonts 2, 4 and 6 must be enabled (LOAD_FONT2/4/6). Install the provided User_Setup.h."
#endif

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI(CFG_TOUCH_SPI_HOST);
#if CFG_TOUCH_USE_IRQ
XPT2046_Touchscreen ts(CFG_TOUCH_CS, CFG_TOUCH_IRQ);
#else
XPT2046_Touchscreen ts(CFG_TOUCH_CS);
#endif

// ---------------------------------------------------------------------------
// Colours (RGB565)
// ---------------------------------------------------------------------------
#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

const uint16_t C_BG      = RGB565(0, 0, 0);
const uint16_t C_WHITE   = RGB565(240, 244, 248);
const uint16_t C_GREY    = RGB565(150, 158, 168);
const uint16_t C_DIM     = RGB565(62, 68, 78);
const uint16_t C_OFF     = RGB565(28, 33, 40);
const uint16_t C_ICE     = RGB565(110, 200, 255);
const uint16_t C_ICE_DIM = RGB565(30, 70, 100);
const uint16_t C_ORANGE  = RGB565(255, 140, 0);
const uint16_t C_ORG_DIM = RGB565(85, 42, 0);
const uint16_t C_RED     = RGB565(235, 25, 25);
const uint16_t C_RED_DIM = RGB565(80, 14, 12);
const uint16_t C_GREEN   = RGB565(40, 215, 110);

// ---------------------------------------------------------------------------
// Layout (320 x 240, landscape)
// ---------------------------------------------------------------------------
const int16_t TOP_H   = 22;
const int16_t STRIP_Y = 216;

// Tachometer
const int16_t TACH_CX = 78;
const int16_t TACH_CY = 114;
const int16_t R_ARC   = 74;
const int16_t R_OUT   = 70;
const int16_t R_IN    = 60;
const int16_t R_TICK_MAJ_IN = 51;
const int16_t R_TICK_MIN_IN = 54;
const int16_t R_TICK_OUT    = 57;
const int16_t R_LABEL = 41;
const float   ANG_START = -135.0f;
const float   ANG_SPAN  = 270.0f;
const float   D2R       = 0.01745329252f;

const int RPM_MAX     = 8000;
const int RPM_REDLINE = 6500;
const int N_SEG       = 64;
const int RPM_PER_SEG = RPM_MAX / N_SEG;      // 125
const int RED_SEG     = RPM_REDLINE / RPM_PER_SEG;
const int RED2_SEG    = 58;

// Speed
const int16_t SPEED_CX = 204;
const int16_t SPEED_Y  = 88;

// Right panel (DRIVE page)
const int16_t DIV_X   = 254;
const int16_t PANEL_X = 260;
const int16_t PANEL_W = 56;
const int16_t BLK_Y[4] = { 28, 74, 120, 166 };
const uint8_t BLK_PID[4]   = { PIDX_COOLANT, PIDX_LOAD, PIDX_VOLTAGE, PIDX_FUEL };
const float   BLK_LO[4]    = { 70.0f, 0.0f, 11.0f, 0.0f };
const float   BLK_HI[4]    = { 110.0f, 100.0f, 15.0f, 100.0f };
const int     BLK_SCALE[4] = { 1, 1, 10, 1 };

// Shift lights
const int     N_LED   = 8;
const int16_t LED_W   = 11;
const int16_t LED_H   = 7;
const int16_t LED_GAP = 4;
const int16_t LED_X0  = 102;
const int16_t LED_Y   = 7;
const float   SHIFT_LED_START = 4800.0f;
const float   SHIFT_LED_STEP  = 240.0f;

// Bottom strip
enum { PAGE_DRIVE = 0, PAGE_DATA = 1, PAGE_LINK = 2, PAGE_COUNT = 3 };
const char *TAB_NAMES[PAGE_COUNT] = { "DRIVE", "DATA", "LINK" };
const int16_t TAB_W = 76;
const int16_t TAB_PITCH = 78;
const int16_t IND_X[3] = { 252, 283, 314 };

// DATA page
const uint8_t CELL_PID[PIDX_COUNT] = { PIDX_RPM, PIDX_SPEED, PIDX_COOLANT, PIDX_LOAD, PIDX_IAT,
                                       PIDX_MAP, PIDX_MAF, PIDX_THROTTLE, PIDX_VOLTAGE, PIDX_FUEL };
const char *CELL_LABEL[PIDX_COUNT] = { "RPM", "SPEED km/h", "COOLANT C", "LOAD %", "INTAKE AIR C",
                                       "MAP kPa", "MAF g/s", "THROTTLE %", "VOLTAGE V", "FUEL %" };
const uint8_t CELL_DEC[PIDX_COUNT] = { 0, 0, 0, 0, 0, 0, 1, 0, 1, 0 };

// LINK page
const int16_t LK_Y0 = 26, LK_DY = 19, LK_LBL_X = 8, LK_VAL_X = 100;
const char *LK_LABELS[7] = { "BLUETOOTH", "ELM327", "ECU / CAR", "TARGET", "PIDS", "LAST ERR", "TOUCH" };
const int16_t BTN_X = 8, BTN_Y = 176, BTN_W = 150, BTN_H = 34;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
struct Seg {
  int16_t x[4];
  int16_t y[4];
};
Seg segs[N_SEG];

uint8_t  page = PAGE_DRIVE;
int      speedPad = 96;
int      rpmPad = 64;
float    dispRpm = 0.0f;

ObdValueState vs[PIDX_COUNT];

// caches: what is currently drawn (sentinel -9999 / 255 = "unknown, redraw")
int      tachLit = 0;
int      shownRpmKey = -9999;
uint8_t  shownRpmSt = 255;
uint32_t lastRpmDraw = 0;
int      shownLeds = -1;
int      shownSpeedKey = -9999;
uint8_t  shownSpeedSt = 255;
int      blkKey[4];
uint8_t  blkSt[4];
int      thrKey = -9999;
uint8_t  thrSt = 255;
int      cellKey[PIDX_COUNT];
uint8_t  cellSt[PIDX_COUNT];
uint8_t  cellTag[PIDX_COUNT];
char     lkShown[7][28];
uint16_t lkColor[7];
uint16_t indCol[3] = { 0, 0, 0 };

char     touchText[24] = "tap to read";
uint32_t btnFlashUntil = 0;
bool     btnLit = false;
uint32_t lastFrame = 0;

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void polar(float r, float angDeg, int16_t &x, int16_t &y) {
  float a = angDeg * D2R;
  x = (int16_t)lroundf(TACH_CX + r * sinf(a));
  y = (int16_t)lroundf(TACH_CY - r * cosf(a));
}

void drawTxt(const char *s, int16_t x, int16_t y, uint8_t font, uint8_t datum,
             uint16_t fg, uint16_t pad) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, C_BG);
  tft.setTextPadding(pad);
  tft.drawString(s, x, y, font);
  tft.setTextPadding(0);
}

void drawNumT(long v, int16_t x, int16_t y, uint8_t font, uint8_t datum,
              uint16_t fg, uint16_t pad) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, C_BG);
  tft.setTextPadding(pad);
  tft.drawNumber(v, x, y, font);
  tft.setTextPadding(0);
}

void drawBar(int16_t x, int16_t y, int16_t w, int16_t h, float frac, uint16_t col) {
  frac = clampf(frac, 0.0f, 1.0f);
  int16_t fw = (int16_t)(w * frac);
  if (fw > 0) tft.fillRect(x, y, fw, h, col);
  if (fw < w) tft.fillRect(x + fw, y, w - fw, h, C_OFF);
}

void drawUnitDegC(int16_t x, int16_t y) {
  tft.drawCircle(x + 1, y + 1, 1, C_GREY);
  drawTxt("C", x + 5, y, 1, TL_DATUM, C_GREY, 0);
}

uint16_t valueColor(ObdValueState st) {
  if (st == OBD_VAL_OK)    return C_WHITE;
  if (st == OBD_VAL_STALE) return C_GREY;
  return C_DIM;
}

// ---------------------------------------------------------------------------
// RPM bar (64 pre-computed segments, only changed ones are repainted)
// ---------------------------------------------------------------------------
void buildSegments() {
  const float per = ANG_SPAN / N_SEG;
  const float gap = 0.7f;
  for (int i = 0; i < N_SEG; i++) {
    float a0 = ANG_START + i * per + gap;
    float a1 = ANG_START + (i + 1) * per - gap;
    polar(R_OUT, a0, segs[i].x[0], segs[i].y[0]);
    polar(R_OUT, a1, segs[i].x[1], segs[i].y[1]);
    polar(R_IN,  a1, segs[i].x[2], segs[i].y[2]);
    polar(R_IN,  a0, segs[i].x[3], segs[i].y[3]);
  }
}

uint16_t litColor(int i) {
  if (i >= RED2_SEG) return C_RED;
  if (i >= RED_SEG)  return C_ORANGE;
  return C_ICE;
}

uint16_t offColor(int i) {
  if (i >= RED2_SEG) return C_RED_DIM;
  if (i >= RED_SEG)  return C_ORG_DIM;
  return C_OFF;
}

void drawSeg(int i, uint16_t c) {
  const Seg &s = segs[i];
  tft.fillTriangle(s.x[0], s.y[0], s.x[1], s.y[1], s.x[2], s.y[2], c);
  tft.fillTriangle(s.x[0], s.y[0], s.x[2], s.y[2], s.x[3], s.y[3], c);
}

void updateTach(int newLit) {
  if (newLit == tachLit) return;
  int oldTip = tachLit - 1;

  if (newLit > tachLit) {
    for (int i = tachLit; i < newLit; i++) drawSeg(i, litColor(i));
  } else {
    for (int i = newLit; i < tachLit; i++) drawSeg(i, offColor(i));
  }
  tachLit = newLit;

  int newTip = newLit - 1;
  if (oldTip >= 0 && oldTip < newLit && oldTip != newTip) drawSeg(oldTip, litColor(oldTip));
  if (newTip >= 0) drawSeg(newTip, (newTip >= RED_SEG) ? litColor(newTip) : C_WHITE);
}

// ---------------------------------------------------------------------------
// Bottom strip (always visible): tabs + three status dots
// ---------------------------------------------------------------------------
void drawStripStatic() {
  tft.drawFastHLine(4, STRIP_Y, 312, C_DIM);
  drawTxt("BT",  236, 225, 1, TL_DATUM, C_GREY, 0);
  drawTxt("ELM", 262, 225, 1, TL_DATUM, C_GREY, 0);
  drawTxt("ECU", 293, 225, 1, TL_DATUM, C_GREY, 0);
}

void drawTabs() {
  tft.fillRect(0, STRIP_Y + 1, 234, 23, C_BG);
  for (int i = 0; i < PAGE_COUNT; i++) {
    int16_t x0 = 4 + i * TAB_PITCH;
    bool act = (i == page);
    if (act) tft.fillRect(x0, STRIP_Y + 1, TAB_W, 2, C_ICE);
    drawTxt(TAB_NAMES[i], x0 + TAB_W / 2, STRIP_Y + 14, 2, MC_DATUM, act ? C_WHITE : C_GREY, 0);
  }
}

void renderIndicators(const ObdSnapshot &s) {
  uint16_t c[3];

  switch (s.link) {
    case OBD_LNK_CONNECTED:    c[0] = C_GREEN;  break;
    case OBD_LNK_CONNECTING:   c[0] = C_ORANGE; break;
    case OBD_LNK_DISCONNECTED: c[0] = C_RED;    break;
    case OBD_LNK_CONFIG_ERROR: c[0] = C_RED;    break;
    default:                   c[0] = C_DIM;    break;
  }
  switch (s.init) {
    case OBD_INI_OK:      c[1] = C_GREEN;  break;
    case OBD_INI_RUNNING: c[1] = C_ORANGE; break;
    case OBD_INI_FAILED:  c[1] = C_RED;    break;
    default:              c[1] = C_DIM;    break;
  }
  switch (s.veh) {
    case OBD_VEH_ONLINE:      c[2] = C_GREEN;  break;
    case OBD_VEH_DISCOVERING: c[2] = C_ORANGE; break;
    case OBD_VEH_NO_RESPONSE: c[2] = C_RED;    break;
    default:                  c[2] = C_DIM;    break;
  }

  for (int i = 0; i < 3; i++) {
    if (c[i] != indCol[i]) {
      indCol[i] = c[i];
      tft.fillCircle(IND_X[i], 229, 3, c[i]);
    }
  }
}

// ---------------------------------------------------------------------------
// DRIVE page - static artwork
// ---------------------------------------------------------------------------
void drawTopDrive() {
  drawTxt("SPORT", 8, 3, 2, TL_DATUM, C_ICE, 0);
  drawTxt("RS", 312, 3, 2, TR_DATUM, C_RED, 0);
  tft.drawFastHLine(4, TOP_H - 1, 312, C_DIM);
}

void drawShiftLights(int lit) {
  for (int i = 0; i < N_LED; i++) {
    int16_t x = LED_X0 + i * (LED_W + LED_GAP);
    uint16_t c = C_OFF;
    if (i < lit) c = (i < 3) ? C_ICE : ((i < 6) ? C_ORANGE : C_RED);
    tft.fillRect(x, LED_Y, LED_W, LED_H, c);
  }
}

void drawTachStatic() {
  int16_t px, py, qx, qy;

  const float redAng = ANG_START + ANG_SPAN * RPM_REDLINE / RPM_MAX;
  polar(R_ARC, ANG_START, px, py);
  for (int a = 1; a <= 270; a++) {
    float ang = ANG_START + a;
    polar(R_ARC, ang, qx, qy);
    tft.drawLine(px, py, qx, qy, (ang > redAng) ? C_RED_DIM : C_ICE_DIM);
    px = qx;
    py = qy;
  }

  for (int i = 0; i < N_SEG; i++) drawSeg(i, offColor(i));

  for (int k = 0; k <= 16; k++) {
    int rpm = k * 500;
    float ang = ANG_START + ANG_SPAN * rpm / RPM_MAX;
    bool major = (k % 2) == 0;
    int16_t x0, y0, x1, y1;
    polar(major ? R_TICK_MAJ_IN : R_TICK_MIN_IN, ang, x0, y0);
    polar(R_TICK_OUT, ang, x1, y1);
    uint16_t c = (rpm >= RPM_REDLINE) ? C_ORANGE : (major ? C_WHITE : C_GREY);
    tft.drawLine(x0, y0, x1, y1, c);
  }

  for (int k = 0; k <= 8; k++) {
    float ang = ANG_START + ANG_SPAN * (k * 1000) / RPM_MAX;
    int16_t x, y;
    polar(R_LABEL, ang, x, y);
    drawNumT(k, x, y, 2, MC_DATUM, (k >= 7) ? C_RED : C_WHITE, 0);
  }

  drawTxt("RPM", TACH_CX, TACH_CY + 20, 1, TC_DATUM, C_GREY, 0);
}

void drawSpeedStatic() {
  tft.drawFastHLine(164, 80, 80, C_ICE_DIM);
  tft.drawFastHLine(164, 164, 80, C_ICE_DIM);
  drawTxt("km/h", SPEED_CX, 142, 2, TC_DATUM, C_ICE, 0);
}

void drawPanelStatic() {
  tft.drawFastVLine(DIV_X, 28, 184, C_DIM);
  const char *names[4] = { "COOLANT", "LOAD", "VOLTAGE", "FUEL" };
  for (int b = 0; b < 4; b++) drawTxt(names[b], PANEL_X, BLK_Y[b], 1, TL_DATUM, C_GREY, 0);
  drawUnitDegC(307, BLK_Y[0]);
  drawTxt("%", 310, BLK_Y[1], 1, TL_DATUM, C_GREY, 0);
  drawTxt("V", 310, BLK_Y[2], 1, TL_DATUM, C_GREY, 0);
  drawTxt("%", 310, BLK_Y[3], 1, TL_DATUM, C_GREY, 0);
  for (int b = 0; b < 4; b++) drawBar(PANEL_X, BLK_Y[b] + 39, PANEL_W, 3, 0.0f, C_ICE);
}

void drawThrottleStatic() {
  drawTxt("THROTTLE", 16, 178, 1, TL_DATUM, C_GREY, 0);
  drawBar(16, 192, 124, 5, 0.0f, C_ICE);
}

void drawDriveStatic() {
  drawTopDrive();
  drawShiftLights(0);
  drawTachStatic();
  drawSpeedStatic();
  drawPanelStatic();
  drawThrottleStatic();
}

// ---------------------------------------------------------------------------
// DRIVE page - dynamic
// ---------------------------------------------------------------------------
void drawRpmText(ObdValueState st, int key) {
  if (st == OBD_VAL_NA) drawTxt("N/A", TACH_CX, TACH_CY - 10, 4, TC_DATUM, C_DIM, rpmPad);
  else drawNumT(key, TACH_CX, TACH_CY - 10, 4, TC_DATUM,
                (st == OBD_VAL_STALE) ? C_GREY : ((key >= RPM_REDLINE) ? C_ORANGE : C_WHITE), rpmPad);
}

void drawSpeed(ObdValueState st, int key, uint8_t prevSt) {
  const bool wasNA = (prevSt == OBD_VAL_NA) || (prevSt == 255);
  const bool isNA = (st == OBD_VAL_NA);
  if (isNA != wasNA || prevSt == 255) tft.fillRect(SPEED_CX - 48, 86, 96, 56, C_BG);

  if (isNA) {
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(C_DIM, C_BG);
    tft.setTextSize(2);
    tft.drawString("N/A", SPEED_CX, SPEED_Y, 4);
    tft.setTextSize(1);
  } else {
    drawNumT(key, SPEED_CX, SPEED_Y, 6, TC_DATUM, valueColor(st), speedPad);
  }
}

uint16_t blockColor(int b, ObdValueState st, float v) {
  if (st == OBD_VAL_STALE) return C_GREY;
  switch (b) {
    case 0:  return (v >= 105.0f) ? C_ORANGE : C_WHITE;
    case 1:  return (v >= 80.0f) ? C_ORANGE : C_WHITE;
    case 2:  return (v < 12.0f || v > 15.0f) ? C_ORANGE : C_WHITE;
    default: return (v <= 15.0f) ? C_ORANGE : C_WHITE;
  }
}

void renderDrive(const ObdSnapshot &s, uint32_t now) {
  // --- RPM bar, shift lights, RPM digits
  ObdValueState rs = vs[PIDX_RPM];
  float rpm = s.value[PIDX_RPM];

  if (rs == OBD_VAL_NA) {
    dispRpm = 0.0f;
  } else {
    float d = rpm - dispRpm;                 // display interpolation of the real reading
    dispRpm += d * 0.35f;
    if (fabsf(d) < 15.0f) dispRpm = rpm;
  }
  int lit = (rs == OBD_VAL_NA) ? 0 : (int)(dispRpm / RPM_PER_SEG + 0.5f);
  lit = constrain(lit, 0, N_SEG);
  updateTach(lit);

  int sl = 0;
  if (rs == OBD_VAL_OK && rpm >= SHIFT_LED_START) {
    sl = (int)((rpm - SHIFT_LED_START) / SHIFT_LED_STEP) + 1;
    if (sl > N_LED) sl = N_LED;
  }
  if (sl != shownLeds) {
    drawShiftLights(sl);
    shownLeds = sl;
  }

  int rkey = (rs == OBD_VAL_NA) ? -1 : ((int)lroundf(rpm / 10.0f)) * 10;
  if ((rkey != shownRpmKey || rs != shownRpmSt) && (now - lastRpmDraw >= 60 || rs != shownRpmSt)) {
    lastRpmDraw = now;
    shownRpmKey = rkey;
    shownRpmSt = rs;
    drawRpmText(rs, rkey);
  }

  // --- Speed
  ObdValueState ss = vs[PIDX_SPEED];
  int skey = (ss == OBD_VAL_NA) ? -1 : (int)lroundf(s.value[PIDX_SPEED]);
  if (skey != shownSpeedKey || ss != shownSpeedSt) {
    drawSpeed(ss, skey, shownSpeedSt);
    shownSpeedKey = skey;
    shownSpeedSt = ss;
  }

  // --- Right-hand blocks
  for (int b = 0; b < 4; b++) {
    uint8_t id = BLK_PID[b];
    ObdValueState st = vs[id];
    float v = s.value[id];
    int key = (st == OBD_VAL_NA) ? -1 : (int)lroundf(v * BLK_SCALE[b]);
    if (key == blkKey[b] && st == blkSt[b]) continue;
    blkKey[b] = key;
    blkSt[b] = st;

    int16_t y = BLK_Y[b] + 10;
    if (st == OBD_VAL_NA) {
      drawTxt("N/A", PANEL_X, y, 4, TL_DATUM, C_DIM, PANEL_W);
      drawBar(PANEL_X, BLK_Y[b] + 39, PANEL_W, 3, 0.0f, C_ICE);
    } else {
      uint16_t col = blockColor(b, st, v);
      if (BLK_SCALE[b] == 10) {
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(col, C_BG);
        tft.setTextPadding(PANEL_W);
        tft.drawFloat(key / 10.0f, 1, PANEL_X, y, 4);
        tft.setTextPadding(0);
      } else {
        drawNumT(key, PANEL_X, y, 4, TL_DATUM, col, PANEL_W);
      }
      float frac = (v - BLK_LO[b]) / (BLK_HI[b] - BLK_LO[b]);
      drawBar(PANEL_X, BLK_Y[b] + 39, PANEL_W, 3, frac, (col == C_ORANGE) ? C_ORANGE : ((st == OBD_VAL_STALE) ? C_DIM : C_ICE));
    }
  }

  // --- Throttle row under the tachometer
  ObdValueState ts2 = vs[PIDX_THROTTLE];
  int tkey = (ts2 == OBD_VAL_NA) ? -1 : (int)lroundf(s.value[PIDX_THROTTLE]);
  if (tkey != thrKey || ts2 != thrSt) {
    thrKey = tkey;
    thrSt = ts2;
    if (ts2 == OBD_VAL_NA) {
      drawTxt("N/A", 140, 176, 2, TR_DATUM, C_DIM, 60);
      drawBar(16, 192, 124, 5, 0.0f, C_ICE);
    } else {
      char buf[8];
      snprintf(buf, sizeof(buf), "%d%%", tkey);
      drawTxt(buf, 140, 176, 2, TR_DATUM, valueColor(ts2), 60);
      drawBar(16, 192, 124, 5, s.value[PIDX_THROTTLE] / 100.0f, (ts2 == OBD_VAL_STALE) ? C_DIM : C_ICE);
    }
  }
}

// ---------------------------------------------------------------------------
// DATA page
// ---------------------------------------------------------------------------
int16_t cellX(int i) { return 6 + (i % 2) * 158; }
int16_t cellY(int i) { return 24 + (i / 2) * 38; }

void drawDataStatic() {
  drawTxt("ENGINE DATA", 8, 3, 2, TL_DATUM, C_ICE, 0);
  drawTxt("OBD-II", 312, 3, 2, TR_DATUM, C_GREY, 0);
  tft.drawFastHLine(4, TOP_H - 1, 312, C_DIM);
  tft.drawFastVLine(158, 24, 188, C_OFF);
  for (int k = 1; k < 5; k++) tft.drawFastHLine(4, 24 + k * 38 - 1, 312, C_OFF);
  for (int i = 0; i < PIDX_COUNT; i++) drawTxt(CELL_LABEL[i], cellX(i), cellY(i) + 2, 1, TL_DATUM, C_GREY, 0);
}

void renderData(const ObdSnapshot &s) {
  for (int i = 0; i < PIDX_COUNT; i++) {
    uint8_t id = CELL_PID[i];
    ObdValueState st = vs[id];
    float v = s.value[id];
    int scale = CELL_DEC[i] ? 10 : 1;
    int key = (st == OBD_VAL_NA) ? -1 : (int)lroundf(v * scale);

    if (key != cellKey[i] || st != cellSt[i]) {
      cellKey[i] = key;
      cellSt[i] = st;
      char buf[16];
      if (st == OBD_VAL_NA) snprintf(buf, sizeof(buf), "N/A");
      else if (CELL_DEC[i]) snprintf(buf, sizeof(buf), "%.1f", key / 10.0f);
      else snprintf(buf, sizeof(buf), "%d", key);
      drawTxt(buf, cellX(i), cellY(i) + 11, 4, TL_DATUM, valueColor(st), 100);
    }

    uint8_t tag;
    if (st == OBD_VAL_OK) tag = 0;
    else if (st == OBD_VAL_STALE) tag = 1;
    else if (s.support[id] == OBD_SUP_NO) tag = 2;
    else tag = 3;

    if (tag != cellTag[i]) {
      cellTag[i] = tag;
      const char *txt = "";
      uint16_t col = C_DIM;
      if (tag == 1) { txt = "STALE"; col = C_ORANGE; }
      else if (tag == 2) { txt = "UNSUP"; col = C_DIM; }
      else if (tag == 3) { txt = "NO DATA"; col = C_DIM; }
      drawTxt(txt, cellX(i) + 150, cellY(i) + 2, 1, TR_DATUM, col, 54);
    }
  }
}

// ---------------------------------------------------------------------------
// LINK page
// ---------------------------------------------------------------------------
void drawLinkButton(bool lit) {
  tft.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, lit ? C_ICE_DIM : C_OFF);
  tft.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, C_ICE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_WHITE, lit ? C_ICE_DIM : C_OFF);
  tft.drawString("RECONNECT", BTN_X + BTN_W / 2, BTN_Y + BTN_H / 2, 2);
  btnLit = lit;
}

void drawLinkStatic() {
  drawTxt("CONNECTION", 8, 3, 2, TL_DATUM, C_ICE, 0);
  drawTxt("OBD-II", 312, 3, 2, TR_DATUM, C_GREY, 0);
  tft.drawFastHLine(4, TOP_H - 1, 312, C_DIM);
  for (int i = 0; i < 7; i++) drawTxt(LK_LABELS[i], LK_LBL_X, LK_Y0 + i * LK_DY, 2, TL_DATUM, C_GREY, 0);
  drawLinkButton(false);
}

void linkLine(int i, const char *text, uint16_t col) {
  if (strcmp(lkShown[i], text) == 0 && lkColor[i] == col) return;
  strncpy(lkShown[i], text, sizeof(lkShown[i]) - 1);
  lkShown[i][sizeof(lkShown[i]) - 1] = 0;
  lkColor[i] = col;
  drawTxt(text, LK_VAL_X, LK_Y0 + i * LK_DY, 2, TL_DATUM, col, 212);
}

void renderLink(const ObdSnapshot &s, uint32_t now) {
  char b[28];
  uint16_t c;

  switch (s.link) {
    case OBD_LNK_CONNECTED:  strcpy(b, "CONNECTED"); c = C_GREEN; break;
    case OBD_LNK_CONNECTING: strcpy(b, "CONNECTING..."); c = C_ORANGE; break;
    case OBD_LNK_DISCONNECTED:
      if (s.nextRetryAt > now) snprintf(b, sizeof(b), "LOST  retry %lus", (unsigned long)((s.nextRetryAt - now + 999) / 1000));
      else strcpy(b, "DISCONNECTED");
      c = C_RED;
      break;
    case OBD_LNK_CONFIG_ERROR: strcpy(b, "BAD MAC FORMAT"); c = C_RED; break;
    default: strcpy(b, "IDLE"); c = C_GREY; break;
  }
  linkLine(0, b, c);

  switch (s.init) {
    case OBD_INI_OK:      strcpy(b, "READY"); c = C_GREEN; break;
    case OBD_INI_RUNNING: strcpy(b, "INITIALIZING"); c = C_ORANGE; break;
    case OBD_INI_FAILED:  strcpy(b, "INIT FAILED"); c = C_RED; break;
    default:              strcpy(b, "-"); c = C_GREY; break;
  }
  linkLine(1, b, c);

  switch (s.veh) {
    case OBD_VEH_ONLINE:      strcpy(b, "ONLINE"); c = C_GREEN; break;
    case OBD_VEH_DISCOVERING: strcpy(b, "SCANNING PIDS"); c = C_ORANGE; break;
    case OBD_VEH_NO_RESPONSE: strcpy(b, "NO RESPONSE"); c = C_RED; break;
    default:                  strcpy(b, "-"); c = C_GREY; break;
  }
  linkLine(2, b, c);

  snprintf(b, sizeof(b), "%s", obdTargetText());
  linkLine(3, b, obdTargetIsMac() ? C_WHITE : C_ORANGE);

  int yes = 0, no = 0, unk = 0;
  for (int i = 0; i < PIDX_COUNT; i++) {
    if (s.support[i] == OBD_SUP_YES) yes++;
    else if (s.support[i] == OBD_SUP_NO) no++;
    else unk++;
  }
  snprintf(b, sizeof(b), "%d yes  %d no  %d ?", yes, no, unk);
  linkLine(4, b, C_WHITE);

  snprintf(b, sizeof(b), "E%d  ok %lu err %lu", (int)s.lastError, (unsigned long)s.okCount, (unsigned long)s.errCount);
  linkLine(5, b, (s.lastError == 0) ? C_WHITE : C_ORANGE);

  linkLine(6, touchText, C_WHITE);

  if (btnLit && now >= btnFlashUntil) drawLinkButton(false);
}

// ---------------------------------------------------------------------------
// Page management
// ---------------------------------------------------------------------------
void resetCaches() {
  tachLit = 0;
  shownRpmKey = -9999;
  shownRpmSt = 255;
  shownLeds = -1;
  shownSpeedKey = -9999;
  shownSpeedSt = 255;
  thrKey = -9999;
  thrSt = 255;
  for (int i = 0; i < 4; i++) { blkKey[i] = -9999; blkSt[i] = 255; }
  for (int i = 0; i < PIDX_COUNT; i++) { cellKey[i] = -9999; cellSt[i] = 255; cellTag[i] = 255; }
  for (int i = 0; i < 7; i++) { lkShown[i][0] = 0; lkColor[i] = 0; }
  dispRpm = 0.0f;
}

void enterPage(uint8_t p) {
  page = p;
  tft.fillRect(0, 0, 320, STRIP_Y, C_BG);
  resetCaches();
  switch (p) {
    case PAGE_DRIVE: drawDriveStatic(); break;
    case PAGE_DATA:  drawDataStatic();  break;
    default:         drawLinkStatic();  break;
  }
  drawTabs();
}

// ---------------------------------------------------------------------------
// Touch
// ---------------------------------------------------------------------------
void mapTouch(int rx, int ry, int &sx, int &sy) {
#if CFG_TOUCH_SWAP_XY
  int t = rx; rx = ry; ry = t;
#endif
  long x = map(rx, CFG_TOUCH_X_MIN, CFG_TOUCH_X_MAX, 0, 319);
  long y = map(ry, CFG_TOUCH_Y_MIN, CFG_TOUCH_Y_MAX, 0, 239);
  sx = (int)constrain(x, 0, 319);
  sy = (int)constrain(y, 0, 239);
}

void onTap(int x, int y, uint32_t now) {
  if (y >= STRIP_Y) {
    for (int i = 0; i < PAGE_COUNT; i++) {
      int x0 = 4 + i * TAB_PITCH;
      if (x >= x0 && x < x0 + TAB_W + 2) {
        if (i != page) enterPage(i);
        return;
      }
    }
    return;
  }
  if (page == PAGE_LINK && x >= BTN_X && x < BTN_X + BTN_W && y >= BTN_Y && y < BTN_Y + BTN_H) {
    obdRequestReconnect();
    drawLinkButton(true);
    btnFlashUntil = now + 300;
  }
}

void handleTouch(uint32_t now) {
  static bool wasDown = false;
  static uint32_t lastPoll = 0, lastTap = 0;

  if (now - lastPoll < 25) return;
  lastPoll = now;

#if CFG_TOUCH_USE_IRQ
  bool down = ts.tirqTouched() && ts.touched();
#else
  bool down = ts.touched();
#endif
  if (!down) {
    wasDown = false;
    return;
  }

  TS_Point p = ts.getPoint();
  int sx, sy;
  mapTouch(p.x, p.y, sx, sy);
  snprintf(touchText, sizeof(touchText), "%d,%d > %d,%d", (int)p.x, (int)p.y, sx, sy);

  if (wasDown || (now - lastTap) < 250) {
    wasDown = true;
    return;
  }
  wasDown = true;
  lastTap = now;
  onTap(sx, sy, now);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                      // only used for ELMduino debug output

  // Working display bring-up sequence - do not change.
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  delay(100);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);

  // Touch on its own pins / SPI peripheral (TFT_eSPI keeps VSPI).
  touchSPI.begin(CFG_TOUCH_SCLK, CFG_TOUCH_MISO, CFG_TOUCH_MOSI, CFG_TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  buildSegments();
  speedPad = tft.textWidth("188", 6);
  if (speedPad > 96) speedPad = 96;
  rpmPad = tft.textWidth("8888", 4);
  if (rpmPad > 66) rpmPad = 66;

  drawStripStatic();
  enterPage(PAGE_DRIVE);

  obdBegin();                                // Bluetooth + ELMduino run in their own task
}

void loop() {
  uint32_t now = millis();

  handleTouch(now);

  if (now - lastFrame >= 30) {
    lastFrame = now;

    ObdSnapshot s;
    obdGetSnapshot(s);
    for (int i = 0; i < PIDX_COUNT; i++) vs[i] = obdValueState(s, i, now);

    switch (page) {
      case PAGE_DRIVE: renderDrive(s, now); break;
      case PAGE_DATA:  renderData(s);       break;
      default:         renderLink(s, now);  break;
    }
    renderIndicators(s);
  }

  yield();
}
