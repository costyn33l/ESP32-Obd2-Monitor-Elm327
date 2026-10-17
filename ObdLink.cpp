// ObdLink.cpp - Bluetooth Classic (SPP master) + ELMduino, non-blocking for the UI.
//
// NOT COMPILED OR TESTED by the author. ELMduino calls follow the library's
// documented API (begin, nb_rx_state, rpm(), kph(), engineCoolantTemp(),
// engineLoad(), intakeAirTemp(), manifoldPressure(), mafRate(), throttle(),
// ctrlModVoltage(), fuelLevel(), supportedPIDs_xx_xx()). If your installed
// version differs, check the signatures in ELMduino.h.

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ObdLink.h"
#include "Config.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Select an ESP32 (classic) board, not an S3/C3 board.
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the classic ESP32 chip.
#endif

BluetoothSerial SerialBT;
ELM327 myELM327;

// ---------------------------------------------------------------------------
//  Shared snapshot
// ---------------------------------------------------------------------------
static ObdSnapshot       g_snap;
static SemaphoreHandle_t g_mtx = NULL;
static volatile bool     g_reconnectReq = false;

static void lock()   { xSemaphoreTake(g_mtx, portMAX_DELAY); }
static void unlock() { xSemaphoreGive(g_mtx); }

// ---------------------------------------------------------------------------
//  Task state machine
// ---------------------------------------------------------------------------
enum TaskState : uint8_t { TS_WAIT_RETRY, TS_CONNECT, TS_INIT_ELM, TS_DISCOVER, TS_POLL };

static TaskState g_ts = TS_CONNECT;
static uint32_t  g_retryAt = 0;
static uint32_t  g_retryDelay = CFG_RETRY_MIN_MS;
static uint8_t   g_initTries = 0;
static uint8_t   g_connFails = 0;

// discovery
static bool      g_discActive = false;
static uint8_t   g_discRange = 0;
static uint32_t  g_discReqStart = 0;
static uint32_t  g_discWaitUntil = 0;
static uint8_t   g_discRounds = 0;
static uint8_t   g_rangeResult[3];     // 0 pending, 1 ok, 2 no data, 3 error

// polling
static bool      g_reqActive = false;
static uint8_t   g_curPid = 255;
static uint32_t  g_reqStart = 0;
static uint8_t   g_slot = 0;
static uint8_t   g_noData[PIDX_COUNT];
static uint8_t   g_adapterFail = 0;
static uint16_t  g_noDataStreak = 0;

// Weighted round-robin: fast-changing PIDs appear several times per cycle.
// Exactly ONE PID request is in flight at any time.
static const uint8_t SCHEDULE[] = {
  PIDX_RPM, PIDX_SPEED, PIDX_THROTTLE, PIDX_LOAD,
  PIDX_RPM, PIDX_SPEED, PIDX_MAP,      PIDX_MAF,
  PIDX_RPM, PIDX_SPEED, PIDX_COOLANT,  PIDX_IAT,
  PIDX_RPM, PIDX_SPEED, PIDX_VOLTAGE,  PIDX_FUEL
};
static const uint8_t SCHED_LEN = sizeof(SCHEDULE);

// Standard Mode 01 PID numbers in enum order
static const uint8_t PID_NUM[PIDX_COUNT] = {
  0x0C,  // RPM
  0x0D,  // speed
  0x05,  // coolant temperature
  0x04,  // calculated engine load
  0x0F,  // intake air temperature
  0x0B,  // intake manifold absolute pressure
  0x10,  // MAF air flow
  0x11,  // throttle position
  0x42,  // control module voltage
  0x2F   // fuel tank level
};

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
static bool parseMac(const char *s, uint8_t out[6]) {
  unsigned int b[6];
  if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
  return true;
}

static bool isSlowPid(uint8_t p) {
  return p == PIDX_COOLANT || p == PIDX_IAT || p == PIDX_VOLTAGE || p == PIDX_FUEL;
}

static void btSetPin() {
#if CFG_USE_PIN
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    SerialBT.setPin(CFG_BT_PIN, strlen(CFG_BT_PIN));   // core 3.x signature
  #else
    SerialBT.setPin(CFG_BT_PIN);                       // core 2.x signature
  #endif
#endif
}

static void removeAllBonds() {
  int count = esp_bt_gap_get_bond_device_num();
  if (count <= 0) return;
  if (count > 10) count = 10;
  esp_bd_addr_t list[10];
  if (esp_bt_gap_get_bond_device_list(&count, list) == ESP_OK) {
    for (int i = 0; i < count; i++) esp_bt_gap_remove_bond_device(list[i]);
  }
}

static void obdBtBegin() {
  SerialBT.begin(String(CFG_LOCAL_BT_NAME), true);     // true = master / client
  btSetPin();
#if CFG_CLEAR_BONDS_ON_BOOT
  removeAllBonds();
#endif
}

static void obdBtRestart() {
  SerialBT.end();
  vTaskDelay(pdMS_TO_TICKS(300));
  obdBtBegin();
}

static void scheduleRetry() {
  uint32_t now = millis();
  g_retryAt = now + g_retryDelay;
  lock();
  g_snap.nextRetryAt = g_retryAt;
  unlock();
  g_retryDelay *= 2;
  if (g_retryDelay > CFG_RETRY_MAX_MS) g_retryDelay = CFG_RETRY_MAX_MS;
  g_ts = TS_WAIT_RETRY;
}

static void resetPollState() {
  g_slot = 0;
  g_reqActive = false;
  g_curPid = 255;
  g_adapterFail = 0;
  g_noDataStreak = 0;
  for (uint8_t i = 0; i < PIDX_COUNT; i++) g_noData[i] = 0;
}

static void onLinkLost() {
  uint32_t t = millis();
  if (t == 0) t = 1;
  lock();
  if (g_snap.link == OBD_LNK_CONNECTED && g_snap.linkLostAt == 0) g_snap.linkLostAt = t;
  g_snap.link = OBD_LNK_DISCONNECTED;
  g_snap.init = OBD_INI_IDLE;
  g_snap.veh  = OBD_VEH_UNKNOWN;
  unlock();
  g_reqActive = false;
  g_discActive = false;
  if (SerialBT.connected(0)) SerialBT.disconnect();
  scheduleRetry();
}

static void forceReconnect() {
  onLinkLost();
  g_retryDelay = CFG_RETRY_MIN_MS;
  g_retryAt = 0;
  g_connFails = 0;
  g_ts = TS_CONNECT;
}

// ---------------------------------------------------------------------------
//  1) Bluetooth connect (blocking inside this task only)
// ---------------------------------------------------------------------------
static void doConnect() {
  uint8_t mac[6];
  const bool macSet = strlen(CFG_BT_MAC) > 0;
  const bool macOk  = macSet && parseMac(CFG_BT_MAC, mac);

  if (macSet && !macOk) {
    lock();
    g_snap.link = OBD_LNK_CONFIG_ERROR;
    unlock();
    g_retryAt = millis() + 3600000UL;      // wait for a manual reconnect request
    g_ts = TS_WAIT_RETRY;
    return;
  }

  lock();
  g_snap.link = OBD_LNK_CONNECTING;
  g_snap.connectAttempts++;
  unlock();

  if (SerialBT.connected(0)) SerialBT.disconnect();

  bool ok;
  if (macOk) ok = SerialBT.connect(mac, CFG_BT_CHANNEL);
  else       ok = SerialBT.connect(String(CFG_BT_NAME));
  if (ok && !SerialBT.connected(500)) ok = false;

  if (ok) {
    while (SerialBT.available()) SerialBT.read();      // flush stale bytes
    lock();
    g_snap.link = OBD_LNK_CONNECTED;
    g_snap.linkLostAt = 0;
    g_snap.init = OBD_INI_IDLE;
    g_snap.veh = OBD_VEH_UNKNOWN;
    unlock();
    g_initTries = 0;
    g_ts = TS_INIT_ELM;
  } else {
    if (SerialBT.connected(0)) SerialBT.disconnect();
    lock();
    g_snap.link = OBD_LNK_DISCONNECTED;
    unlock();
    g_connFails++;
#if CFG_BT_RESTART_AFTER_FAILS > 0
    if (g_connFails >= CFG_BT_RESTART_AFTER_FAILS) {
      g_connFails = 0;
      obdBtRestart();
    }
#endif
    scheduleRetry();
  }
}

// ---------------------------------------------------------------------------
//  2) ELM327 initialisation
// ---------------------------------------------------------------------------
static void doInit() {
  lock();
  g_snap.init = OBD_INI_RUNNING;
  g_snap.veh = OBD_VEH_UNKNOWN;
  unlock();

  bool ok = myELM327.begin(SerialBT, CFG_ELM_DEBUG != 0, CFG_ELM_TIMEOUT_MS, CFG_ELM_PROTOCOL);

  if (!SerialBT.connected(0)) {
    onLinkLost();
    return;
  }

  if (ok) {
    lock();
    g_snap.init = OBD_INI_OK;
    unlock();
    g_retryDelay = CFG_RETRY_MIN_MS;
    g_connFails = 0;
    g_discActive = false;
    g_discWaitUntil = 0;
    g_ts = TS_DISCOVER;
  } else {
    lock();
    g_snap.init = OBD_INI_FAILED;
    unlock();
    g_initTries++;
    if (g_initTries < CFG_ELM_INIT_TRIES) {
      vTaskDelay(pdMS_TO_TICKS(500));                    // try begin() again on the same link
    } else {
      onLinkLost();                                      // drop link, back-off, reconnect
    }
  }
}

// ---------------------------------------------------------------------------
//  3) Supported-PID discovery (vehicle layer)
//  The ELMduino supportedPIDs_xx_xx() value passes through a float inside the
//  library, so the lowest ~8 bits can be inexact. Every PID used here sits in
//  bits 15..30, which are unaffected. Bit 0 ("next range supported") is NOT
//  used; all three ranges are simply queried.
// ---------------------------------------------------------------------------
static void applyRange(uint8_t range, uint32_t mask) {
  lock();
  for (uint8_t id = 0; id < PIDX_COUNT; id++) {
    uint8_t num = PID_NUM[id];
    uint8_t r = (num - 1) / 32;
    if (r != range) continue;
#if CFG_TRUST_SUPPORT_MASK
    uint8_t off = num - 32 * r;                          // 1..32
    uint8_t bit = 32 - off;
    g_snap.support[id] = ((mask >> bit) & 1UL) ? OBD_SUP_YES : OBD_SUP_NO;
#else
    g_snap.support[id] = OBD_SUP_UNKNOWN;
#endif
  }
  unlock();
}

static void finishDiscovery() {
  g_discActive = false;
  if (g_rangeResult[0] == 1) {
    lock();
    g_snap.veh = OBD_VEH_ONLINE;
    unlock();
    g_discRounds = 0;
    resetPollState();
    g_ts = TS_POLL;
    return;
  }

  g_discRounds++;
  lock();
  g_snap.veh = OBD_VEH_NO_RESPONSE;
  unlock();

  if (g_discRounds >= CFG_DISCOVERY_ROUNDS_BEFORE_PROBE) {
    // ECU never answered the support query: probe each PID directly instead.
    lock();
    for (uint8_t id = 0; id < PIDX_COUNT; id++) g_snap.support[id] = OBD_SUP_UNKNOWN;
    g_snap.veh = OBD_VEH_UNKNOWN;
    unlock();
    g_discRounds = 0;
    resetPollState();
    g_ts = TS_POLL;
  } else {
    g_discWaitUntil = millis() + CFG_DISCOVERY_RETRY_MS;
  }
}

static void doDiscover() {
  uint32_t now = millis();

  if (!g_discActive) {
    if (now < g_discWaitUntil) return;
    g_discActive = true;
    g_discRange = 0;
    g_discReqStart = now;
    for (uint8_t r = 0; r < 3; r++) g_rangeResult[r] = 0;
    lock();
    g_snap.veh = OBD_VEH_DISCOVERING;
    for (uint8_t id = 0; id < PIDX_COUNT; id++) g_snap.support[id] = OBD_SUP_UNKNOWN;
    unlock();
  }

  uint32_t mask = 0;
  switch (g_discRange) {
    case 0:  mask = myELM327.supportedPIDs_1_20();  break;
    case 1:  mask = myELM327.supportedPIDs_21_40(); break;
    default: mask = myELM327.supportedPIDs_41_60(); break;
  }
  int8_t st = myELM327.nb_rx_state;

  if (st == ELM_GETTING_MSG) {
    if (millis() - g_discReqStart > (uint32_t)CFG_ELM_TIMEOUT_MS + 2000UL) {
      g_discActive = false;                              // adapter hung: re-initialise it
      g_initTries = 0;
      g_ts = TS_INIT_ELM;
    }
    return;
  }

  if (st == ELM_SUCCESS) {
    g_rangeResult[g_discRange] = 1;
    applyRange(g_discRange, mask);
  } else if (st == ELM_NO_DATA) {
    g_rangeResult[g_discRange] = 2;
    applyRange(g_discRange, 0);
  } else {
    g_rangeResult[g_discRange] = 3;
    lock();
    g_snap.lastError = st;
    g_snap.errCount++;
    unlock();
#if CFG_ELM_DEBUG
    myELM327.printError();
#endif
  }

  g_discRange++;
  g_discReqStart = millis();
  if (g_discRange >= 3) finishDiscovery();
}

// ---------------------------------------------------------------------------
//  4) PID polling - one request at a time
// ---------------------------------------------------------------------------
static float callPid(uint8_t id) {
  switch (id) {
    case PIDX_RPM:      return myELM327.rpm();
    case PIDX_SPEED:    return (float)myELM327.kph();
    case PIDX_COOLANT:  return myELM327.engineCoolantTemp();
    case PIDX_LOAD:     return myELM327.engineLoad();
    case PIDX_IAT:      return myELM327.intakeAirTemp();
    case PIDX_MAP:      return (float)myELM327.manifoldPressure();
    case PIDX_MAF:      return myELM327.mafRate();
    case PIDX_THROTTLE: return myELM327.throttle();
    case PIDX_VOLTAGE:  return myELM327.ctrlModVoltage();
    case PIDX_FUEL:     return myELM327.fuelLevel();
    default:            return 0.0f;
  }
}

static void doPoll() {
  if (!g_reqActive) {
    g_curPid = 255;
    for (uint8_t n = 0; n < SCHED_LEN; n++) {
      uint8_t id = SCHEDULE[g_slot];
      g_slot = (g_slot + 1) % SCHED_LEN;
      if (g_snap.support[id] != OBD_SUP_NO) {            // only this task writes 'support'
        g_curPid = id;
        break;
      }
    }
    if (g_curPid == 255) {                               // nothing available: look again later
      lock();
      g_snap.veh = OBD_VEH_NO_RESPONSE;
      unlock();
      g_discActive = false;
      g_discWaitUntil = millis() + CFG_DISCOVERY_RETRY_MS;
      g_ts = TS_DISCOVER;
      return;
    }
    g_reqActive = true;
    g_reqStart = millis();
  }

  float v = callPid(g_curPid);
  int8_t st = myELM327.nb_rx_state;

  if (st == ELM_GETTING_MSG) {
    if (millis() - g_reqStart > (uint32_t)CFG_ELM_TIMEOUT_MS + 2000UL) {
      g_reqActive = false;                               // adapter hung: re-initialise it
      lock();
      g_snap.lastError = st;
      g_snap.errCount++;
      unlock();
      g_initTries = 0;
      g_ts = TS_INIT_ELM;
    }
    return;
  }

  g_reqActive = false;
  const uint8_t id = g_curPid;

  if (st == ELM_SUCCESS) {
    lock();
    g_snap.value[id] = v;
    g_snap.valid[id] = true;
    g_snap.stamp[id] = millis();
    g_snap.support[id] = OBD_SUP_YES;
    g_snap.veh = OBD_VEH_ONLINE;
    g_snap.lastError = 0;
    g_snap.okCount++;
    unlock();
    g_noData[id] = 0;
    g_adapterFail = 0;
    g_noDataStreak = 0;
  } else if (st == ELM_NO_DATA) {
    g_noData[id]++;
    g_noDataStreak++;
    lock();
    g_snap.lastError = st;
    g_snap.errCount++;
    if (g_snap.support[id] == OBD_SUP_UNKNOWN && g_noData[id] >= 3) {
      g_snap.support[id] = OBD_SUP_NO;                   // probed and not answered
      g_snap.valid[id] = false;
    } else if (g_snap.support[id] == OBD_SUP_YES && g_noData[id] >= 5) {
      g_snap.valid[id] = false;                          // advertised but not delivering
    }
    unlock();
    if (g_noDataStreak >= CFG_NODATA_STREAK_LIMIT) {     // ECU asleep / ignition off
      lock();
      g_snap.veh = OBD_VEH_NO_RESPONSE;
      unlock();
      g_noDataStreak = 0;
      g_discActive = false;
      g_discWaitUntil = millis() + CFG_DISCOVERY_RETRY_MS;
      g_ts = TS_DISCOVER;
    }
  } else {
    g_adapterFail++;
    lock();
    g_snap.lastError = st;
    g_snap.errCount++;
    unlock();
#if CFG_ELM_DEBUG
    myELM327.printError();
#endif
    if (g_adapterFail >= 5) {                            // adapter not behaving: re-init it
      g_adapterFail = 0;
      g_initTries = 0;
      g_ts = TS_INIT_ELM;
    }
  }
}

// ---------------------------------------------------------------------------
//  Task
// ---------------------------------------------------------------------------
static void obdTask(void *) {
  obdBtBegin();

  for (;;) {
    uint32_t now = millis();

    if (g_reconnectReq) {
      g_reconnectReq = false;
      forceReconnect();
    }

    if ((g_ts == TS_INIT_ELM || g_ts == TS_DISCOVER || g_ts == TS_POLL) && !SerialBT.connected(0)) {
      onLinkLost();
    }

    switch (g_ts) {
      case TS_WAIT_RETRY: if (now >= g_retryAt) g_ts = TS_CONNECT; break;
      case TS_CONNECT:    doConnect();  break;
      case TS_INIT_ELM:   doInit();     break;
      case TS_DISCOVER:   doDiscover(); break;
      case TS_POLL:       doPoll();     break;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------
void obdBegin() {
  g_mtx = xSemaphoreCreateMutex();
  memset(&g_snap, 0, sizeof(g_snap));
  g_ts = TS_CONNECT;
  xTaskCreatePinnedToCore(obdTask, "obd", 10240, NULL, 1, NULL, 0);
}

void obdGetSnapshot(ObdSnapshot &out) {
  if (g_mtx == NULL) { memset(&out, 0, sizeof(out)); return; }
  lock();
  out = g_snap;
  unlock();
}

void obdRequestReconnect() { g_reconnectReq = true; }

ObdValueState obdValueState(const ObdSnapshot &s, uint8_t pid, uint32_t nowMs) {
  if (pid >= PIDX_COUNT || !s.valid[pid]) return OBD_VAL_NA;

  if (s.link != OBD_LNK_CONNECTED) {
    // Link down: keep the last reading visible but stale for a short time only.
    if (s.linkLostAt == 0 || (nowMs - s.linkLostAt) > CFG_DISCONNECT_HOLD_MS) return OBD_VAL_NA;
    return OBD_VAL_STALE;
  }

  const bool slow = isSlowPid(pid);
  const uint32_t fresh  = slow ? CFG_FRESH_SLOW_MS  : CFG_FRESH_FAST_MS;
  const uint32_t maxAge = slow ? CFG_MAXAGE_SLOW_MS : CFG_MAXAGE_FAST_MS;
  const uint32_t age = nowMs - s.stamp[pid];

  if (age > maxAge) return OBD_VAL_NA;
  if (age > fresh)  return OBD_VAL_STALE;
  return OBD_VAL_OK;
}

bool obdTargetIsMac() { return strlen(CFG_BT_MAC) > 0; }

const char *obdTargetText() {
  return strlen(CFG_BT_MAC) ? CFG_BT_MAC : "NAME " CFG_BT_NAME;
}
