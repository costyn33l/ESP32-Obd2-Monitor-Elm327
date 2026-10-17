/*
 * ObdConnectionTest.ino
 * Serial-monitor tool (115200 baud, newline). No display needed.
 *
 *   1. SCAN  : lists nearby Bluetooth Classic devices with their MAC addresses
 *   2. CONNECT: connects to the ELM327 by MAC (or by name), runs ELMduino begin()
 *   3. POLL  : reads the dashboard PIDs one at a time and prints value or error
 *
 * Commands (type, press Enter):
 *   h            help
 *   s            scan for devices (about 12 s) -> note your adapter's MAC
 *   m AA:BB:CC:DD:EE:FF   set the MAC to use for this session
 *   c            connect + initialise ELM327 + print supported-PID masks
 *   p            start / stop polling the PIDs
 *   d            disconnect
 *   u            remove ALL stored Bluetooth pairings (fixes stale-pairing problems)
 *
 * Bluetooth SPP has no baud rate, so there is nothing like 38400 / 115200 to
 * select here. Tunables are the PIN, the SPP channel and the ELM timeout below.
 *
 * NOT COMPILED OR TESTED by the author.
 */

#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "esp_gap_bt_api.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Select an ESP32 (classic) board, not an S3/C3 board.
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the classic ESP32 chip.
#endif

// ----------------------------- settings ------------------------------------
#define TEST_BT_MAC        ""          // optional; or set at runtime with:  m AA:BB:CC:DD:EE:FF
#define TEST_BT_NAME       "OBDII"     // used only if no MAC is set
#define TEST_BT_CHANNEL    0           // 0 = auto; try 1 if connect fails
#define TEST_USE_PIN       1
#define TEST_BT_PIN        "1234"      // common clone PINs: 1234, 0000, 6789
#define TEST_ELM_TIMEOUT   2000
#define TEST_ELM_PROTOCOL  '0'
#define TEST_SCAN_MS       12000
// ---------------------------------------------------------------------------

BluetoothSerial SerialBT;
ELM327 myELM327;

uint8_t  targetMac[6];
bool     haveMac = false;
bool     polling = false;
bool     reqActive = false;
uint8_t  pidIdx = 0;
uint32_t reqStart = 0;

const uint8_t NUM_PIDS = 10;
const char *PID_NAME[NUM_PIDS] = { "RPM", "SPEED km/h", "COOLANT C", "LOAD %", "INTAKE AIR C",
                                   "MAP kPa", "MAF g/s", "THROTTLE %", "CTRL MODULE V", "FUEL %" };

bool parseMac(const char *s, uint8_t out[6]) {
  unsigned int b[6];
  if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
  return true;
}

void setPin() {
#if TEST_USE_PIN
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    SerialBT.setPin(TEST_BT_PIN, strlen(TEST_BT_PIN));
  #else
    SerialBT.setPin(TEST_BT_PIN);
  #endif
#endif
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== OBD connection test ==="));
  Serial.println(F("h help | s scan | m <MAC> set MAC | c connect+init | p poll on/off | d disconnect | u unpair all"));
  Serial.println(F("Tip: switch the phone's Bluetooth off; ignition ON (engine running for RPM/speed)."));
  Serial.println();
}

void doScan() {
  Serial.println(F("Scanning (about 12 s)... keep the adapter powered (in the OBD socket, ignition ON)."));
  BTScanResults *r = SerialBT.discover(TEST_SCAN_MS);
  if (r) {
    r->dump(&Serial);
    Serial.println(F("Copy the address of your ELM327 (often named OBDII, V-LINK, OBDLink ...)."));
  } else {
    Serial.println(F("Scan failed. If you connected earlier, reset the board and scan again."));
  }
}

void doUnpair() {
  int count = esp_bt_gap_get_bond_device_num();
  if (count <= 0) { Serial.println(F("No stored pairings.")); return; }
  if (count > 10) count = 10;
  esp_bd_addr_t list[10];
  if (esp_bt_gap_get_bond_device_list(&count, list) == ESP_OK) {
    for (int i = 0; i < count; i++) esp_bt_gap_remove_bond_device(list[i]);
    Serial.printf("Removed %d pairing(s).\n", count);
  }
}

uint32_t readMask(int range, bool &ok) {
  uint32_t t0 = millis();
  uint32_t m = 0;
  ok = false;
  while (millis() - t0 < 6000) {
    switch (range) {
      case 0:  m = myELM327.supportedPIDs_1_20();  break;
      case 1:  m = myELM327.supportedPIDs_21_40(); break;
      default: m = myELM327.supportedPIDs_41_60(); break;
    }
    if (myELM327.nb_rx_state == ELM_SUCCESS) { ok = true; return m; }
    if (myELM327.nb_rx_state != ELM_GETTING_MSG) { myELM327.printError(); return 0; }
    delay(2);
  }
  Serial.println(F("  (timeout)"));
  return 0;
}

void doConnect() {
  polling = false;
  reqActive = false;
  if (SerialBT.connected(0)) SerialBT.disconnect();

  bool ok;
  if (haveMac) {
    Serial.println(F("Connecting by MAC..."));
    ok = SerialBT.connect(targetMac, TEST_BT_CHANNEL);
  } else {
    Serial.print(F("No MAC set - connecting by name: "));
    Serial.println(TEST_BT_NAME);
    ok = SerialBT.connect(String(TEST_BT_NAME));
  }
  if (!ok || !SerialBT.connected(500)) {
    Serial.println(F("BLUETOOTH CONNECT FAILED. Try: u (unpair), other PIN, TEST_BT_CHANNEL 1, phone BT off."));
    return;
  }
  Serial.println(F("Bluetooth link OK."));
  while (SerialBT.available()) SerialBT.read();

  Serial.println(F("Initialising ELM327 (ELMduino begin)..."));
  if (!myELM327.begin(SerialBT, true, TEST_ELM_TIMEOUT, TEST_ELM_PROTOCOL)) {
    Serial.println(F("ELM327 INIT FAILED. Adapter reachable but not initialised - ignition ON? Try again."));
    return;
  }
  Serial.println(F("ELM327 INIT OK."));

  const char *rname[3] = { "01-20", "21-40", "41-60" };
  for (int r = 0; r < 3; r++) {
    bool mok;
    uint32_t m = readMask(r, mok);
    if (!mok) { Serial.printf("PIDs %s: no answer\n", rname[r]); continue; }
    Serial.printf("PIDs %s: mask 0x%08lX  supported:", rname[r], (unsigned long)m);
    for (int off = 1; off <= 31; off++) {                 // bit 0 (next-range flag) skipped
      if ((m >> (32 - off)) & 1UL) Serial.printf(" %02X", r * 32 + off);
    }
    Serial.println();
  }
  Serial.println(F("(ELMduino returns the mask via a float; the lowest ~8 bits, PIDs 19-20, can be inexact.)"));
  Serial.println(F("Type p to poll the dashboard PIDs."));
}

float callPid(uint8_t id) {
  switch (id) {
    case 0: return myELM327.rpm();
    case 1: return (float)myELM327.kph();
    case 2: return myELM327.engineCoolantTemp();
    case 3: return myELM327.engineLoad();
    case 4: return myELM327.intakeAirTemp();
    case 5: return (float)myELM327.manifoldPressure();
    case 6: return myELM327.mafRate();
    case 7: return myELM327.throttle();
    case 8: return myELM327.ctrlModVoltage();
    default: return myELM327.fuelLevel();
  }
}

void pollStep() {
  if (!SerialBT.connected(0)) {
    Serial.println(F("Link lost - polling stopped."));
    polling = false;
    reqActive = false;
    return;
  }
  if (!reqActive) {
    reqActive = true;
    reqStart = millis();
  }
  float v = callPid(pidIdx);
  int8_t st = myELM327.nb_rx_state;

  if (st == ELM_GETTING_MSG) {
    if (millis() - reqStart > (uint32_t)TEST_ELM_TIMEOUT + 2000UL) {
      Serial.printf("%-14s : stuck waiting - stopping poll\n", PID_NAME[pidIdx]);
      polling = false;
      reqActive = false;
    }
    return;
  }
  reqActive = false;
  if (st == ELM_SUCCESS) {
    Serial.printf("%-14s : %.2f\n", PID_NAME[pidIdx], v);
  } else if (st == ELM_NO_DATA) {
    Serial.printf("%-14s : N/A (NO DATA - not supported or ECU silent)\n", PID_NAME[pidIdx]);
  } else {
    Serial.printf("%-14s : error %d -> ", PID_NAME[pidIdx], (int)st);
    myELM327.printError();
  }
  pidIdx = (pidIdx + 1) % NUM_PIDS;
}

void runCommand(char *l) {
  while (*l == ' ') l++;
  switch (tolower(*l)) {
    case 'h': printHelp(); break;
    case 's': doScan(); break;
    case 'm': {
      char *a = l + 1;
      while (*a == ' ') a++;
      if (parseMac(a, targetMac)) {
        haveMac = true;
        Serial.printf("MAC set to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      targetMac[0], targetMac[1], targetMac[2], targetMac[3], targetMac[4], targetMac[5]);
      } else {
        Serial.println(F("Bad format. Use: m AA:BB:CC:DD:EE:FF"));
      }
      break;
    }
    case 'c': doConnect(); break;
    case 'p':
      if (!SerialBT.connected(0)) { Serial.println(F("Not connected. Use c first.")); break; }
      polling = !polling;
      reqActive = false;
      Serial.println(polling ? F("Polling ON") : F("Polling OFF"));
      break;
    case 'd':
      polling = false;
      reqActive = false;
      if (SerialBT.connected(0)) SerialBT.disconnect();
      Serial.println(F("Disconnected."));
      break;
    case 'u': doUnpair(); break;
    default: break;
  }
}

void handleSerial() {
  static char line[40];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[n] = 0;
      n = 0;
      runCommand(line);
    } else if (n < sizeof(line) - 1) {
      line[n++] = ch;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  SerialBT.begin("ESP32-OBD-Test", true);     // master / client
  setPin();
  if (strlen(TEST_BT_MAC) > 0 && parseMac(TEST_BT_MAC, targetMac)) haveMac = true;
  printHelp();
  doScan();                                   // step 1 runs automatically
}

void loop() {
  handleSerial();
  if (polling) pollStep();
}
