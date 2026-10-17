#pragma once
// ===========================================================================
//  USER CONFIGURATION  -  edit this file only
// ===========================================================================

// ---------------------------------------------------------------------------
//  ELM327 Bluetooth Classic target
// ---------------------------------------------------------------------------
// >>> SET THIS: the real Bluetooth MAC of YOUR ELM327 adapter, format
//     "AA:BB:CC:DD:EE:FF". Find it with the ObdConnectionTest sketch (README.txt).
//     No address is provided on purpose - do not guess one.
//     If left "" the dashboard falls back to connecting by NAME (slower and
//     less reliable, and the LINK page shows the MAC as not set).
#define CFG_BT_MAC           "01:23:45:67:89:ba"

#define CFG_BT_NAME          "OBDII"      // fallback only, used when CFG_BT_MAC is ""
#define CFG_BT_CHANNEL       0            // 0 = look up the SPP channel automatically; try 1 if connect fails
#define CFG_USE_PIN          1            // 1 = call setPin() before connecting
#define CFG_BT_PIN           "1234"       // common clone PINs: 1234, 0000, 6789
#define CFG_LOCAL_BT_NAME    "ESP32-Cockpit"
#define CFG_CLEAR_BONDS_ON_BOOT 0         // 1 = delete all stored pairings at boot (troubleshooting)
#define CFG_BT_RESTART_AFTER_FAILS 3      // restart the BT stack after N failed connects in a row (0 = never)

// NOTE: Bluetooth SPP has no baud rate. 38400 / 115200 only matter for wired
// UART adapters; they cannot be selected through BluetoothSerial.

// ---------------------------------------------------------------------------
//  ELMduino
// ---------------------------------------------------------------------------
#define CFG_ELM_PROTOCOL     '0'          // '0' = automatic (ELMduino default)
#define CFG_ELM_TIMEOUT_MS   2000         // per-command timeout passed to ELMduino
#define CFG_ELM_DEBUG        0            // 1 = ELMduino debug output on Serial (115200)
#define CFG_ELM_INIT_TRIES   2            // begin() attempts before dropping the Bluetooth link

// ---------------------------------------------------------------------------
//  PID support handling
// ---------------------------------------------------------------------------
#define CFG_TRUST_SUPPORT_MASK 1          // 1 = trust the ECU's supported-PID bitmask, 0 = probe every PID
#define CFG_DISCOVERY_ROUNDS_BEFORE_PROBE 3  // failed discovery rounds before probing PIDs directly
#define CFG_DISCOVERY_RETRY_MS 3000
#define CFG_NODATA_STREAK_LIMIT 20        // consecutive NO DATA answers => treat ECU as not responding

// ---------------------------------------------------------------------------
//  Reconnect back-off
// ---------------------------------------------------------------------------
#define CFG_RETRY_MIN_MS     2000
#define CFG_RETRY_MAX_MS     15000

// ---------------------------------------------------------------------------
//  Freshness / staleness of displayed values
// ---------------------------------------------------------------------------
#define CFG_FRESH_FAST_MS    3000         // RPM, speed, load, throttle, MAP, MAF
#define CFG_MAXAGE_FAST_MS   10000        // older than this -> N/A
#define CFG_FRESH_SLOW_MS    20000        // coolant, IAT, voltage, fuel
#define CFG_MAXAGE_SLOW_MS   60000
#define CFG_DISCONNECT_HOLD_MS 5000       // after link loss: show last values as STALE this long, then N/A

// ---------------------------------------------------------------------------
//  XPT2046 touch (pins as supplied; separate bus from the display)
// ---------------------------------------------------------------------------
#define CFG_TOUCH_CS         33
#define CFG_TOUCH_SCLK       25
#define CFG_TOUCH_MOSI       32
#define CFG_TOUCH_MISO       39
#define CFG_TOUCH_IRQ        36
#define CFG_TOUCH_SPI_HOST   HSPI         // TFT_eSPI uses VSPI with your User_Setup.h
#define CFG_TOUCH_USE_IRQ    1            // 0 = poll without the IRQ pin if taps are unreliable

// Raw-to-screen calibration. These are COMMON values for this kind of board
// and are NOT verified on your unit: check them with the raw read-out on the
// LINK page (tap the corners). If the picture is mirrored, swap MIN and MAX.
#define CFG_TOUCH_X_MIN      200
#define CFG_TOUCH_X_MAX      3700
#define CFG_TOUCH_Y_MIN      240
#define CFG_TOUCH_Y_MAX      3800
#define CFG_TOUCH_SWAP_XY    0
