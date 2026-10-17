AUDI-STYLE OBD-II DASHBOARD  (ESP32-32E classic + ILI9341 + XPT2046 + ELM327 Bluetooth)
=========================================================================================

STATUS / HONESTY NOTE
---------------------
Written and reviewed by reading only. NOT compiled, uploaded or run on hardware, and
never tested against a car. ELMduino names used (begin, nb_rx_state, ELM_SUCCESS,
ELM_GETTING_MSG, ELM_NO_DATA, rpm, kph, engineCoolantTemp, engineLoad, intakeAirTemp,
manifoldPressure, mafRate, throttle, ctrlModVoltage, fuelLevel, supportedPIDs_1_20 /
21_40 / 41_60, printError) were checked against the library's README and published API
lists, not against your installed copy. If the compiler complains about an ELMduino or
BluetoothSerial name, compare it with ELMduino.h / BluetoothSerial.h and report the error.

FILES
-----
ObdConnectionTest/ObdConnectionTest.ino   STEP 1+2: find the MAC, test the link (Serial Monitor)
AudiOBDCockpit/AudiOBDCockpit.ino         STEP 3: the multi-page dashboard
AudiOBDCockpit/Config.h                   all settings (MAC field, retries, staleness, touch)
AudiOBDCockpit/ObdLink.h / ObdLink.cpp    Bluetooth + ELMduino task and PID scheduler
AudiOBDCockpit/User_Setup.h               TFT_eSPI setup, UNCHANGED from the working project

Keep the three tabs (.ino, Config.h, ObdLink.h/.cpp) together in the folder
AudiOBDCockpit/ (folder name = .ino name).

LIBRARIES (Library Manager)
---------------------------
- TFT_eSPI                (Bodmer)       configured exactly as before (see README.txt of the demo)
- XPT2046_Touchscreen     (Paul Stoffregen)
- ELMduino                (PowerBroker2)  https://github.com/PowerBroker2/ELMduino
BluetoothSerial ships with the Arduino ESP32 core. Recommended core: 2.0.17. On core 3.x
the setPin() signature differs (handled with a version check) and other changes may
need adjusting.

ARDUINO IDE SETTINGS
--------------------
Board            : ESP32 Dev Module (classic ESP32, NOT S3)
Partition Scheme : Huge APP (3MB No OTA/1MB SPIFFS)   <-- IMPORTANT: Bluetooth Classic
                   makes the firmware too large for the default 1.2 MB app partition
Flash 4MB, CPU 240 MHz, PSRAM Disabled, Upload Speed 460800

TFT_eSPI / DISPLAY / TOUCH
--------------------------
Nothing changes: ILI9341 driver, pins, GPIO 21 active-HIGH backlight start-up and
tft.setRotation(1) are as before. The touch controller is on its own pins
(CS 33, SCLK 25, MOSI 32, MISO 39, IRQ 36), so TFT_eSPI's built-in touch cannot be used.
The dashboard drives it with XPT2046_Touchscreen on the HSPI peripheral; TFT_eSPI keeps
its default VSPI port with your User_Setup.h, so the two buses do not clash. (This
follows from how TFT_eSPI selects its SPI port; it has not been verified on your board.
If display and touch interfere, see Troubleshooting.)

=========================================================================================
STEP 1 - FIND THE ADAPTER'S MAC ADDRESS
=========================================================================================
1. Plug the ELM327 into the car's OBD socket, ignition ON.
2. Switch Bluetooth OFF on your phone (most adapters accept only one connection).
3. Upload ObdConnectionTest.ino, open Serial Monitor at 115200 baud, "Newline" line ending.
4. The sketch scans automatically (about 12 s) and prints every Bluetooth Classic device
   with name and address. Your adapter is usually named OBDII, V-LINK, OBDLink or similar.
   Type  s  to scan again. Write the address down as AA:BB:CC:DD:EE:FF.
(Alternative: the address is also visible in some phones' Bluetooth device details.)
Never copy an address from an example or from someone else's post - use your own.

=========================================================================================
STEP 2 - TEST THE CONNECTION BEFORE USING THE DASHBOARD
=========================================================================================
1. In the Serial Monitor type:   m AA:BB:CC:DD:EE:FF    (your address)
2. Type  c   -> connects, runs ELMduino begin(), then prints the supported-PID masks.
   Expected: "Bluetooth link OK", "ELM327 INIT OK", then PID lists.
3. Type  p   -> polls RPM, speed, coolant, load, intake temp, MAP, MAF, throttle, control
   module voltage and fuel level one at a time. Unsupported ones print "N/A".
   Engine running gives RPM/speed; with ignition only, several may answer N/A or error.
4. Only when this works, put the same address into Config.h (CFG_BT_MAC) and upload the
   dashboard.

If it fails:
- "BLUETOOTH CONNECT FAILED": type  u  (remove stored pairings) and retry; switch the
  phone's Bluetooth off; try PIN 0000 or 6789 (TEST_BT_PIN); try TEST_BT_CHANNEL 1.
- "ELM327 INIT FAILED": link works but initialisation did not; check ignition, retry c.
  Cheap clones are often flaky; try another adapter or re-plug it.
- About baud rates: Bluetooth SPP has no baud rate, so 38400 / 115200 cannot be selected
  through BluetoothSerial. They only apply to wired UART adapters. What you can tune here:
  PIN, SPP channel, ELM timeout (TEST_ELM_TIMEOUT / CFG_ELM_TIMEOUT_MS) and protocol.

=========================================================================================
STEP 3 - THE DASHBOARD
=========================================================================================
Edit Config.h: set CFG_BT_MAC (and PIN / channel if STEP 2 needed non-defaults). Upload.

Pages (tap the tabs in the bottom strip)
  DRIVE : tachometer bar 0-8000 (orange from 6500, red from 7250), shift lights, RPM,
          large speed, coolant, load, control-module voltage, fuel, throttle.
  DATA  : all ten PIDs with value and a tag: STALE, UNSUP (ECU says unsupported) or NO DATA.
  LINK  : Bluetooth / ELM327 / ECU status, target address, PID support counts, last error,
          raw touch coordinates, RECONNECT button.
Strip dots (BT, ELM, ECU): grey = unknown, orange = in progress, green = OK, red = problem.

Three separate status layers
  1. Bluetooth link   : idle / connecting / connected / disconnected / bad MAC format
  2. ELM327 init      : idle / initialising / ready / failed   (ELMduino begin())
  3. Vehicle / ECU    : unknown / scanning PIDs / online / no response
  plus per-PID support: yes / no / unknown (from the ECU's supported-PID masks; if the
  masks never arrive, PIDs are probed directly and marked unsupported after 3 NO DATA).

N/A policy
  A value is shown only if a real reading exists. Never read, unsupported or too old
  shows "N/A" (dim). Readings that are real but no longer fresh show in grey and as
  STALE. Nothing is ever shown as 0 or simulated. The tach bar eases toward the real RPM
  between readings (display interpolation only); the digits show the actual reading.
  Freshness limits are in Config.h (fast PIDs 3 s fresh / 10 s max, slow PIDs 20 s / 60 s).

Disconnect behaviour
  Link lost -> values stay visible as STALE for CFG_DISCONNECT_HOLD_MS (5 s), then N/A.
  Reconnection is automatic with back-off (2 s doubling to 15 s). The display and touch
  keep running because Bluetooth and ELMduino live in their own task on core 0; the UI
  (core 1) only reads a copy of the shared snapshot.

Scheduler
  One PID request at a time; the next is sent only after the current one ends in
  ELM_SUCCESS or an error. RPM and speed come up every fourth request, the rest in turn.
  Update speed depends on the adapter and the car's protocol.

Notes on specific PIDs
  - Control module voltage is OBD PID 0x42; many cars do not support it -> N/A. The
    adapter-side battery voltage (ATRV) is deliberately not substituted.
  - Fuel level (0x2F), MAF (0x10) and MAP (0x0B) are often missing on some vehicles.

TOUCH CALIBRATION
-----------------
Config.h holds common raw-range values (X 200..3700, Y 240..3800). They are NOT verified
for your unit. Open the LINK page and tap the four corners: the TOUCH line shows
"rawX,rawY > screenX,screenY". If positions are off, adjust CFG_TOUCH_X_MIN/MAX and
Y_MIN/MAX (swap MIN and MAX to mirror an axis); set CFG_TOUCH_SWAP_XY 1 if axes are
exchanged. If taps are missed, try CFG_TOUCH_USE_IRQ 0.

TROUBLESHOOTING
---------------
- Sketch too large: choose the Huge APP partition scheme.
- Link keeps failing after earlier pairings: set CFG_CLEAR_BONDS_ON_BOOT 1 for one boot
  (or use  u  in the test sketch), then set it back to 0.
- Display glitches when touching: TFT_eSPI and touch should be on different SPI hosts. If
  you ever add #define USE_HSPI_PORT to User_Setup.h, change CFG_TOUCH_SPI_HOST to VSPI.
- ELMduino debug text: CFG_ELM_DEBUG 1 (Serial Monitor 115200).
- Wrong colours / mirrored picture: see the troubleshooting list of the earlier demo README.

SAFETY
------
Use the touch screen only when the car is stationary. This is an information display,
not a diagnostic or safety instrument; adapters and vehicle data can be wrong or missing.
Remove cheap adapters from the OBD socket when parked for long periods.
