#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  OBD link: runs Bluetooth + ELMduino in its own FreeRTOS task (core 0), so
//  the TFT / touch loop never waits for a connection attempt or a PID reply.
//  The UI only ever reads a copy of the shared snapshot.
// ---------------------------------------------------------------------------

enum ObdPidIndex : uint8_t {
  PIDX_RPM = 0,
  PIDX_SPEED,
  PIDX_COOLANT,
  PIDX_LOAD,
  PIDX_IAT,
  PIDX_MAP,
  PIDX_MAF,
  PIDX_THROTTLE,
  PIDX_VOLTAGE,
  PIDX_FUEL,
  PIDX_COUNT
};

// Three separate status layers (zero value = "nothing known yet"):
enum ObdLinkState : uint8_t {          // 1) Bluetooth link to the adapter
  OBD_LNK_IDLE = 0,
  OBD_LNK_CONNECTING,
  OBD_LNK_CONNECTED,
  OBD_LNK_DISCONNECTED,
  OBD_LNK_CONFIG_ERROR
};

enum ObdInitState : uint8_t {          // 2) ELM327 initialisation (ELMduino begin())
  OBD_INI_IDLE = 0,
  OBD_INI_RUNNING,
  OBD_INI_OK,
  OBD_INI_FAILED
};

enum ObdVehicleState : uint8_t {       // 3) Vehicle / ECU answering PID requests
  OBD_VEH_UNKNOWN = 0,
  OBD_VEH_DISCOVERING,
  OBD_VEH_ONLINE,
  OBD_VEH_NO_RESPONSE
};

enum ObdSupport : uint8_t {            // per-PID availability
  OBD_SUP_UNKNOWN = 0,
  OBD_SUP_YES,
  OBD_SUP_NO
};

enum ObdValueState : uint8_t {         // what the UI may show for a value
  OBD_VAL_NA = 0,                      // never read, unsupported or too old -> "N/A"
  OBD_VAL_STALE,                       // real value, but not fresh
  OBD_VAL_OK
};

struct ObdSnapshot {
  ObdLinkState    link;
  ObdInitState    init;
  ObdVehicleState veh;
  ObdSupport      support[PIDX_COUNT];
  bool            valid[PIDX_COUNT];   // a real reading exists
  float           value[PIDX_COUNT];
  uint32_t        stamp[PIDX_COUNT];   // millis() of last real reading
  uint32_t        linkLostAt;          // millis() when the link dropped, 0 if not applicable
  uint32_t        nextRetryAt;         // millis() of the next connect attempt
  uint16_t        connectAttempts;
  int8_t          lastError;           // last ELMduino nb_rx_state error code (0 = none)
  uint32_t        okCount;
  uint32_t        errCount;
};

void          obdBegin();                                   // call once from setup()
void          obdGetSnapshot(ObdSnapshot &out);             // thread-safe copy
void          obdRequestReconnect();                        // "reconnect now" (non-blocking)
ObdValueState obdValueState(const ObdSnapshot &s, uint8_t pid, uint32_t nowMs);
bool          obdTargetIsMac();                             // false if only a name is configured
const char   *obdTargetText();                              // MAC string or "NAME xxxx"
