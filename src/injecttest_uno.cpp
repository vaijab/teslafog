// 0x273 injection tests: single frame, and a bounded hold.
//
// The frame sent is always a verbatim copy of the CID's own most recent 0x273
// with UI_rearFogSwitch cleared - byte for byte what the touchscreen itself
// transmits when rear fog is off. Nothing is fabricated, so no other field can
// be disturbed, and the injection can only ever turn the lamp OFF.
//
//   t  inject one frame, then print every 273/3E3 frame for 3 seconds
//   h  hold for 10 seconds: answer every CID frame carrying switch=1 with an
//      immediate switch=0 copy
//
// Hold mode exists to answer two questions a single frame cannot:
//   1. Does re-asserting after each CID frame keep the lamp visually off?
//      The body controller follows the level with ~5 ms latency, so the lamp
//      should be lit only for the round trip, roughly 1 ms in every 500.
//   2. Does the dash tell-tale follow the actual lamp, or the CID's own switch
//      state? That decides whether this approach is R48-compliant or merely
//      makes the lamp dark while the indicator lies.
//
// Safety:
//   * Transmits nothing until you press a key.
//   * Refuses unless a live 0x273 has been captured with its switch bit set.
//   * ONE-SHOT mode throughout: no retry storms, an arbitration loss is simply
//     skipped.
//   * Hold is time-bounded and any keypress aborts it early.
//   * Returns to listen-only afterwards.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include <tesla_signals.h>

static const uint8_t PIN_CS = 10;
static const uint32_t MCP_SPI_HZ = 8000000;
static const CAN_SPEED BUS_SPEED = CAN_500KBPS;

static const uint16_t VERBOSE_MS = 3000;
static const uint16_t HOLD_MS = 10000;
static const uint16_t TX_TIMEOUT_MS = 50;
static const uint16_t TX_POLL_MS = 5;

static const uint8_t CMD_READ = 0x03;
static const uint8_t CMD_BITMOD = 0x05;

static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_TEC = 0x1C;
static const uint8_t REG_TXB0CTRL = 0x30;

static const uint8_t OPMOD_MASK = 0xE0;
static const uint8_t OPMOD_NORMAL = 0x00;
static const uint8_t OPMOD_CONFIG = 0x80;
static const uint8_t REQOP_MASK = 0xE0;
static const uint8_t CANCTRL_OSM = 0x08;

static const uint8_t TXREQ = 0x08;
static const uint8_t TXERR = 0x10;
static const uint8_t MLOA = 0x20;
static const uint8_t ABTF = 0x40;

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);

static uint8_t lastUi[8];
static bool haveUi = false;
static bool lastSwitch = false;
static bool lastRearFog = false;
static bool lastTail = false;
static bool haveRight = false;
static unsigned long verboseUntil = 0;

static bool holdActive = false;
static unsigned long holdUntil = 0;
static uint16_t holdInjections = 0;
static uint16_t holdArbLost = 0;
static uint16_t holdFailed = 0;
static uint16_t holdFogOn = 0;
static uint16_t holdFogOff = 0;

static uint8_t mcpRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(reg);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return value;
}

static void mcpBitModify(uint8_t reg, uint8_t mask, uint8_t data) {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_BITMOD);
  SPI.transfer(reg);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

static void printHex8(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static bool waitForMode(uint8_t opmod, uint16_t timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  while ((long)(millis() - deadline) < 0) {
    if ((mcpRead(REG_CANSTAT) & OPMOD_MASK) == opmod) return true;
  }
  return false;
}

// The library's setNormalOneShotMode() compares CANSTAT's mode field against
// the OSM bit, which lives in CANCTRL, so it always reports failure. Drive the
// registers directly instead.
static bool enterNormalOneShot() {
  mcpBitModify(REG_CANCTRL, REQOP_MASK, OPMOD_CONFIG);
  if (!waitForMode(OPMOD_CONFIG, 20)) return false;
  mcpBitModify(REG_CANCTRL, CANCTRL_OSM, CANCTRL_OSM);
  mcpBitModify(REG_CANCTRL, REQOP_MASK, OPMOD_NORMAL);
  if (!waitForMode(OPMOD_NORMAL, 50)) return false;
  return (mcpRead(REG_CANCTRL) & CANCTRL_OSM) != 0;
}

// Sends one switch=0 copy of the payload just received. Returns the TXB0CTRL
// result so the caller can tell a genuine failure from a lost arbitration.
static uint8_t sendSwitchOff(const uint8_t *source, uint16_t timeoutMs) {
  struct can_frame frame;
  frame.can_id = tesla::CAN_ID_UI_VEHICLE_CONTROL;
  frame.can_dlc = tesla::DLC_UI_VEHICLE_CONTROL;
  for (uint8_t i = 0; i < tesla::DLC_UI_VEHICLE_CONTROL; i++) frame.data[i] = source[i];
  tesla::setRearFogSwitch(frame.data, frame.can_dlc, false);

  mcp2515.sendMessage(&frame);

  uint8_t ctrl = 0;
  unsigned long deadline = millis() + timeoutMs;
  do {
    ctrl = mcpRead(REG_TXB0CTRL);
  } while ((ctrl & TXREQ) && (long)(millis() - deadline) < 0);
  return ctrl;
}

static void printTxResult(uint8_t ctrl) {
  Serial.print(F(">>> TXB0CTRL=0x"));
  printHex8(ctrl);
  Serial.print(F(" TEC="));
  Serial.print(mcpRead(REG_TEC));
  Serial.print(' ');
  if (ctrl & TXREQ) {
    Serial.println(F("INCONCLUSIVE - never completed"));
  } else if (ctrl & MLOA) {
    Serial.println(F("lost arbitration, retry"));
  } else if (ctrl & (TXERR | ABTF)) {
    Serial.println(F("NOT ACKNOWLEDGED"));
  } else {
    Serial.println(F("sent and acknowledged"));
  }
}

static bool armed() {
  if (!haveUi) {
    Serial.println(F("refusing: no 0x273 captured yet"));
    return false;
  }
  if (!lastSwitch) {
    Serial.println(F("refusing: switch is already 0, injection would change nothing"));
    Serial.println(F("turn rear fog ON first (headlights on)"));
    return false;
  }
  return true;
}

static void injectOnce() {
  if (!armed()) return;

  Serial.println(F("\n>>> injecting one frame"));
  if (!enterNormalOneShot()) {
    Serial.println(F("FAILED to arm one-shot - aborting, still passive"));
    mcp2515.setListenOnlyMode();
    return;
  }
  uint8_t ctrl = sendSwitchOff(lastUi, TX_TIMEOUT_MS);
  mcp2515.setListenOnlyMode();
  verboseUntil = millis() + VERBOSE_MS;

  printTxResult(ctrl);
  Serial.println(F(">>> watching 3s - every frame below\n"));
}

static void startHold() {
  if (!armed()) return;

  Serial.println(F("\n>>> HOLD: answering every CID frame for 10s"));
  Serial.println(F(">>> WATCH THE DASH: does the amber fog icon go out?"));
  Serial.println(F(">>> any key aborts\n"));

  if (!enterNormalOneShot()) {
    Serial.println(F("FAILED to arm one-shot - aborting, still passive"));
    mcp2515.setListenOnlyMode();
    return;
  }

  holdInjections = 0;
  holdArbLost = 0;
  holdFailed = 0;
  holdFogOn = 0;
  holdFogOff = 0;
  holdUntil = millis() + HOLD_MS;
  holdActive = true;
}

static void endHold(bool aborted) {
  holdActive = false;
  mcp2515.setListenOnlyMode();

  Serial.println(aborted ? F("\n>>> HOLD ABORTED") : F("\n>>> HOLD COMPLETE"));
  Serial.print(F("    injections: "));
  Serial.print(holdInjections);
  Serial.print(F("   arbitration lost: "));
  Serial.print(holdArbLost);
  Serial.print(F("   failed: "));
  Serial.println(holdFailed);

  const uint16_t total = holdFogOn + holdFogOff;
  Serial.print(F("    3E3 status frames: lamp ON "));
  Serial.print(holdFogOn);
  Serial.print(F(", OFF "));
  Serial.print(holdFogOff);
  if (total > 0) {
    Serial.print(F("  ("));
    Serial.print((uint16_t)((uint32_t)holdFogOn * 100 / total));
    Serial.print(F("% on)"));
  }
  Serial.println();
  Serial.println(F("    tell-tale: your eyes, not the bus\n"));
}

static void printFrame(uint32_t id, const uint8_t *data, uint8_t dlc,
                       const __FlashStringHelper *tag) {
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(id, HEX);
  Serial.print(' ');
  Serial.print(tag);
  Serial.print(F("  raw"));
  for (uint8_t i = 0; i < dlc; i++) {
    Serial.print(' ');
    printHex8(data[i]);
  }
  Serial.println();
}

static void handleUi(const struct can_frame &frame) {
  if (frame.can_dlc < tesla::DLC_UI_VEHICLE_CONTROL) return;
  tesla::UiVehicleControl ui{};
  tesla::decodeUiVehicleControl(frame.data, frame.can_dlc, ui);

  // Reply first, print never: serial output here would add milliseconds to the
  // round trip and that latency is exactly what the lamp is lit for.
  if (holdActive) {
    for (uint8_t i = 0; i < 8; i++) lastUi[i] = frame.data[i];
    lastSwitch = ui.rearFogSwitch;
    if (ui.rearFogSwitch) {
      uint8_t ctrl = sendSwitchOff(frame.data, TX_POLL_MS);
      holdInjections++;
      if (ctrl & MLOA) holdArbLost++;
      else if (ctrl & (TXREQ | TXERR | ABTF)) holdFailed++;
    }
    return;
  }

  bool changed = !haveUi || ui.rearFogSwitch != lastSwitch;
  for (uint8_t i = 0; i < 8; i++) lastUi[i] = frame.data[i];
  lastSwitch = ui.rearFogSwitch;
  haveUi = true;

  if (changed || (long)(millis() - verboseUntil) < 0) {
    printFrame(tesla::CAN_ID_UI_VEHICLE_CONTROL, frame.data, frame.can_dlc,
               ui.rearFogSwitch ? F("switch=1") : F("switch=0"));
  }
}

static void handleRight(const struct can_frame &frame) {
  tesla::RightLighting r{};
  if (!tesla::decodeRightLighting(frame.data, frame.can_dlc, r)) return;

  const bool fog = tesla::rearFogOn(r);
  const bool tail = tesla::isOn(r.tail);

  if (holdActive) {
    if (fog) holdFogOn++;
    else holdFogOff++;
    lastRearFog = fog;
    lastTail = tail;
    return;
  }

  const bool changed = !haveRight || fog != lastRearFog || tail != lastTail;
  lastRearFog = fog;
  lastTail = tail;
  haveRight = true;

  if (changed || (long)(millis() - verboseUntil) < 0) {
    printFrame(tesla::CAN_ID_VCRIGHT_LIGHT_STATUS, frame.data, frame.can_dlc,
               fog ? F("rearFog=1") : F("rearFog=0"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(BUS_SPEED, MCP_8MHZ);

  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF1, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF2, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF3, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);
  mcp2515.setFilter(MCP2515::RXF4, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF5, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);

  mcp2515.setListenOnlyMode();

  Serial.println(F("\n\n=== 0x273 injection tests ==="));
  Serial.println(F("headlights ON, rear fog ON, then:"));
  Serial.println(F("  t  inject a single frame"));
  Serial.println(F("  h  hold the lamp off for 10s (watch the dash icon)\n"));
}

void loop() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    switch (frame.can_id & CAN_SFF_MASK) {
      case tesla::CAN_ID_UI_VEHICLE_CONTROL: handleUi(frame); break;
      case tesla::CAN_ID_VCRIGHT_LIGHT_STATUS: handleRight(frame); break;
      default: break;
    }
  }

  if (holdActive) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      endHold(true);
    } else if ((long)(millis() - holdUntil) >= 0) {
      endHold(false);
    }
    return;
  }

  if (Serial.available()) {
    int c = Serial.read();
    if (c == 't' || c == 'T') injectOnce();
    else if (c == 'h' || c == 'H') startHold();
  }
}
