// teslafog - UNECE R48 6.11.7.3 rear fog lamp compliance controller.
//
// Watches the vehicle bus for the position lamps being switched off, and from
// then on answers every touchscreen 0x273 carrying UI_rearFogSwitch=1 with an
// immediate switch=0 copy, until the driver deliberately switches the lamp on
// again. See lib/fogcontrol/fog_controller.h for the rule set and
// docs/signals.md for how the signals were established.
//
// Wiring (Arduino Uno + MCP2515 HW-184, both 5 V, no level shifting):
//
//   VCC -> 5V     CS  -> D10     SO  -> D12 (MISO)     INT -> D2 (unused)
//   GND -> GND    SI  -> D11     SCK -> D13
//   CANH/CANL -> vehicle bus, NO termination resistor
//   optional status LED + resistor on D7
//
// D13 is the SPI clock, so the onboard LED flickers with bus traffic. That is
// normal and is not a status indicator; use D7 for that.
//
// Set DRY_RUN to 0 only after watching it make correct decisions in the car.

#include <Arduino.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <mcp2515.h>

#include <fog_controller.h>
#include <tesla_signals.h>

// 1 = decide and log but never transmit. Verify behaviour before arming.
#define DRY_RUN 0

static const uint8_t PIN_CS = 10;
static const uint8_t PIN_LED = 7;
static const uint32_t MCP_SPI_HZ = 8000000;
static const CAN_SPEED BUS_SPEED = CAN_500KBPS;

// Back to passive if the touchscreen stops transmitting: the car is asleep, or
// something is wrong. Never act on a bus that has gone quiet.
static const uint16_t BUS_TIMEOUT_MS = 3000;
// Error-passive is 128. Back off well before that.
static const uint8_t TEC_BACKOFF = 96;
static const uint16_t BACKOFF_MS = 5000;
static const uint16_t HEARTBEAT_MS = 10000;
static const uint16_t TX_POLL_MS = 5;

static const uint8_t CMD_READ = 0x03;
static const uint8_t CMD_BITMOD = 0x05;

static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_TEC = 0x1C;
static const uint8_t REG_REC = 0x1D;
static const uint8_t REG_CNF3 = 0x28;
static const uint8_t REG_CNF2 = 0x29;
static const uint8_t REG_CNF1 = 0x2A;
static const uint8_t REG_EFLG = 0x2D;
static const uint8_t REG_TXB0CTRL = 0x30;

static const uint8_t CANSTAT_AFTER_RESET = 0x80;
static const uint8_t CANCTRL_AFTER_RESET = 0x87;
static const uint8_t EXPECT_CNF1 = 0x00;
static const uint8_t EXPECT_CNF2 = 0x90;
static const uint8_t EXPECT_CNF3 = 0x82;

static const uint8_t OPMOD_MASK = 0xE0;
static const uint8_t OPMOD_NORMAL = 0x00;
static const uint8_t OPMOD_CONFIG = 0x80;
static const uint8_t REQOP_MASK = 0xE0;
static const uint8_t CANCTRL_OSM = 0x08;

static const uint8_t TXREQ = 0x08;
static const uint8_t TXERR = 0x10;
static const uint8_t MLOA = 0x20;
static const uint8_t ABTF = 0x40;

enum class State : uint8_t { Fault, Listening, Active, BackOff };

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);
fogcontrol::FogController controller;

static State state = State::Fault;
static unsigned long lastUiMs = 0;
static unsigned long lastFrontMs = 0;
static unsigned long backOffUntil = 0;
static unsigned long lastHeartbeat = 0;
static bool haveUi = false;
static bool haveFront = false;

static uint32_t txOk = 0;
static uint32_t txArbLost = 0;
static uint32_t txFailed = 0;
static uint32_t txPending = 0;
static uint32_t backOffs = 0;

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

// Normal mode with hardware retry, deliberately NOT one-shot.
//
// This bus runs around 60% loaded, so losing arbitration to a lower-numbered ID
// is routine traffic rather than an error. One-shot would discard the frame and
// leave the lamp lit until the next correction; the hardware instead retries
// within microseconds, which is exactly the behaviour CAN arbitration is for.
//
// The safety net is the TEC backoff in loop(): if the bus genuinely stops
// acknowledging, the error counter climbs and the controller goes passive. That
// catches sustained trouble, which one-shot never did.
static bool enterNormal() {
  mcpBitModify(REG_CANCTRL, REQOP_MASK, OPMOD_CONFIG);
  if (!waitForMode(OPMOD_CONFIG, 20)) return false;
  mcpBitModify(REG_CANCTRL, CANCTRL_OSM, 0);
  mcpBitModify(REG_CANCTRL, REQOP_MASK, OPMOD_NORMAL);
  if (!waitForMode(OPMOD_NORMAL, 50)) return false;
  return (mcpRead(REG_CANCTRL) & CANCTRL_OSM) == 0;
}

static bool check(const __FlashStringHelper *label, uint8_t got, uint8_t expected) {
  const bool ok = (got == expected);
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(" 0x"));
  printHex8(got);
  if (!ok) {
    Serial.print(F(" expected 0x"));
    printHex8(expected);
    Serial.println(F("  FAIL"));
  } else {
    Serial.println(F("  ok"));
  }
  return ok;
}

// Everything downstream depends on the SPI link and the bit timing being right,
// and both fail silently: a wrong crystal setting simply receives nothing.
static bool selfCheck() {
  Serial.println(F("[selfcheck]"));
  mcp2515.reset();
  bool ok = check(F("CANSTAT"), mcpRead(REG_CANSTAT), CANSTAT_AFTER_RESET);
  ok &= check(F("CANCTRL"), mcpRead(REG_CANCTRL), CANCTRL_AFTER_RESET);

  mcp2515.setBitrate(BUS_SPEED, MCP_8MHZ);
  ok &= check(F("CNF1"), mcpRead(REG_CNF1), EXPECT_CNF1);
  ok &= check(F("CNF2"), mcpRead(REG_CNF2), EXPECT_CNF2);
  ok &= check(F("CNF3"), mcpRead(REG_CNF3), EXPECT_CNF3);
  return ok;
}

// Cuts a ~2300 frame/s bus down to about 17. Both receive buffers accept all
// three IDs so a rollover cannot drop one.
static void applyFilters() {
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF1, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF2, false, tesla::CAN_ID_VCFRONT_LIGHTING);
  mcp2515.setFilter(MCP2515::RXF3, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF4, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);
  mcp2515.setFilter(MCP2515::RXF5, false, tesla::CAN_ID_VCFRONT_LIGHTING);
}

static void goPassive(State next) {
  mcp2515.setListenOnlyMode();
  state = next;
}

static bool goActive() {
  if (!enterNormal()) {
    goPassive(State::Listening);
    return false;
  }
  state = State::Active;
  return true;
}

static void transmit(const uint8_t *payload) {
#if DRY_RUN
  (void)payload;
  return;
#else
  struct can_frame frame;
  frame.can_id = tesla::CAN_ID_UI_VEHICLE_CONTROL;
  frame.can_dlc = tesla::DLC_UI_VEHICLE_CONTROL;
  for (uint8_t i = 0; i < tesla::DLC_UI_VEHICLE_CONTROL; i++) frame.data[i] = payload[i];

  mcp2515.sendMessage(&frame);

  // The hardware retries on its own now, so this poll is only for statistics.
  // A frame still pending when it expires is normal on a loaded bus and will go
  // out shortly; it is not counted as a failure.
  uint8_t ctrl = 0;
  unsigned long deadline = millis() + TX_POLL_MS;
  do {
    ctrl = mcpRead(REG_TXB0CTRL);
  } while ((ctrl & TXREQ) && (long)(millis() - deadline) < 0);

  // MLOA is sticky until the next TXREQ is set, so with hardware retry it means
  // "contended at least once during this transmission", not "failed". Count it
  // for visibility but judge success on TXERR/ABTF alone.
  if (ctrl & MLOA) txArbLost++;

  if (ctrl & TXREQ) txPending++;
  else if (ctrl & (TXERR | ABTF)) txFailed++;
  else txOk++;
#endif
}

static void serviceBus() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    const uint16_t id = frame.can_id & CAN_SFF_MASK;

    if (id == tesla::CAN_ID_VCFRONT_LIGHTING) {
      tesla::FrontLighting front{};
      if (tesla::decodeFrontLighting(frame.data, frame.can_dlc, front)) {
        controller.onFrontLighting(front, millis());
        lastFrontMs = millis();
        haveFront = true;
      }
      continue;
    }

    if (id == tesla::CAN_ID_VCRIGHT_LIGHT_STATUS) {
      tesla::RightLighting right{};
      if (tesla::decodeRightLighting(frame.data, frame.can_dlc, right)) {
        // Corrects at once if the lamp is lit while suppressing, which is what
        // closes the visible flash when the position lamps come back on.
        fogcontrol::Response response = controller.onRightLighting(right, millis());
        if (response.transmit && state == State::Active) transmit(response.payload);
      }
      continue;
    }

    if (id != tesla::CAN_ID_UI_VEHICLE_CONTROL) continue;

    lastUiMs = millis();
    haveUi = true;

    // Answer before anything else. The lamp is lit for exactly this round trip,
    // so no serial output or bookkeeping belongs between here and the transmit.
    fogcontrol::Response response = controller.onUiVehicleControl(frame.data, frame.can_dlc, millis());
    if (response.transmit && state == State::Active) transmit(response.payload);
  }
}

static void serviceLed() {
  const unsigned long now = millis();
  bool on = false;
  switch (state) {
    case State::Fault: on = (now % 200) < 100; break;
    case State::Listening: on = (now % 4000) < 100; break;
    case State::BackOff: on = (now % 1000) < 500; break;
    case State::Active: on = controller.suppressing() ? true : ((now % 2000) < 100); break;
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static void heartbeat() {
  const uint8_t eflg = mcpRead(REG_EFLG);
  Serial.print(F("["));
  switch (state) {
    case State::Fault: Serial.print(F("FAULT")); break;
    case State::Listening: Serial.print(F("listen")); break;
    case State::BackOff: Serial.print(F("backoff")); break;
    case State::Active: Serial.print(DRY_RUN ? F("active/dry") : F("active")); break;
  }
  Serial.print(F("] suppress="));
  Serial.print(controller.suppressing());
  Serial.print(F(" permit="));
  Serial.print(controller.permitted());
  Serial.print(F(" pos="));
  Serial.print(controller.positionLampsOn());
  Serial.print(F(" lamp="));
  Serial.print(controller.lampOn());
  Serial.print(F(" sw="));
  Serial.print(controller.switchAsserted());
  Serial.print(F(" decided="));
  Serial.print(controller.transmitCount());
  Serial.print(F(" tx="));
  Serial.print(txOk);
  Serial.print(F("/arb"));
  Serial.print(txArbLost);
  Serial.print(F("/fail"));
  Serial.print(txFailed);
  Serial.print(F("/pend"));
  Serial.print(txPending);
  Serial.print(F(" backoffs="));
  Serial.print(backOffs);
  Serial.print(F(" TEC="));
  Serial.print(mcpRead(REG_TEC));
  Serial.print(F(" REC="));
  Serial.print(mcpRead(REG_REC));
  Serial.print(F(" EFLG=0x"));
  printHex8(eflg);
  Serial.println();

  if (eflg & 0xC0) mcp2515.clearRXnOVR();
}

void setup() {
  // The bootloader on a genuine Uno R3 is optiboot, which clears the watchdog
  // on entry, so enabling it here cannot cause a reset loop.
  wdt_disable();

  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  Serial.println(F("\n\nteslafog - R48 6.11.7.3 rear fog compliance"));
#if DRY_RUN
  Serial.println(F("DRY RUN: decisions logged, nothing transmitted"));
#else
  Serial.println(F("ARMED: will transmit 0x273 suppression frames"));
#endif

  if (!selfCheck()) {
    Serial.println(F("[selfcheck] FAILED - staying passive"));
    state = State::Fault;
    return;
  }
  Serial.println(F("[selfcheck] passed"));

  applyFilters();

  // Go active before any traffic arrives rather than after. At wake the car
  // restores the lamp immediately, so every millisecond spent waiting to be
  // ready is a millisecond the lamp is lit. Transmitting still requires a
  // captured 0x273, so being active early cannot produce a fabricated frame -
  // it only removes the mode switch from the critical path.
  if (goActive()) {
    Serial.println(F("[active] armed, waiting for first 0x273"));
  } else {
    Serial.println(F("[listen] could not enter normal mode, staying passive"));
  }

  lastHeartbeat = millis();
  wdt_enable(WDTO_1S);
}

void loop() {
  if (state == State::Fault) {
    serviceLed();
    return;  // watchdog was never enabled, so this parks safely
  }

  wdt_reset();
  serviceBus();

  if (state == State::Active) {
    fogcontrol::Response response = controller.tick(millis());
    if (response.transmit) transmit(response.payload);
  }

  const unsigned long now = millis();
  // Only 0x273 gates readiness. The suppression latch starts set, so the first
  // correction needs a touchscreen payload but not a lamp status frame.
  const bool busAlive = haveUi && (now - lastUiMs) < BUS_TIMEOUT_MS;
  // 0x3F5 counts too: without it the controller cannot tell whether a switch-on
  // is permitted, and would suppress a legitimate one forever. Losing it must
  // hand control back to the car rather than block the lamp.
  const bool busLost = (haveUi && (now - lastUiMs) >= BUS_TIMEOUT_MS) ||
                       (haveFront && (now - lastFrontMs) >= BUS_TIMEOUT_MS);

  switch (state) {
    case State::Listening:
      if (busAlive) {
        if (goActive()) Serial.println(F("[active] bus healthy, suppression armed"));
      }
      break;

    case State::Active:
      // Never having seen traffic is not the same as having lost it: at wake the
      // controller may be ready before the car starts transmitting.
      if (busLost) {
        Serial.println(F("[listen] bus quiet, going passive"));
        goPassive(State::Listening);
      } else if (mcpRead(REG_TEC) > TEC_BACKOFF) {
        Serial.println(F("[backoff] transmit errors, going passive"));
        backOffs++;
        backOffUntil = now + BACKOFF_MS;
        goPassive(State::BackOff);
      }
      break;

    case State::BackOff:
      if ((long)(now - backOffUntil) >= 0) goPassive(State::Listening);
      break;

    case State::Fault: break;
  }

  serviceLed();

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    heartbeat();
  }
}
