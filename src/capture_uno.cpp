// Vehicle capture tool. Listen-only, for working through the checklist in
// docs/signals.md.
//
// Hardware acceptance filters cut the bus down to the three IDs this project
// cares about, which is what makes this usable at 115200 baud - an unfiltered
// Tesla bus produces far more text per second than the serial link can carry,
// and the frames you lose are invisible.
//
// Prints only when a decoded signal changes, so toggling a light produces one
// line. Byte-level noise that does not affect a signal of interest - ambient
// lighting brightness in 0x3F5, for instance - is ignored.
//
// Wiring is unchanged from the bring-up sketch. The CAN screw terminals now go
// to the vehicle: CANH to CANH, CANL to CANL, no termination resistor.
//
// Listen-only means the controller never drives the bus and never ACKs, so a
// wrong bit rate cannot disturb the car - it just receives nothing.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

#include <tesla_signals.h>

static const uint8_t PIN_CS = 10;
static const uint32_t MCP_SPI_HZ = 8000000;

static const uint8_t REG_TEC = 0x1C;
static const uint8_t REG_REC = 0x1D;
static const uint8_t REG_EFLG = 0x2D;
static const uint8_t CMD_READ = 0x03;

static const uint16_t STATS_INTERVAL_MS = 5000;
static const uint16_t NO_UI_WARNING_MS = 30000;

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);

static tesla::FrontLighting front;
static tesla::RightLighting right;
static tesla::UiVehicleControl ui;
static bool haveFront = false;
static bool haveRight = false;
static bool haveUi = false;

static uint32_t totalFront = 0, totalRight = 0, totalUi = 0;
static uint16_t windowFront = 0, windowRight = 0, windowUi = 0;
static unsigned long lastStats = 0;
static bool warnedNoUi = false;

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

static char lampChar(tesla::LampState state) {
  switch (state) {
    case tesla::LampState::Off: return '0';
    case tesla::LampState::On: return '1';
    case tesla::LampState::Fault: return 'F';
    default: return '?';
  }
}

static void printHex8(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static void printRaw(const struct can_frame &frame) {
  Serial.print(F(" raw"));
  for (uint8_t i = 0; i < frame.can_dlc; i++) {
    Serial.print(' ');
    printHex8(frame.data[i]);
  }
  Serial.println();
}

static void printTimestamp(uint32_t id) {
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(id, HEX);
  Serial.print(' ');
}

// The compliance-relevant booleans, so the checklist can be worked through
// without decoding bits by eye.
static void printDerived() {
  if (!haveFront || !haveRight) return;
  Serial.print(F("   => permit="));
  Serial.print(tesla::permitsRearFog(front));
  Serial.print(F(" positionLamps="));
  Serial.print(tesla::positionLampsOn(right));
  Serial.print(F(" rearFog="));
  Serial.println(tesla::rearFogOn(right));
}

static bool frontChanged(const tesla::FrontLighting &a, const tesla::FrontLighting &b) {
  return a.raw != b.raw;
}

static bool rightChanged(const tesla::RightLighting &a, const tesla::RightLighting &b) {
  return a.brake != b.brake || a.tail != b.tail || a.turnSignal != b.turnSignal ||
         a.reverse != b.reverse || a.rearFog != b.rearFog;
}

static void handleFront(const struct can_frame &frame) {
  totalFront++;
  windowFront++;
  tesla::FrontLighting decoded{};
  if (!tesla::decodeFrontLighting(frame.data, frame.can_dlc, decoded)) return;
  if (haveFront && !frontChanged(decoded, front)) {
    front = decoded;
    return;
  }
  front = decoded;
  haveFront = true;

  printTimestamp(frame.can_id & CAN_SFF_MASK);
  Serial.print(F("park="));
  Serial.print(front.parkLamps);
  Serial.print(F(" dipped="));
  Serial.print(front.dippedBeam);
  Serial.print(F(" main="));
  Serial.print(front.mainBeam);
  Serial.print(F(" frontFog="));
  Serial.print(front.frontFog);
  printRaw(frame);
  printDerived();
}

static void handleRight(const struct can_frame &frame) {
  totalRight++;
  windowRight++;
  tesla::RightLighting decoded{};
  if (!tesla::decodeRightLighting(frame.data, frame.can_dlc, decoded)) return;
  if (haveRight && !rightChanged(decoded, right)) {
    right = decoded;
    return;
  }
  right = decoded;
  haveRight = true;

  printTimestamp(frame.can_id & CAN_SFF_MASK);
  Serial.print(F("tail="));
  Serial.print(lampChar(right.tail));
  Serial.print(F(" rearFog="));
  Serial.print(lampChar(right.rearFog));
  Serial.print(F(" brake="));
  Serial.print(lampChar(right.brake));
  Serial.print(F(" reverse="));
  Serial.print(lampChar(right.reverse));
  Serial.print(F(" turn="));
  Serial.print(lampChar(right.turnSignal));
  printRaw(frame);
  printDerived();
}

static void handleUi(const struct can_frame &frame) {
  totalUi++;
  windowUi++;
  tesla::UiVehicleControl decoded{};
  if (!tesla::decodeUiVehicleControl(frame.data, frame.can_dlc, decoded)) return;

  bool first = !haveUi;
  bool changed = first || decoded.frontFogSwitch != ui.frontFogSwitch ||
                 decoded.rearFogSwitch != ui.rearFogSwitch;
  ui = decoded;
  haveUi = true;
  if (!changed) return;

  printTimestamp(frame.can_id & CAN_SFF_MASK);
  Serial.print(F("frontFogSwitch="));
  Serial.print(ui.frontFogSwitch);
  Serial.print(F(" rearFogSwitch="));
  Serial.print(ui.rearFogSwitch);
  printRaw(frame);
}

static void printStats() {
  Serial.print(F("--- 5s: 3F5="));
  Serial.print(windowFront);
  Serial.print(F(" 3E3="));
  Serial.print(windowRight);
  Serial.print(F(" 273="));
  Serial.print(windowUi);
  Serial.print(F(" | totals "));
  Serial.print(totalFront);
  Serial.print('/');
  Serial.print(totalRight);
  Serial.print('/');
  Serial.print(totalUi);
  Serial.print(F(" | REC="));
  Serial.print(mcpRead(REG_REC));
  Serial.print(F(" TEC="));
  Serial.print(mcpRead(REG_TEC));
  Serial.print(F(" EFLG=0x"));
  printHex8(mcpRead(REG_EFLG));
  Serial.println();

  windowFront = 0;
  windowRight = 0;
  windowUi = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

  // RXB0 takes MASK0 with RXF0/RXF1; RXB1 takes MASK1 with RXF2..RXF5. Unused
  // filters are pointed at an ID of interest rather than left at 0, which would
  // otherwise let unrelated traffic through.
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, tesla::CAN_ID_VCFRONT_LIGHTING);
  mcp2515.setFilter(MCP2515::RXF1, false, tesla::CAN_ID_VCRIGHT_LIGHT_STATUS);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF2, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF3, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF4, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);
  mcp2515.setFilter(MCP2515::RXF5, false, tesla::CAN_ID_UI_VEHICLE_CONTROL);

  mcp2515.setListenOnlyMode();

  Serial.println(F("\n\n=== Tesla lighting capture (listen-only) ==="));
  Serial.println(F("filters: 3F5 VCFRONT_lighting, 3E3 VCRIGHT_lightStatus, 273 UI_vehicleControl"));
  Serial.println(F("lamp states: 0=off 1=on F=fault ?=SNA; pairs are left,right"));
  Serial.println(F("lines print only when a decoded signal changes\n"));

  lastStats = millis();
}

void loop() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    switch (frame.can_id & CAN_SFF_MASK) {
      case tesla::CAN_ID_VCFRONT_LIGHTING: handleFront(frame); break;
      case tesla::CAN_ID_VCRIGHT_LIGHT_STATUS: handleRight(frame); break;
      case tesla::CAN_ID_UI_VEHICLE_CONTROL: handleUi(frame); break;
      default: break;
    }
  }

  if (millis() - lastStats >= STATS_INTERVAL_MS) {
    lastStats = millis();
    printStats();
  }

  if (!warnedNoUi && totalUi == 0 && millis() > NO_UI_WARNING_MS) {
    warnedNoUi = true;
    Serial.println(F("!! no 0x273 seen in 30s - either the CID only sends it on"));
    Serial.println(F("!! change, or this bus segment does not carry it"));
  }
}
