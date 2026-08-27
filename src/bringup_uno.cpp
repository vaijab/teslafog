// MCP2515 bring-up diagnostic for Arduino Uno R3.
//
// Uno and HW-184 are both 5 V, so this is a direct connection with no level
// shifting:
//
//   VCC -> 5V     CS  -> D10     SO  -> D12 (MISO)     INT -> D2
//   GND -> GND    SI  -> D11     SCK -> D13
//
// Runs four checks, then exercises transmit and receive in loopback mode. The
// CAN screw terminals can stay disconnected: loopback routes frames back inside
// the controller, so nothing reaches the transceiver or a bus.
//
// Loopback proves SPI, bit timing, framing and buffer handling. It does not
// prove the transceiver works or that the timing matches another node's - both
// ends share one oscillator here, so a wrong crystal setting still passes.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

static const uint8_t PIN_CS = 10;
static const uint8_t PIN_INT = 2;  // wired but unused; polling is enough here

// Verified clean to 8 MHz by the speed sweep below. MCP2515 tops out at 10 MHz.
static const uint32_t MCP_SPI_HZ = 8000000;

// After the checks: 1 keeps hammering loopback, 0 switches to listen-only and
// dumps whatever is on the bus. Use 0 when connected to a vehicle.
#define STAY_IN_LOOPBACK 1

static const uint8_t CMD_WRITE = 0x02;
static const uint8_t CMD_READ = 0x03;
static const uint8_t CMD_RESET = 0xC0;

static const uint8_t REG_RXF0SIDH = 0x00;  // scratch: writable in config mode, no side effects
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_TEC = 0x1C;
static const uint8_t REG_REC = 0x1D;
static const uint8_t REG_CNF3 = 0x28;
static const uint8_t REG_CNF2 = 0x29;
static const uint8_t REG_CNF1 = 0x2A;
static const uint8_t REG_EFLG = 0x2D;

// Post-reset defaults from the MCP2515 datasheet.
static const uint8_t CANSTAT_AFTER_RESET = 0x80;  // OPMOD = 100, configuration
static const uint8_t CANCTRL_AFTER_RESET = 0x87;

static const uint8_t OPMOD_MASK = 0xE0;
static const uint8_t OPMOD_LOOPBACK = 0x40;
static const uint8_t OPMOD_LISTEN_ONLY = 0x60;

// 8 MHz crystal, 500 kbit/s, per the autowp library timing table.
static const uint8_t EXPECT_CNF1 = 0x00;
static const uint8_t EXPECT_CNF2 = 0x90;
static const uint8_t EXPECT_CNF3 = 0x82;

static const uint8_t EFLG_RX0OVR = 0x40;
static const uint8_t EFLG_RX1OVR = 0x80;

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);

static uint32_t spiHz = MCP_SPI_HZ;
static uint32_t rngState = 0x2A3B4C5D;

static uint32_t frameCount = 0;
static uint32_t framesThisWindow = 0;
static uint32_t overrunEvents = 0;
static unsigned long lastStats = 0;

static uint32_t nextRandom() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

static void printHex8(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static inline void mcpSelect() {
  SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
}

static inline void mcpDeselect() {
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

static uint8_t mcpRead(uint8_t reg) {
  mcpSelect();
  SPI.transfer(CMD_READ);
  SPI.transfer(reg);
  uint8_t value = SPI.transfer(0x00);
  mcpDeselect();
  return value;
}

static void mcpWrite(uint8_t reg, uint8_t value) {
  mcpSelect();
  SPI.transfer(CMD_WRITE);
  SPI.transfer(reg);
  SPI.transfer(value);
  mcpDeselect();
}

static void mcpReset() {
  mcpSelect();
  SPI.transfer(CMD_RESET);
  mcpDeselect();
  delay(10);
}

static bool report(const __FlashStringHelper *label, uint8_t got, uint8_t expected) {
  bool ok = (got == expected);
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(" got 0x"));
  printHex8(got);
  Serial.print(F(" expected 0x"));
  printHex8(expected);
  Serial.println(ok ? F("  PASS") : F("  FAIL"));
  return ok;
}

// Random write/read-back over a scratch register. Wiring faults show up as a
// solid failure; marginal signals show up as a low, sporadic error rate, so run
// enough cycles to see one.
static uint16_t patternTest(uint16_t iterations) {
  uint16_t errors = 0;
  for (uint16_t i = 0; i < iterations; i++) {
    uint8_t written = (uint8_t)(nextRandom() & 0xFF);
    mcpWrite(REG_RXF0SIDH, written);
    if (mcpRead(REG_RXF0SIDH) != written) errors++;
  }
  return errors;
}

// Sweeping the clock separates "wiring is wrong" from "wiring has no margin".
static void spiSpeedSweep() {
  static const uint32_t speeds[] PROGMEM = {1000000UL, 2000000UL, 4000000UL, 8000000UL};
  Serial.println(F("\n[2] SPI integrity vs clock (2000 write/read cycles each)"));
  for (uint8_t i = 0; i < 4; i++) {
    uint32_t speed;
    memcpy_P(&speed, &speeds[i], sizeof(speed));
    spiHz = speed;
    uint16_t errors = patternTest(2000);
    Serial.print(F("  "));
    Serial.print(speed / 1000UL);
    Serial.print(F(" kHz  "));
    Serial.print(errors);
    Serial.println(errors == 0 ? F(" errors  PASS") : F(" errors  FAIL"));
  }
  spiHz = MCP_SPI_HZ;
}

// Round-trips frames through the controller's internal loopback and compares
// every field. Exercises the transmit path, the receive buffers and the frame
// encode/decode that everything above this depends on.
static uint16_t loopbackTest(uint16_t iterations) {
  uint16_t failures = 0;
  for (uint16_t i = 0; i < iterations; i++) {
    struct can_frame tx;
    bool extended = (i & 1);
    tx.can_id = extended ? ((nextRandom() & CAN_EFF_MASK) | CAN_EFF_FLAG)
                         : (nextRandom() & CAN_SFF_MASK);
    tx.can_dlc = (uint8_t)(nextRandom() % 9);
    for (uint8_t b = 0; b < tx.can_dlc; b++) tx.data[b] = (uint8_t)(nextRandom() & 0xFF);

    if (mcp2515.sendMessage(&tx) != MCP2515::ERROR_OK) {
      failures++;
      continue;
    }

    struct can_frame rx;
    MCP2515::ERROR err = MCP2515::ERROR_NOMSG;
    unsigned long deadline = millis() + 20;
    while ((long)(millis() - deadline) < 0) {
      err = mcp2515.readMessage(&rx);
      if (err == MCP2515::ERROR_OK) break;
    }

    if (err != MCP2515::ERROR_OK || rx.can_id != tx.can_id || rx.can_dlc != tx.can_dlc ||
        memcmp(rx.data, tx.data, tx.can_dlc) != 0) {
      failures++;
    }
  }
  return failures;
}

static void printFrame(const struct can_frame &frame) {
  bool extended = frame.can_id & CAN_EFF_FLAG;
  uint32_t id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  Serial.print(millis());
  Serial.print(extended ? F("  EXT ") : F("  STD "));
  Serial.print(id, HEX);
  Serial.print(F("  ["));
  Serial.print(frame.can_dlc);
  Serial.print(F("] "));
  for (uint8_t i = 0; i < frame.can_dlc; i++) {
    printHex8(frame.data[i]);
    Serial.print(' ');
  }
  Serial.println();
}

static void printStats() {
  uint8_t eflg = mcpRead(REG_EFLG);
  if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
    overrunEvents++;
    mcp2515.clearRXnOVR();
  }
  Serial.print(F("--- "));
  Serial.print(framesThisWindow);
  Serial.print(F(" fps, "));
  Serial.print(frameCount);
  Serial.print(F(" total, REC="));
  Serial.print(mcpRead(REG_REC));
  Serial.print(F(" TEC="));
  Serial.print(mcpRead(REG_TEC));
  Serial.print(F(" EFLG=0x"));
  printHex8(eflg);
  Serial.print(F(" overruns="));
  Serial.println(overrunEvents);
  framesThisWindow = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT, INPUT);
  SPI.begin();

  Serial.println(F("\n\n=== MCP2515 bring-up (Uno) ==="));

  Serial.println(F("\n[1] Reset and post-reset register defaults"));
  mcpReset();
  bool ok = report(F("CANSTAT"), mcpRead(REG_CANSTAT), CANSTAT_AFTER_RESET);
  ok &= report(F("CANCTRL"), mcpRead(REG_CANCTRL), CANCTRL_AFTER_RESET);
  if (!ok) {
    Serial.println(F("  0x00 or 0xFF means no SPI link at all: check CS, the 5V"));
    Serial.println(F("  supply to the module, and that grounds are common."));
  }

  spiSpeedSweep();

  Serial.println(F("\n[3] Bit timing for 8 MHz crystal @ 500 kbit/s"));
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  ok &= report(F("CNF1"), mcpRead(REG_CNF1), EXPECT_CNF1);
  ok &= report(F("CNF2"), mcpRead(REG_CNF2), EXPECT_CNF2);
  ok &= report(F("CNF3"), mcpRead(REG_CNF3), EXPECT_CNF3);

  Serial.println(F("\n[4] Loopback transmit and receive (200 frames)"));
  mcp2515.setLoopbackMode();
  bool modeOk = report(F("CANSTAT OPMOD"), mcpRead(REG_CANSTAT) & OPMOD_MASK, OPMOD_LOOPBACK);
  ok &= modeOk;
  uint16_t failures = loopbackTest(200);
  Serial.print(F("  round trip "));
  Serial.print(failures);
  Serial.println(failures == 0 ? F(" failures  PASS") : F(" failures  FAIL"));
  ok &= (failures == 0);

  Serial.println(ok ? F("\n=== ALL CHECKS PASSED ===")
                    : F("\n=== CHECKS FAILED - fix before proceeding ==="));

#if STAY_IN_LOOPBACK
  Serial.println(F("\nStaying in loopback, sending 10 frames/s to itself.\n"));
#else
  mcp2515.setListenOnlyMode();
  report(F("CANSTAT OPMOD"), mcpRead(REG_CANSTAT) & OPMOD_MASK, OPMOD_LISTEN_ONLY);
  Serial.println(F("\nListening. Nothing is transmitted; the bus is not ACKed.\n"));
#endif

  lastStats = millis();
}

void loop() {
#if STAY_IN_LOOPBACK
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    lastSend = millis();
    struct can_frame tx;
    tx.can_id = 0x3D8;
    tx.can_dlc = 8;
    for (uint8_t i = 0; i < 8; i++) tx.data[i] = (uint8_t)(nextRandom() & 0xFF);
    mcp2515.sendMessage(&tx);
  }
#endif

  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    frameCount++;
    framesThisWindow++;
    printFrame(frame);
  }

  if (millis() - lastStats >= 1000) {
    lastStats = millis();
    printStats();
  }
}
