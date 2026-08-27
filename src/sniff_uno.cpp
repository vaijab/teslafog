// Change detector. Listen-only, unfiltered, DBC-independent.
//
// Learns which individual bits are volatile during a baseline window, then
// reports only changes in bits that were stable. This matters because a single
// message commonly carries both a free-running counter and the signal you are
// hunting: muting the whole ID loses the signal, keeping it drowns you in
// counter traffic. Masking per bit keeps the counter quiet and the signal loud.
//
// Workflow: put the car in the "before" state, press any key, wait for the
// baseline, then flip exactly one switch. What prints is carrying that signal.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

static const uint8_t PIN_CS = 10;
static const uint32_t MCP_SPI_HZ = 8000000;

static const uint8_t REG_EFLG = 0x2D;
static const uint8_t CMD_READ = 0x03;

// Confirmed by the scanner on this vehicle.
static const CAN_SPEED BUS_SPEED = CAN_500KBPS;

static const uint16_t LEARN_MS = 2000;
// Long enough for slow counters to roll through their bits and be masked out.
static const uint16_t PROFILE_MS = 20000;
// A real signal reports once or twice when you flip a switch. Anything that
// keeps reporting is a counter whose bits happened to sit still through the
// profile window, so mute it and keep the output readable.
static const uint8_t WATCH_NOISE_LIMIT = 6;
static const uint8_t MAX_IDS = 52;

// A busy vehicle bus carries more IDs than the Uno can hold state for, so sweep
// it in passes. Widen or move this window and reflash; the [window] line at
// startup reports how full the table got, which tells you if the pass was
// truncated.
static const uint16_t ID_MIN = 0x3F5;
static const uint16_t ID_MAX = 0x3F5;

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);

struct Entry {
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t volatileBits[8];
  uint8_t reports;
  bool muted;
};

static Entry entries[MAX_IDS];
static uint8_t entryCount = 0;
static uint8_t overflowed = 0;

enum class Phase : uint8_t { Learn, Profile, Watch };
static Phase phase = Phase::Learn;
static unsigned long phaseStart = 0;

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

static void printHex8(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static Entry *findEntry(uint16_t id) {
  for (uint8_t i = 0; i < entryCount; i++) {
    if (entries[i].id == id) return &entries[i];
  }
  return nullptr;
}

static Entry *addEntry(uint16_t id, const uint8_t *data, uint8_t dlc) {
  if (entryCount >= MAX_IDS) {
    overflowed++;
    return nullptr;
  }
  Entry *e = &entries[entryCount++];
  e->id = id;
  e->dlc = dlc;
  e->reports = 0;
  e->muted = false;
  for (uint8_t i = 0; i < 8; i++) {
    e->data[i] = (i < dlc) ? data[i] : 0;
    e->volatileBits[i] = 0;
  }
  return e;
}

static void report(const Entry *e, const uint8_t *now, uint8_t dlc, const uint8_t *stable) {
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(e->id, HEX);
  for (uint8_t i = 0; i < dlc; i++) {
    if (stable[i] == 0) continue;
    Serial.print(F("  b"));
    Serial.print(i);
    Serial.print(' ');
    printHex8(e->data[i]);
    Serial.print('>');
    printHex8(now[i]);
    Serial.print(F(" ^"));
    printHex8(stable[i]);
  }
  Serial.print(F("   full:"));
  for (uint8_t i = 0; i < dlc; i++) {
    Serial.print(' ');
    printHex8(now[i]);
  }
  Serial.println();
}

static void beginPhase(Phase next) {
  phase = next;
  phaseStart = millis();
  // EFLG's overrun bits latch until cleared, so without this the flag set
  // during startup is reported at the end of every phase forever after.
  mcp2515.clearRXnOVR();

  if (next == Phase::Learn) {
    entryCount = 0;
    overflowed = 0;
    Serial.println(F("\n[baseline] learning IDs, stay still..."));
    return;
  }
  if (next == Phase::Profile) {
    for (uint8_t i = 0; i < entryCount; i++) {
      entries[i].reports = 0;
      entries[i].muted = false;
      for (uint8_t b = 0; b < 8; b++) entries[i].volatileBits[b] = 0;
    }
    Serial.print(F("[baseline] found "));
    Serial.print(entryCount);
    Serial.print(F(" IDs in window"));
    if (entryCount >= MAX_IDS || overflowed) Serial.print(F(" (TABLE FULL - narrow it)"));
    Serial.println(F(", profiling volatile bits, keep still..."));
    return;
  }

  uint16_t masked = 0;
  uint16_t watched = 0;
  for (uint8_t i = 0; i < entryCount; i++) {
    for (uint8_t b = 0; b < entries[i].dlc; b++) {
      uint8_t v = entries[i].volatileBits[b];
      for (uint8_t bit = 0; bit < 8; bit++) {
        if (v & (1 << bit)) masked++;
        else watched++;
      }
    }
  }
  Serial.print(F("[baseline] masked "));
  Serial.print(masked);
  Serial.print(F(" volatile bits, watching "));
  Serial.print(watched);
  Serial.println(F(". Flip one switch now.\n"));
}

static void handleFrame(const struct can_frame &frame) {
  if (frame.can_id & CAN_EFF_FLAG) return;
  const uint16_t id = (uint16_t)(frame.can_id & CAN_SFF_MASK);
  if (id < ID_MIN || id > ID_MAX) return;
  const uint8_t dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;

  Entry *e = findEntry(id);
  if (e == nullptr) {
    e = addEntry(id, frame.data, dlc);
    if (e != nullptr && phase == Phase::Watch) {
      Serial.print(millis());
      Serial.print(F(" NEW ID "));
      Serial.println(id, HEX);
    }
    return;
  }

  uint8_t changed[8];
  bool any = false;
  for (uint8_t i = 0; i < dlc; i++) {
    changed[i] = e->data[i] ^ frame.data[i];
    if (changed[i]) any = true;
  }
  if (!any && dlc == e->dlc) return;

  if (phase == Phase::Profile) {
    for (uint8_t i = 0; i < dlc; i++) e->volatileBits[i] |= changed[i];
  } else if (phase == Phase::Watch && !e->muted) {
    uint8_t stable[8];
    bool report_it = false;
    for (uint8_t i = 0; i < dlc; i++) {
      stable[i] = changed[i] & ~e->volatileBits[i];
      if (stable[i]) report_it = true;
    }
    if (report_it) {
      if (e->reports >= WATCH_NOISE_LIMIT) {
        e->muted = true;
        Serial.print(F("[muted "));
        Serial.print(e->id, HEX);
        Serial.println(F(" - too noisy]"));
      } else {
        e->reports++;
        report(e, frame.data, dlc, stable);
      }
    }
  }

  e->dlc = dlc;
  for (uint8_t i = 0; i < 8; i++) e->data[i] = (i < dlc) ? frame.data[i] : 0;
}

void setup() {
  // Faster than the other sketches: at 115200 a full transmit buffer blocks
  // long enough for the MCP2515's two receive buffers to overrun on a ~1000
  // frame/s vehicle bus, which silently costs you frames mid-capture.
  Serial.begin(500000);
  while (!Serial) {
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(BUS_SPEED, MCP_8MHZ);
  mcp2515.setListenOnlyMode();

  Serial.println(F("\n\n=== change detector (listen-only, per-bit masking) ==="));
  Serial.print(F("window: "));
  Serial.print(ID_MIN, HEX);
  Serial.print(F(" - "));
  Serial.println(ID_MAX, HEX);
  Serial.println(F("reports: <ms> <id>  b<n> old>new ^changedbits   full: <payload>"));
  Serial.println(F("press any key to re-baseline"));
  beginPhase(Phase::Learn);
}

void loop() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) handleFrame(frame);

  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    beginPhase(Phase::Learn);
    return;
  }

  const unsigned long elapsed = millis() - phaseStart;
  if (phase == Phase::Learn && elapsed >= LEARN_MS) {
    if (overflowed) {
      Serial.print(F("[baseline] ID table full, dropped frames from "));
      Serial.print(overflowed);
      Serial.println(F(" unlisted IDs"));
    }
    beginPhase(Phase::Profile);
  } else if (phase == Phase::Profile && elapsed >= PROFILE_MS) {
    uint8_t eflg = mcpRead(REG_EFLG);
    if (eflg & 0xC0) {
      Serial.print(F("[baseline] warning: RX overrun, EFLG=0x"));
      printHex8(eflg);
      Serial.println();
    }
    beginPhase(Phase::Watch);
  }
}
