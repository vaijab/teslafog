// Bus scanner. Listen-only, unfiltered, for answering "is there anything on
// this wire, at what speed, and what IDs does it carry".
//
// Use when the capture sketch shows silence. It sweeps bit rates, picks the one
// that actually receives frames, then inventories the IDs present.
//
// Prints an ID census rather than every frame, so it cannot outrun the serial
// link the way an unfiltered raw dump does on a busy bus.
//
// Listen-only throughout: never transmits, never ACKs. A wrong bit rate costs
// nothing but silence.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include <tesla_signals.h>

static const uint8_t PIN_CS = 10;
static const uint32_t MCP_SPI_HZ = 8000000;

static const uint8_t REG_TEC = 0x1C;
static const uint8_t REG_REC = 0x1D;
static const uint8_t REG_EFLG = 0x2D;
static const uint8_t CMD_READ = 0x03;

// Set to a CAN_SPEED value to skip the sweep and go straight to listening.
#define FORCE_SPEED_INDEX -1

static const uint16_t SWEEP_DWELL_MS = 1500;
static const uint16_t CENSUS_INTERVAL_MS = 5000;
static const uint8_t MAX_IDS = 180;

struct SpeedEntry {
  CAN_SPEED speed;
  uint16_t kbps;
};

static const SpeedEntry SPEEDS[] = {
    {CAN_500KBPS, 500}, {CAN_250KBPS, 250},  {CAN_125KBPS, 125}, {CAN_1000KBPS, 1000},
    {CAN_100KBPS, 100}, {CAN_83K3BPS, 83},   {CAN_50KBPS, 50},   {CAN_33KBPS, 33},
};
static const uint8_t SPEED_COUNT = sizeof(SPEEDS) / sizeof(SPEEDS[0]);

MCP2515 mcp2515(PIN_CS, MCP_SPI_HZ, &SPI);

static const uint16_t WANTED[3] = {(uint16_t)tesla::CAN_ID_VCFRONT_LIGHTING,
                                   (uint16_t)tesla::CAN_ID_VCRIGHT_LIGHT_STATUS,
                                   (uint16_t)tesla::CAN_ID_UI_VEHICLE_CONTROL};
// Counted outside the ID table so a full table can never hide them.
static uint16_t wantedCounts[3] = {0, 0, 0};

static uint16_t seenIds[MAX_IDS];
static uint16_t seenCounts[MAX_IDS];
static uint8_t seenTotal = 0;
static uint8_t lastListedTotal = 0xFF;
static uint16_t extendedFrames = 0;
static uint32_t windowFrames = 0;
static uint32_t totalFrames = 0;
static unsigned long lastCensus = 0;
static uint8_t chosenIndex = 0;

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

// Filters are left wide open: reset() clears both masks to zero, which accepts
// every ID.
static void listenAt(uint8_t index) {
  mcp2515.reset();
  mcp2515.setBitrate(SPEEDS[index].speed, MCP_8MHZ);
  mcp2515.setListenOnlyMode();
}

static uint16_t countFramesFor(uint16_t durationMs) {
  uint16_t frames = 0;
  unsigned long deadline = millis() + durationMs;
  struct can_frame frame;
  while ((long)(millis() - deadline) < 0) {
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      if (frames < 0xFFFF) frames++;
    }
  }
  return frames;
}

static uint8_t sweep() {
  Serial.println(F("\n=== bit rate sweep (listen-only) ==="));
  uint8_t best = 0;
  uint16_t bestFrames = 0;

  for (uint8_t i = 0; i < SPEED_COUNT; i++) {
    listenAt(i);
    uint16_t frames = countFramesFor(SWEEP_DWELL_MS);
    uint8_t rec = mcpRead(REG_REC);
    uint8_t eflg = mcpRead(REG_EFLG);

    Serial.print(F("  "));
    if (SPEEDS[i].kbps < 1000) Serial.print(' ');
    Serial.print(SPEEDS[i].kbps);
    Serial.print(F(" kbit/s  "));
    Serial.print(frames);
    Serial.print(F(" frames  REC="));
    Serial.print(rec);
    Serial.print(F(" EFLG=0x"));
    printHex8(eflg);
    if (frames > bestFrames) {
      bestFrames = frames;
      best = i;
      Serial.print(F("   <-- best so far"));
    }
    Serial.println();
  }

  if (bestFrames == 0) {
    Serial.println(F("\n!! nothing received at any bit rate."));
    Serial.println(F("!! check: is the car awake? CANH/CANL on the right pins?"));
    Serial.println(F("!! module powered (POW led)? grounds common with the Uno?"));
    Serial.println(F("!! a climbing REC above means traffic exists but timing is wrong."));
  }
  return best;
}

static void recordId(const struct can_frame &frame) {
  totalFrames++;
  windowFrames++;
  if (frame.can_id & CAN_EFF_FLAG) {
    extendedFrames++;
    return;
  }
  uint16_t id = (uint16_t)(frame.can_id & CAN_SFF_MASK);
  for (uint8_t w = 0; w < 3; w++) {
    if (id == WANTED[w] && wantedCounts[w] < 0xFFFF) wantedCounts[w]++;
  }
  for (uint8_t i = 0; i < seenTotal; i++) {
    if (seenIds[i] == id) {
      if (seenCounts[i] < 0xFFFF) seenCounts[i]++;
      return;
    }
  }
  if (seenTotal < MAX_IDS) {
    seenIds[seenTotal] = id;
    seenCounts[seenTotal] = 1;
    seenTotal++;
  }
}

static void reportInterestingIds() {
  Serial.print(F("  wanted:"));
  for (uint8_t w = 0; w < 3; w++) {
    Serial.print(' ');
    Serial.print(WANTED[w], HEX);
    Serial.print('=');
    Serial.print(wantedCounts[w]);
  }
  Serial.println();
}

static void printCensus() {
  Serial.print(F("--- "));
  Serial.print(windowFrames);
  Serial.print(F(" frames/5s, "));
  Serial.print(seenTotal);
  Serial.print(F(" unique IDs, "));
  Serial.print(extendedFrames);
  Serial.print(F(" ext, total "));
  Serial.print(totalFrames);
  Serial.print(F(", REC="));
  Serial.print(mcpRead(REG_REC));
  Serial.print(F(" TEC="));
  Serial.print(mcpRead(REG_TEC));
  Serial.print(F(" EFLG=0x"));
  printHex8(mcpRead(REG_EFLG));
  Serial.println();

  if (seenTotal >= MAX_IDS) Serial.println(F("  (ID table full - more IDs exist than listed)"));

  // Printing ~100 IDs takes long enough at 115200 baud to overrun the MCP2515's
  // two receive buffers, so only reprint the list when a new ID turns up.
  if (seenTotal != lastListedTotal) {
    lastListedTotal = seenTotal;
    for (uint8_t i = 0; i < seenTotal; i++) {
      if (i % 8 == 0) Serial.print(F("  "));
      Serial.print(seenIds[i], HEX);
      Serial.print(' ');
      if (i % 8 == 7) Serial.println();
    }
    if (seenTotal % 8 != 0) Serial.println();
  }

  reportInterestingIds();
  windowFrames = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  Serial.println(F("\n\n=== Tesla bus scanner (listen-only, unfiltered) ==="));

#if FORCE_SPEED_INDEX >= 0
  chosenIndex = FORCE_SPEED_INDEX;
#else
  chosenIndex = sweep();
#endif

  listenAt(chosenIndex);
  Serial.print(F("\nlistening at "));
  Serial.print(SPEEDS[chosenIndex].kbps);
  Serial.println(F(" kbit/s, all IDs\n"));
  lastCensus = millis();
}

void loop() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) recordId(frame);

  if (millis() - lastCensus >= CENSUS_INTERVAL_MS) {
    lastCensus = millis();
    printCensus();
  }
}
