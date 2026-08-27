// Compliance rule set for UNECE R48 6.11.7.3, exercised on the host.
//
//   uv run pio test -e native

#include <string.h>
#include <unity.h>

#include <fog_controller.h>
#include <tesla_signals.h>

using namespace fogcontrol;

namespace {

// 0x3E3 VCRIGHT_lightStatus. Bit positions verified on the car - see
// docs/signals.md.
tesla::RightLighting right(bool tail, bool rearFog) {
  uint8_t d[2] = {0, 0};
  if (tail) tesla::insertBitsLE(d, 2, 2, 2, 1);
  if (rearFog) tesla::insertBitsLE(d, 2, 8, 2, 1);
  tesla::RightLighting out{};
  tesla::decodeRightLighting(d, 2, out);
  return out;
}

// 0x3F5 byte 4 is a switch-state bitmap on this car, with bit 7 always set.
tesla::FrontLighting front(bool dipped, bool mainBeam = false, bool frontFog = false,
                           bool park = false) {
  uint8_t d[8] = {0};
  uint8_t mode = 0x80;
  if (park) mode |= tesla::VCFRONT_MODE_PARK;
  if (dipped) mode |= tesla::VCFRONT_MODE_DIPPED_BEAM;
  if (mainBeam) mode |= tesla::VCFRONT_MODE_MAIN_BEAM;
  if (frontFog) mode |= tesla::VCFRONT_MODE_FRONT_FOG;
  d[tesla::VCFRONT_MODE_BYTE] = mode;

  tesla::FrontLighting out{};
  tesla::decodeFrontLighting(d, 8, out);
  return out;
}

// A realistic 0x273 payload, taken from the car: many unrelated fields set, so
// tests can prove the response preserves everything but the one switch bit.
void makeUi(uint8_t out[8], bool rearFogSwitch) {
  const uint8_t base[8] = {0x81, 0xE1, 0x10, 0x40, 0xC7, 0x03, 0x30, 0x11};
  memcpy(out, base, 8);
  tesla::setRearFogSwitch(out, 8, rearFogSwitch);
}

Response feedUi(FogController &c, bool rearFogSwitch, uint32_t now) {
  uint8_t ui[8];
  makeUi(ui, rearFogSwitch);
  return c.onUiVehicleControl(ui, 8, now);
}

// The touchscreen transmits at 2 Hz whatever else is happening.
uint32_t runUi(FogController &c, bool rearFogSwitch, uint32_t from, uint32_t count,
               uint32_t *transmits) {
  uint32_t t = from;
  for (uint32_t i = 0; i < count; i++) {
    if (feedUi(c, rearFogSwitch, t).transmit && transmits) (*transmits)++;
    t += 500;
  }
  return t;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_starts_suppressing_because_the_car_was_asleep() {
  FogController c;
  TEST_ASSERT_TRUE(c.suppressing());
}

void test_boot_with_switch_latched_on_suppresses_the_lamp() {
  FogController c;
  Response r = feedUi(c, true, 100);
  TEST_ASSERT_TRUE(r.transmit);
  TEST_ASSERT_EQUAL(1, (int)c.transmitCount());
}

void test_boot_with_switch_off_sends_nothing() {
  FogController c;
  TEST_ASSERT_FALSE(feedUi(c, false, 100).transmit);
  TEST_ASSERT_EQUAL(0, (int)c.transmitCount());
}

void test_response_clears_only_the_switch_bit() {
  FogController c;
  Response r = feedUi(c, true, 100);
  TEST_ASSERT_TRUE(r.transmit);

  uint8_t expected[8];
  makeUi(expected, false);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, r.payload, 8);

  tesla::UiVehicleControl ui{};
  tesla::decodeUiVehicleControl(r.payload, 8, ui);
  TEST_ASSERT_FALSE(ui.rearFogSwitch);
}

void test_deliberate_reactivation_clears_suppression() {
  FogController c;
  c.onFrontLighting(front(true), 0);              // dipped beam on: press is legal
  TEST_ASSERT_TRUE(feedUi(c, true, 0).transmit);  // suppressing from boot

  // Driver presses off, then on: the only way to re-assert a level that is
  // already held.
  TEST_ASSERT_FALSE(feedUi(c, false, 500).transmit);
  TEST_ASSERT_FALSE(feedUi(c, true, 1000).transmit);
  TEST_ASSERT_FALSE(c.suppressing());

  uint32_t transmits = 0;
  runUi(c, true, 1500, 40, &transmits);
  TEST_ASSERT_EQUAL(0, (int)transmits);
}

// R48 6.11.7.1: position lamps alone do not permit switching the rear fog on,
// and this car allows exactly that.
void test_switch_on_with_position_lamps_only_is_blocked() {
  FogController c;
  c.onFrontLighting(front(false, false, false, true), 0);  // park only
  c.onRightLighting(right(true, false), 0);

  feedUi(c, false, 0);
  TEST_ASSERT_TRUE(feedUi(c, true, 500).transmit);  // press ignored, still suppressing
  TEST_ASSERT_TRUE(c.suppressing());
  TEST_ASSERT_FALSE(c.permitted());

  uint32_t transmits = 0;
  runUi(c, true, 1000, 20, &transmits);
  TEST_ASSERT_EQUAL(20, (int)transmits);
}

void test_main_beam_alone_permits_switching_on() {
  FogController c;
  c.onFrontLighting(front(false, true), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);
  TEST_ASSERT_FALSE(c.suppressing());
}

void test_front_fog_alone_permits_switching_on() {
  FogController c;
  c.onFrontLighting(front(false, false, true), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);
  TEST_ASSERT_FALSE(c.suppressing());
}

// 6.11.7.3.1 lets a legally lit lamp keep running once the beams are dropped,
// so losing permission must not switch it off by itself.
void test_dropping_to_position_lamps_keeps_a_legal_lamp_running() {
  FogController c;
  c.onFrontLighting(front(true), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);
  TEST_ASSERT_FALSE(c.suppressing());

  c.onRightLighting(right(true, true), 600);
  c.onFrontLighting(front(false, false, false, true), 700);  // dipped beam off
  TEST_ASSERT_FALSE(c.permitted());
  TEST_ASSERT_FALSE(c.suppressing());

  uint32_t transmits = 0;
  runUi(c, true, 1000, 20, &transmits);
  TEST_ASSERT_EQUAL(0, (int)transmits);
}

void test_position_lamps_switched_off_starts_suppressing_again() {
  FogController c;
  c.onFrontLighting(front(true), 0);
  c.onRightLighting(right(true, false), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);  // deliberate switch-on clears the boot latch
  TEST_ASSERT_FALSE(c.suppressing());

  c.onRightLighting(right(true, true), 1000);
  uint32_t transmits = 0;
  runUi(c, true, 1000, 4, &transmits);
  TEST_ASSERT_EQUAL(0, (int)transmits);

  // Position lamps off: lamp goes off with them, but the switch stays asserted.
  c.onRightLighting(right(false, false), 3000);
  TEST_ASSERT_TRUE(c.suppressing());

  TEST_ASSERT_TRUE(feedUi(c, true, 3500).transmit);
}

void test_position_lamps_coming_back_on_does_not_clear_suppression() {
  FogController c;
  c.onFrontLighting(front(true), 0);
  c.onRightLighting(right(true, false), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);
  c.onRightLighting(right(false, false), 1000);  // lamps off -> suppress
  c.onRightLighting(right(true, false), 2000);   // lamps back on

  TEST_ASSERT_TRUE(c.suppressing());
  TEST_ASSERT_TRUE(feedUi(c, true, 2500).transmit);
}

void test_full_violation_sequence() {
  FogController c;
  uint32_t t = 0;
  uint32_t transmits = 0;

  // Car wakes with the switch latched on from the previous drive.
  c.onFrontLighting(front(true), t);
  c.onRightLighting(right(true, true), t);
  TEST_ASSERT_TRUE(feedUi(c, true, t).transmit);  // suppressed at boot
  t += 500;

  // Driver deliberately switches it on.
  feedUi(c, false, t);
  t += 500;
  feedUi(c, true, t);
  t += 500;
  c.onRightLighting(right(true, true), t);

  transmits = 0;
  t = runUi(c, true, t, 10, &transmits);
  TEST_ASSERT_EQUAL(0, (int)transmits);  // left alone while legitimately on

  // Lights off. Car extinguishes both lamps, but the switch stays asserted.
  c.onRightLighting(right(false, false), t);
  transmits = 0;
  t = runUi(c, true, t, 10, &transmits);
  TEST_ASSERT_EQUAL(10, (int)transmits);  // suppressed for every CID frame

  // Lights back on. Without the controller the lamp would return here.
  c.onRightLighting(right(true, false), t);
  transmits = 0;
  runUi(c, true, t, 10, &transmits);
  TEST_ASSERT_EQUAL(10, (int)transmits);
  TEST_ASSERT_TRUE(c.suppressing());
}

void test_suppression_persists_indefinitely_without_deliberate_action() {
  FogController c;
  uint32_t transmits = 0;
  runUi(c, true, 0, 400, &transmits);  // 200 seconds of touchscreen frames
  TEST_ASSERT_EQUAL(400, (int)transmits);
  TEST_ASSERT_TRUE(c.suppressing());
}

// Without this the lamp is lit from the moment the position lamps come back on
// until the touchscreen's next frame, up to 500 ms later - long enough to see.
void test_lamp_lit_while_suppressing_corrects_immediately() {
  FogController c;
  TEST_ASSERT_TRUE(feedUi(c, true, 0).transmit);
  c.onRightLighting(right(false, false), 100);

  Response r = c.onRightLighting(right(true, true), 300);
  TEST_ASSERT_TRUE(r.transmit);

  uint8_t expected[8];
  makeUi(expected, false);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, r.payload, 8);
}

void test_tick_reasserts_only_while_the_lamp_is_lit() {
  FogController c;
  TEST_ASSERT_TRUE(feedUi(c, true, 0).transmit);

  c.onRightLighting(right(true, false), 100);  // lamp already off
  TEST_ASSERT_FALSE(c.tick(1000).transmit);    // nothing to correct, stay quiet

  c.onRightLighting(right(true, true), 1100);  // lamp lit again
  TEST_ASSERT_TRUE(c.tick(1300).transmit);     // keeps pushing until it goes out
}

void test_tick_is_silent_when_not_suppressing() {
  FogController c;
  c.onFrontLighting(front(true), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);  // deliberate re-activation clears the boot latch
  TEST_ASSERT_FALSE(c.suppressing());

  c.onRightLighting(right(true, true), 600);
  TEST_ASSERT_FALSE(c.tick(1000).transmit);
  TEST_ASSERT_EQUAL(0, (int)c.transmitCount());
}

void test_rate_limit_floor() {
  Config cfg;
  cfg.minTransmitIntervalMs = 100;
  FogController c(cfg);

  TEST_ASSERT_TRUE(feedUi(c, true, 1000).transmit);
  TEST_ASSERT_FALSE(feedUi(c, true, 1050).transmit);  // too soon
  TEST_ASSERT_TRUE(feedUi(c, true, 1100).transmit);
  TEST_ASSERT_EQUAL(1, (int)c.rateLimitedCount());
}

void test_short_or_null_frames_are_rejected() {
  FogController c;
  uint8_t shortFrame[2] = {0xFF, 0xFF};
  TEST_ASSERT_FALSE(c.onUiVehicleControl(shortFrame, 2, 100).transmit);
  TEST_ASSERT_FALSE(c.onUiVehicleControl(nullptr, 8, 100).transmit);
  TEST_ASSERT_EQUAL(0, (int)c.transmitCount());
}

void test_fault_and_sna_tail_state_is_not_on() {
  FogController c;
  c.onFrontLighting(front(true), 0);
  feedUi(c, false, 0);
  feedUi(c, true, 500);  // clear the boot latch
  TEST_ASSERT_FALSE(c.suppressing());

  uint8_t d[2] = {0, 0};
  tesla::insertBitsLE(d, 2, 2, 2, 1);  // tail ON
  tesla::RightLighting on{};
  tesla::decodeRightLighting(d, 2, on);
  c.onRightLighting(on, 1000);

  tesla::insertBitsLE(d, 2, 2, 2, 3);  // tail SNA - must read as off
  tesla::RightLighting sna{};
  tesla::decodeRightLighting(d, 2, sna);
  c.onRightLighting(sna, 1500);

  TEST_ASSERT_TRUE(c.suppressing());
}

void test_lamp_state_is_tracked_for_diagnostics() {
  FogController c;
  c.onRightLighting(right(true, true), 100);
  TEST_ASSERT_TRUE(c.lampOn());
  TEST_ASSERT_TRUE(c.positionLampsOn());
  c.onRightLighting(right(false, false), 200);
  TEST_ASSERT_FALSE(c.lampOn());
  TEST_ASSERT_FALSE(c.positionLampsOn());
}

void test_timestamp_wraparound_is_handled() {
  Config cfg;
  cfg.minTransmitIntervalMs = 100;
  FogController c(cfg);

  const uint32_t nearMax = 0xFFFFFFF0UL;
  TEST_ASSERT_TRUE(feedUi(c, true, nearMax).transmit);
  // 32 ms later, but the counter has wrapped past zero.
  TEST_ASSERT_FALSE(feedUi(c, true, 0x00000010UL).transmit);
  TEST_ASSERT_TRUE(feedUi(c, true, 0x00000100UL).transmit);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_suppressing_because_the_car_was_asleep);
  RUN_TEST(test_boot_with_switch_latched_on_suppresses_the_lamp);
  RUN_TEST(test_boot_with_switch_off_sends_nothing);
  RUN_TEST(test_response_clears_only_the_switch_bit);
  RUN_TEST(test_deliberate_reactivation_clears_suppression);
  RUN_TEST(test_switch_on_with_position_lamps_only_is_blocked);
  RUN_TEST(test_main_beam_alone_permits_switching_on);
  RUN_TEST(test_front_fog_alone_permits_switching_on);
  RUN_TEST(test_dropping_to_position_lamps_keeps_a_legal_lamp_running);
  RUN_TEST(test_position_lamps_switched_off_starts_suppressing_again);
  RUN_TEST(test_position_lamps_coming_back_on_does_not_clear_suppression);
  RUN_TEST(test_full_violation_sequence);
  RUN_TEST(test_suppression_persists_indefinitely_without_deliberate_action);
  RUN_TEST(test_lamp_lit_while_suppressing_corrects_immediately);
  RUN_TEST(test_tick_reasserts_only_while_the_lamp_is_lit);
  RUN_TEST(test_tick_is_silent_when_not_suppressing);
  RUN_TEST(test_rate_limit_floor);
  RUN_TEST(test_short_or_null_frames_are_rejected);
  RUN_TEST(test_fault_and_sna_tail_state_is_not_on);
  RUN_TEST(test_lamp_state_is_tracked_for_diagnostics);
  RUN_TEST(test_timestamp_wraparound_is_handled);
  return UNITY_END();
}
