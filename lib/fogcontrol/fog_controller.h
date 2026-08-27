// UNECE R48 6.11.7.3 compliance logic for the rear fog lamp.
//
// Behaviour established empirically on the target car (see docs/signals.md):
//
//   * The touchscreen holds UI_rearFogSwitch = 1 continuously at 2 Hz. It is a
//     level, not an event.
//   * The body controller follows that level with ~5 ms latency and accepts
//     frames that did not originate from the touchscreen - no rolling counter,
//     no checksum, no source validation.
//   * When the position lamps go off the lamp goes off, but the switch stays
//     set, so the lamp returns the moment the lamps come back on. That is the
//     violation.
//   * The dash tell-tale follows the real lamp state, so suppressing the lamp
//     also clears the indicator. Lamp off and tell-tale off together, which is
//     what makes this approach compliant rather than merely dark.
//
// Consequently the action is not a one-off injection but sustained
// suppression: while suppressing, UI_rearFogSwitch is held clear.
//
// Three things can produce a frame, because reacting only to the touchscreen
// leaves a visible flash at the moment the position lamps come back on - the
// body controller lights the lamp immediately from the switch value it already
// holds, and the next touchscreen frame may be 500 ms away:
//
//   onUiVehicleControl  answer the touchscreen, the steady-state case
//   onRightLighting     correct as soon as the lamp is observed lit
//   tick                periodic re-assert, bounding the worst case
//
// 6.11.7.1 is not enforced here - the car already refuses to light the rear fog
// lamp with the headlights off.
//
// Pure logic: no I/O, no clock of its own, so the whole rule set is testable on
// the host.

#pragma once

#include <stdint.h>

#include <tesla_signals.h>

namespace fogcontrol {

struct Config {
  // Hard floor between frames, whatever asks for one.
  uint32_t minTransmitIntervalMs = 40;
  // While suppressing with the switch asserted, re-assert at least this often
  // so the lamp is never lit for longer than this at a transition.
  uint32_t reassertIntervalMs = 100;
};

struct Response {
  bool transmit = false;
  // Valid when transmit is true: the last 0x273 payload seen from the
  // touchscreen with UI_rearFogSwitch cleared, every other field untouched.
  uint8_t payload[tesla::DLC_UI_VEHICLE_CONTROL] = {0};
};

class FogController {
 public:
  FogController() = default;
  explicit FogController(const Config &config) : config_(config) {}

  // Front lighting. Only decides whether a switch-on is permitted at all; a
  // change here never switches the lamp off by itself, because 6.11.7.3.1
  // explicitly allows the lamp to keep running once the beams are dropped.
  void onFrontLighting(const tesla::FrontLighting &front, uint32_t nowMs);

  // Position lamps and lamp state. Drives the suppression latch, and corrects
  // immediately if the lamp is seen lit while suppressing.
  Response onRightLighting(const tesla::RightLighting &right, uint32_t nowMs);

  // Call straight from the CAN receive path and transmit the result at once:
  // the lamp stays lit for exactly this round trip.
  Response onUiVehicleControl(const uint8_t *data, uint8_t dlc, uint32_t nowMs);

  // Call from the main loop. Produces the periodic re-assert.
  Response tick(uint32_t nowMs);

  bool suppressing() const { return suppress_; }
  bool permitted() const { return permit_; }
  bool lampOn() const { return lampOn_; }
  bool positionLampsOn() const { return prevTail_; }
  bool switchAsserted() const { return prevSwitch_; }
  bool armed() const { return haveUiPayload_; }
  uint32_t transmitCount() const { return transmitCount_; }
  uint32_t rateLimitedCount() const { return rateLimitedCount_; }

 private:
  // Builds a frame if suppression is active, the switch is asserted, a payload
  // has been captured, and the rate floor allows it.
  Response emit(uint32_t nowMs);

  Config config_;

  // Starts suppressing. The controller only runs when the car is awake, so a
  // fresh start means the car was asleep, which means the position lamps were
  // off - exactly the condition 6.11.7.3 latches on.
  bool suppress_ = true;

  bool haveTail_ = false;
  bool prevTail_ = false;
  bool lampOn_ = false;

  // Defaults to not permitted, so a switch-on seen before any front lighting
  // frame is treated as illegal rather than waved through. 0x3F5 arrives at
  // 10 Hz, so this only matters for the first few milliseconds after a start.
  // If it stops arriving altogether the caller is expected to go passive.
  bool permit_ = false;

  bool haveSwitch_ = false;
  bool prevSwitch_ = false;

  uint8_t lastUi_[tesla::DLC_UI_VEHICLE_CONTROL] = {0};
  bool haveUiPayload_ = false;

  bool haveTransmitted_ = false;
  uint32_t lastTransmitMs_ = 0;
  uint32_t transmitCount_ = 0;
  uint32_t rateLimitedCount_ = 0;
};

}  // namespace fogcontrol
