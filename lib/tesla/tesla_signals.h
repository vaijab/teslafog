// Decoding and encoding for the three Tesla CAN messages this project uses.
// See docs/signals.md for provenance, bit layouts and the verification status
// of every signal here.
//
// Deliberately free of Arduino dependencies so it compiles and can be tested on
// the host.

#pragma once

#include <stdint.h>

namespace tesla {

constexpr uint32_t CAN_ID_VCFRONT_LIGHTING = 0x3F5;
constexpr uint32_t CAN_ID_VCRIGHT_LIGHT_STATUS = 0x3E3;
constexpr uint32_t CAN_ID_UI_VEHICLE_CONTROL = 0x273;

constexpr uint8_t DLC_VCFRONT_LIGHTING = 8;
constexpr uint8_t DLC_VCRIGHT_LIGHT_STATUS = 2;
constexpr uint8_t DLC_UI_VEHICLE_CONTROL = 8;

enum class LampState : uint8_t {
  Off = 0,
  On = 1,
  Fault = 2,
  NotAvailable = 3,
};

// Only On counts as lit. Fault and NotAvailable are treated as not-on in both
// directions: an unknown headlamp state must not grant permission for the rear
// fog lamp, and an unknown rear fog state must not trigger an injection.
inline bool isOn(LampState state) { return state == LampState::On; }
inline bool eitherOn(LampState a, LampState b) { return isOn(a) || isOn(b); }

// 0x3F5 byte 4 on this vehicle is a bitmap of light switch states, NOT the
// pairs of 2-bit lamp fields the community DBC describes. Established by
// toggling each control and watching which bit moved - see docs/signals.md.
// Bit 7 is set in every sample observed and its meaning is unknown.
constexpr uint8_t VCFRONT_MODE_BYTE = 4;
constexpr uint8_t VCFRONT_MODE_PARK = 0x01;
constexpr uint8_t VCFRONT_MODE_DIPPED_BEAM = 0x04;
constexpr uint8_t VCFRONT_MODE_MAIN_BEAM = 0x20;
constexpr uint8_t VCFRONT_MODE_FRONT_FOG = 0x40;

struct FrontLighting {
  bool parkLamps;
  bool dippedBeam;
  bool mainBeam;
  bool frontFog;
  uint8_t raw;
};

struct RightLighting {
  LampState brake;
  LampState tail;
  LampState turnSignal;
  LampState reverse;
  LampState rearFog;
};

struct UiVehicleControl {
  bool frontFogSwitch;
  bool rearFogSwitch;
};

// Return false and leave the output untouched if the frame is too short to hold
// the message, which is how a mismatched or misidentified ID shows up.
bool decodeFrontLighting(const uint8_t *data, uint8_t dlc, FrontLighting &out);
bool decodeRightLighting(const uint8_t *data, uint8_t dlc, RightLighting &out);
bool decodeUiVehicleControl(const uint8_t *data, uint8_t dlc, UiVehicleControl &out);

// Modify a captured 0x273 payload in place. Everything the CID put in the other
// thirty-odd fields is preserved: only the one switch bit changes. Never build
// this frame from zeros - that would command seat heaters, mirrors and locks off
// as a side effect.
bool setRearFogSwitch(uint8_t *data, uint8_t dlc, bool on);
bool setFrontFogSwitch(uint8_t *data, uint8_t dlc, bool on);

inline bool rearFogOn(const RightLighting &r) { return isOn(r.rearFog); }

// R48 6.11.7.1: the rear fog lamp cannot be switched on unless the main beams,
// dipped beams or front fog lamps are switched on. Position lamps alone are
// deliberately not enough.
inline bool permitsRearFog(const FrontLighting &f) {
  return f.dippedBeam || f.mainBeam || f.frontFog;
}

// R48 6.11.7.3 keys the latch-clearing rule on the position lamps. The rear
// tail lamps are used because their state is verified on this car, whereas
// 0x3F5's park bit reflects the switch position rather than the lamps.
inline bool positionLampsOn(const RightLighting &r) { return isOn(r.tail); }

// Little-endian (Intel) field access, matching DBC @1 notation: startBit is the
// position of the field's least significant bit. Exposed for tests and for
// decoding signals not yet promoted to the structs above.
uint8_t extractBitsLE(const uint8_t *data, uint8_t dlc, uint8_t startBit, uint8_t length);
void insertBitsLE(uint8_t *data, uint8_t dlc, uint8_t startBit, uint8_t length, uint8_t value);

}  // namespace tesla
