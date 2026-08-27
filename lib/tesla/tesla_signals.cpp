#include "tesla_signals.h"

namespace tesla {
namespace {

// Start bits are taken straight from the DBC so they can be checked against
// docs/signals.md by eye.
constexpr uint8_t VCRIGHT_BRAKE = 0;
constexpr uint8_t VCRIGHT_TAIL = 2;
constexpr uint8_t VCRIGHT_TURN_SIGNAL = 4;
constexpr uint8_t VCRIGHT_REVERSE = 6;
constexpr uint8_t VCRIGHT_REAR_FOG = 8;

constexpr uint8_t UI_FRONT_FOG_SWITCH = 3;
constexpr uint8_t UI_REAR_FOG_SWITCH = 23;

constexpr uint8_t LAMP_STATE_BITS = 2;

LampState lampAt(const uint8_t *data, uint8_t dlc, uint8_t startBit) {
  return static_cast<LampState>(extractBitsLE(data, dlc, startBit, LAMP_STATE_BITS));
}

bool boolAt(const uint8_t *data, uint8_t dlc, uint8_t startBit) {
  return extractBitsLE(data, dlc, startBit, 1) != 0;
}

}  // namespace

uint8_t extractBitsLE(const uint8_t *data, uint8_t dlc, uint8_t startBit, uint8_t length) {
  if (data == nullptr || length == 0 || length > 8) return 0;
  const uint8_t byteIndex = startBit / 8;
  const uint8_t bitOffset = startBit % 8;
  if (byteIndex >= dlc) return 0;

  uint16_t window = data[byteIndex];
  if (bitOffset + length > 8) {
    if (byteIndex + 1 >= dlc) return 0;
    window |= static_cast<uint16_t>(data[byteIndex + 1]) << 8;
  }
  const uint16_t mask = static_cast<uint16_t>((1u << length) - 1u);
  return static_cast<uint8_t>((window >> bitOffset) & mask);
}

void insertBitsLE(uint8_t *data, uint8_t dlc, uint8_t startBit, uint8_t length, uint8_t value) {
  if (data == nullptr || length == 0 || length > 8) return;
  const uint8_t byteIndex = startBit / 8;
  const uint8_t bitOffset = startBit % 8;
  if (byteIndex >= dlc) return;
  if (bitOffset + length > 8 && byteIndex + 1 >= dlc) return;

  const uint16_t mask = static_cast<uint16_t>((1u << length) - 1u);
  const uint16_t placedMask = static_cast<uint16_t>(mask << bitOffset);
  const uint16_t placedValue = static_cast<uint16_t>((value & mask) << bitOffset);

  uint16_t window = data[byteIndex];
  if (bitOffset + length > 8) window |= static_cast<uint16_t>(data[byteIndex + 1]) << 8;

  window = static_cast<uint16_t>((window & ~placedMask) | placedValue);

  data[byteIndex] = static_cast<uint8_t>(window & 0xFF);
  if (bitOffset + length > 8) data[byteIndex + 1] = static_cast<uint8_t>(window >> 8);
}

bool decodeFrontLighting(const uint8_t *data, uint8_t dlc, FrontLighting &out) {
  if (data == nullptr || dlc <= VCFRONT_MODE_BYTE) return false;
  const uint8_t mode = data[VCFRONT_MODE_BYTE];
  out.raw = mode;
  out.parkLamps = (mode & VCFRONT_MODE_PARK) != 0;
  out.dippedBeam = (mode & VCFRONT_MODE_DIPPED_BEAM) != 0;
  out.mainBeam = (mode & VCFRONT_MODE_MAIN_BEAM) != 0;
  out.frontFog = (mode & VCFRONT_MODE_FRONT_FOG) != 0;
  return true;
}

bool decodeRightLighting(const uint8_t *data, uint8_t dlc, RightLighting &out) {
  if (data == nullptr || dlc < DLC_VCRIGHT_LIGHT_STATUS) return false;
  out.brake = lampAt(data, dlc, VCRIGHT_BRAKE);
  out.tail = lampAt(data, dlc, VCRIGHT_TAIL);
  out.turnSignal = lampAt(data, dlc, VCRIGHT_TURN_SIGNAL);
  out.reverse = lampAt(data, dlc, VCRIGHT_REVERSE);
  out.rearFog = lampAt(data, dlc, VCRIGHT_REAR_FOG);
  return true;
}

bool decodeUiVehicleControl(const uint8_t *data, uint8_t dlc, UiVehicleControl &out) {
  if (data == nullptr || dlc < DLC_UI_VEHICLE_CONTROL) return false;
  out.frontFogSwitch = boolAt(data, dlc, UI_FRONT_FOG_SWITCH);
  out.rearFogSwitch = boolAt(data, dlc, UI_REAR_FOG_SWITCH);
  return true;
}

bool setRearFogSwitch(uint8_t *data, uint8_t dlc, bool on) {
  if (data == nullptr || dlc < DLC_UI_VEHICLE_CONTROL) return false;
  insertBitsLE(data, dlc, UI_REAR_FOG_SWITCH, 1, on ? 1 : 0);
  return true;
}

bool setFrontFogSwitch(uint8_t *data, uint8_t dlc, bool on) {
  if (data == nullptr || dlc < DLC_UI_VEHICLE_CONTROL) return false;
  insertBitsLE(data, dlc, UI_FRONT_FOG_SWITCH, 1, on ? 1 : 0);
  return true;
}

}  // namespace tesla
