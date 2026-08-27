#include "fog_controller.h"

namespace fogcontrol {

Response FogController::emit(uint32_t nowMs) {
  Response response;
  if (!suppress_ || !prevSwitch_ || !haveUiPayload_) return response;

  // Subtraction on unsigned timestamps so a millisecond counter wrapping past
  // 2^32 behaves correctly rather than stalling the controller for 49 days.
  if (haveTransmitted_ && (nowMs - lastTransmitMs_) < config_.minTransmitIntervalMs) {
    rateLimitedCount_++;
    return response;
  }

  for (uint8_t i = 0; i < tesla::DLC_UI_VEHICLE_CONTROL; i++) response.payload[i] = lastUi_[i];
  tesla::setRearFogSwitch(response.payload, tesla::DLC_UI_VEHICLE_CONTROL, false);
  response.transmit = true;

  lastTransmitMs_ = nowMs;
  haveTransmitted_ = true;
  transmitCount_++;
  return response;
}

void FogController::onFrontLighting(const tesla::FrontLighting &front, uint32_t nowMs) {
  (void)nowMs;
  permit_ = tesla::permitsRearFog(front);
}

Response FogController::onRightLighting(const tesla::RightLighting &right, uint32_t nowMs) {
  const bool tail = tesla::isOn(right.tail);
  lampOn_ = tesla::rearFogOn(right);

  // R48 6.11.7.3: once the position lamps are switched off, the rear fog lamp
  // shall remain off until deliberately switched on again.
  if (haveTail_ && prevTail_ && !tail) suppress_ = true;

  prevTail_ = tail;
  haveTail_ = true;

  // The lamp is lit while suppression is active: correct now rather than
  // waiting for the touchscreen's next frame.
  if (lampOn_) return emit(nowMs);
  return Response();
}

Response FogController::onUiVehicleControl(const uint8_t *data, uint8_t dlc, uint32_t nowMs) {
  if (data == nullptr || dlc < tesla::DLC_UI_VEHICLE_CONTROL) return Response();

  tesla::UiVehicleControl ui{};
  if (!tesla::decodeUiVehicleControl(data, dlc, ui)) return Response();

  // The touchscreen asserts the switch continuously, so the only way the driver
  // can deliberately switch the lamp on again is to press off and then on. That
  // rising edge is the deliberate act 6.11.7.3.1 asks for.
  //
  // It only counts when the beams or front fogs are on. 6.11.7.1 forbids
  // switching the rear fog lamp ON with the position lamps alone, and this car
  // allows exactly that, so an unpermitted press leaves suppression in place.
  //
  // Note this gates the switch-on action, not continued operation: dropping
  // from dipped beam to position lamps produces no rising edge, so a lamp that
  // was legally lit keeps running, as 6.11.7.3.1 permits.
  if (haveSwitch_ && !prevSwitch_ && ui.rearFogSwitch) suppress_ = !permit_;

  prevSwitch_ = ui.rearFogSwitch;
  haveSwitch_ = true;

  for (uint8_t i = 0; i < tesla::DLC_UI_VEHICLE_CONTROL; i++) lastUi_[i] = data[i];
  haveUiPayload_ = true;

  return emit(nowMs);
}

// Only re-asserts while the lamp is actually observed lit, so it adds traffic
// exactly when suppression is failing and none at all otherwise. That matters
// because every frame this controller sends shares an ID with the touchscreen,
// and transmitting at arbitrary moments raises the chance of colliding with it.
Response FogController::tick(uint32_t nowMs) {
  if (!lampOn_) return Response();
  if (haveTransmitted_ && (nowMs - lastTransmitMs_) < config_.reassertIntervalMs) {
    return Response();
  }
  return emit(nowMs);
}

}  // namespace fogcontrol
