#include "mhi_publish.h"

void publish_value(CallbackInterface_Status *cb, ACStatus status, int value) {
  if (cb == nullptr) {
    return;
  }
  cb->cbiStatusFunction(status, value);
}

bool publish_if_changed_u8(
    CallbackInterface_Status *cb,
    ACStatus status,
    uint8_t new_value,
    uint8_t &old_value) {
  if (new_value == old_value) {
    return false;
  }

  old_value = new_value;
  publish_value(cb, status, static_cast<int>(new_value));
  return true;
}

bool publish_if_changed_u16(
    CallbackInterface_Status *cb,
    ACStatus status,
    uint16_t new_value,
    uint16_t &old_value) {
  if (new_value == old_value) {
    return false;
  }

  old_value = new_value;
  publish_value(cb, status, static_cast<int>(new_value));
  return true;
}
