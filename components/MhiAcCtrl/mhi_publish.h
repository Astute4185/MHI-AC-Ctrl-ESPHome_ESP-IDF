#pragma once

#include <cstdint>

#include "MHI-AC-Ctrl-core.h"

void publish_value(CallbackInterface_Status *cb, ACStatus status, int value);
bool publish_if_changed_u8(
    CallbackInterface_Status *cb,
    ACStatus status,
    uint8_t new_value,
    uint8_t &old_value);
bool publish_if_changed_u16(
    CallbackInterface_Status *cb,
    ACStatus status,
    uint16_t new_value,
    uint16_t &old_value);
