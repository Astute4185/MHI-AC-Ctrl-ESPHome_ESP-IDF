"""ESPHome schema for FastGPIO transport-specific options."""

import esphome.config_validation as cv

CONF_FRAME_START_IDLE_MS = "frame_start_idle_ms"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_FRAME_START_IDLE_MS): cv.int_range(min=1, max=50),
    }
)
