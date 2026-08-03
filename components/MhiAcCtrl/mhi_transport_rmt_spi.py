"""ESPHome schema for RMT/SPI RX transport-specific options."""

import esphome.config_validation as cv

CONF_FRAME_GAP_US = "frame_gap_us"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_FRAME_GAP_US): cv.int_range(min=500, max=5000),
    }
)
