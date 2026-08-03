"""ESPHome schema for external-clock transport-specific options."""

import esphome.config_validation as cv

# The current implementation has no public driver-specific tuning. Keeping an
# explicit schema gives the driver a stable namespace for future options.
CONFIG_SCHEMA = cv.Schema({})
