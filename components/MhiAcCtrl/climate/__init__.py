import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

MhiClimate = mhi_ns.class_("MhiClimate", climate.Climate, cg.Component)

CONF_TEMPERATURE_OFFSET = "temperature_offset"
CONF_VISUAL_MIN_TEMPERATURE = "visual_min_temperature"
CONF_VISUAL_MAX_TEMPERATURE = "visual_max_temperature"
CONF_VISUAL_TEMPERATURE_STEP = "visual_temperature_step"

MHI_OPDATA_REQ_MODE = 1 << 0

CONFIG_SCHEMA = (
    climate.climate_schema(MhiClimate)
    .extend(
        {
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Optional(CONF_TEMPERATURE_OFFSET, default=False): cv.boolean,
            cv.Optional(CONF_VISUAL_MIN_TEMPERATURE, default=18.0): cv.temperature,
            cv.Optional(CONF_VISUAL_MAX_TEMPERATURE, default=30.0): cv.temperature,
            cv.Optional(CONF_VISUAL_TEMPERATURE_STEP, default=0.5): cv.float_range(min=0.1, max=5.0),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    await cg.register_parented(var, parent)
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    cg.add(parent.set_climate_target(var))
    cg.add(parent.add_opdata_mask(MHI_OPDATA_REQ_MODE))

    cg.add(var.set_temperature_offset_enabled(config[CONF_TEMPERATURE_OFFSET]))
    cg.add(var.set_minimum_temperature(config[CONF_VISUAL_MIN_TEMPERATURE]))
    cg.add(var.set_maximum_temperature(config[CONF_VISUAL_MAX_TEMPERATURE]))
    cg.add(var.set_temperature_step(config[CONF_VISUAL_TEMPERATURE_STEP]))
