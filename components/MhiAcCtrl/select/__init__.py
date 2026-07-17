import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

CONF_VERTICAL = "vertical_vanes"
CONF_VERTICAL_SELECTS = [
    "Up",
    "Up/Center",
    "Center/Down",
    "Down",
    "Swing",
]
ICON_UP_DOWN = "mdi:arrow-up-down"

CONF_HORIZONTAL = "horizontal_vanes"
CONF_HORIZONTAL_SELECTS = [
    "Left",
    "Left/Center",
    "Center",
    "Center/Right",
    "Right",
    "Wide",
    "Spot",
    "Swing",
]
ICON_LEFT_RIGHT = "mdi:arrow-left-right"

CONF_FAN_SPEED = "fan_speed"
CONF_FAN_SPEED_SELECTS = [
    "Auto",
    "Quiet",
    "Low",
    "Medium",
    "High",
]
ICON_FAN = "mdi:fan"

MhiVerticalVanesSelect = mhi_ns.class_(
    "MhiVerticalVanesSelect",
    select.Select,
    cg.Component,
)

MhiHorizontalVanesSelect = mhi_ns.class_(
    "MhiHorizontalVanesSelect",
    select.Select,
    cg.Component,
)

MhiFanSpeedSelect = mhi_ns.class_(
    "MhiFanSpeedSelect",
    select.Select,
    cg.Component,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_VERTICAL): select.select_schema(
            MhiVerticalVanesSelect,
            icon=ICON_UP_DOWN,
        ),
        cv.Optional(CONF_HORIZONTAL): select.select_schema(
            MhiHorizontalVanesSelect,
            icon=ICON_LEFT_RIGHT,
        ),
        cv.Optional(CONF_FAN_SPEED): select.select_schema(
            MhiFanSpeedSelect,
            icon=ICON_FAN,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    if CONF_VERTICAL in config:
        sel = await select.new_select(
            config[CONF_VERTICAL],
            options=CONF_VERTICAL_SELECTS,
        )
        await cg.register_component(sel, config[CONF_VERTICAL])
        await cg.register_parented(sel, parent)
        cg.add(parent.set_vertical_vanes_select(sel))

    if CONF_HORIZONTAL in config:
        sel = await select.new_select(
            config[CONF_HORIZONTAL],
            options=CONF_HORIZONTAL_SELECTS,
        )
        await cg.register_component(sel, config[CONF_HORIZONTAL])
        await cg.register_parented(sel, parent)
        cg.add(parent.set_horizontal_vanes_select(sel))

    if CONF_FAN_SPEED in config:
        sel = await select.new_select(
            config[CONF_FAN_SPEED],
            options=CONF_FAN_SPEED_SELECTS,
        )
        await cg.register_component(sel, config[CONF_FAN_SPEED])
        await cg.register_parented(sel, parent)
        cg.add(parent.set_fan_speed_select(sel))
