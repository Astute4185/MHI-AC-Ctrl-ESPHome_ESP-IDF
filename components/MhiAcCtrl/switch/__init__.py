import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import DEVICE_CLASS_SWITCH

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

Mhi3dAutoSwitch = mhi_ns.class_("Mhi3dAutoSwitch", switch.Switch, cg.Component)
MhiActiveModeSwitch = mhi_ns.class_("MhiActiveModeSwitch", switch.Switch, cg.Component)
MhiOutdoorUnitSilentModeSwitch = mhi_ns.class_("MhiOutdoorUnitSilentModeSwitch", switch.Switch, cg.Component)
MhiSelfCleanSwitch = mhi_ns.class_("MhiSelfCleanSwitch", switch.Switch, cg.Component)

CONF_VANES_3D_AUTO = "vanes_3d_auto"
CONF_ACTIVE_MODE = "active_mode"
CONF_OUTDOOR_UNIT_SILENT_MODE = "outdoor_unit_silent_mode"
CONF_SELF_CLEAN = "self_clean"

ICON_3D = "mdi:video-3d"
ICON_ACTIVE_MODE = "mdi:transmit"
ICON_SILENT_MODE = "mdi:volume-off"
ICON_SELF_CLEAN = "mdi:air-filter"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_ACTIVE_MODE): switch.switch_schema(
            MhiActiveModeSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            icon=ICON_ACTIVE_MODE,
        ),
        cv.Optional(CONF_VANES_3D_AUTO): switch.switch_schema(
            Mhi3dAutoSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            icon=ICON_3D,
        ),
        cv.Optional(CONF_OUTDOOR_UNIT_SILENT_MODE): switch.switch_schema(
            MhiOutdoorUnitSilentModeSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            icon=ICON_SILENT_MODE,
        ),
        cv.Optional(CONF_SELF_CLEAN): switch.switch_schema(
            MhiSelfCleanSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            icon=ICON_SELF_CLEAN,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    if CONF_ACTIVE_MODE in config:
        var = await switch.new_switch(config[CONF_ACTIVE_MODE])
        await cg.register_component(var, config[CONF_ACTIVE_MODE])
        await cg.register_parented(var, parent)
        cg.add(parent.set_active_mode_switch(var))

    if CONF_VANES_3D_AUTO in config:
        var = await switch.new_switch(config[CONF_VANES_3D_AUTO])
        await cg.register_component(var, config[CONF_VANES_3D_AUTO])
        await cg.register_parented(var, parent)
        cg.add(parent.set_vanes_3d_auto_switch(var))

    if CONF_OUTDOOR_UNIT_SILENT_MODE in config:
        var = await switch.new_switch(config[CONF_OUTDOOR_UNIT_SILENT_MODE])
        await cg.register_component(var, config[CONF_OUTDOOR_UNIT_SILENT_MODE])
        await cg.register_parented(var, parent)

    if CONF_SELF_CLEAN in config:
        var = await switch.new_switch(config[CONF_SELF_CLEAN])
        await cg.register_component(var, config[CONF_SELF_CLEAN])
        await cg.register_parented(var, parent)
        cg.add(parent.set_self_clean_switch(var))
