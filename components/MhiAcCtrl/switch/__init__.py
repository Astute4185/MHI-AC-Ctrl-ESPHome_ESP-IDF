import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import DEVICE_CLASS_SWITCH

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

Mhi3dAutoSwitch = mhi_ns.class_("Mhi3dAutoSwitch", switch.Switch, cg.Component)

CONF_VANES_3D_AUTO = "vanes_3d_auto"

ICON_3D = "mdi:video-3d"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_VANES_3D_AUTO): switch.switch_schema(
            Mhi3dAutoSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            icon=ICON_3D,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    if CONF_VANES_3D_AUTO in config:
        var = await switch.new_switch(config[CONF_VANES_3D_AUTO])
        await cg.register_component(var, config[CONF_VANES_3D_AUTO])
        await cg.register_parented(var, parent)
        cg.add(parent.set_vanes_3d_auto_switch(var))
