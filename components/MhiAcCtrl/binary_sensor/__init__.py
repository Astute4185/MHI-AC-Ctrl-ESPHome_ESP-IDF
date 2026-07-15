import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_POWER

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

MhiBinarySensors = mhi_ns.class_("MhiBinarySensors", cg.Component)

CONF_POWER = "power"
CONF_DEFROST = "defrost"
CONF_VANES_3D_AUTO_ENABLED = "vanes_3d_auto_enabled"

ICON_3D = "mdi:video-3d"
ICON_SNOWFLAKE_MELT = "mdi:snowflake-melt"

MHI_OPDATA_REQ_DEFROST = 1 << 16

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MhiBinarySensors),
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_POWER): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER,
        ),
        cv.Optional(CONF_DEFROST): binary_sensor.binary_sensor_schema(
            icon=ICON_SNOWFLAKE_MELT,
        ),
        cv.Optional(CONF_VANES_3D_AUTO_ENABLED): binary_sensor.binary_sensor_schema(
            icon=ICON_3D,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    await cg.register_component(var, config)
    await cg.register_parented(var, parent)

    if CONF_POWER in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_POWER])
        cg.add(var.set_power(sens))
        cg.add(parent.set_power_binary_sensor(sens))

    if CONF_DEFROST in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DEFROST])
        cg.add(var.set_defrost(sens))
        cg.add(parent.set_defrost_binary_sensor(sens))
        cg.add(parent.add_opdata_mask(MHI_OPDATA_REQ_DEFROST))

    if CONF_VANES_3D_AUTO_ENABLED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_VANES_3D_AUTO_ENABLED])
        cg.add(var.set_vanes_3d_auto_enabled(sens))
        cg.add(parent.set_vanes_3d_auto_enabled_binary_sensor(sens))
