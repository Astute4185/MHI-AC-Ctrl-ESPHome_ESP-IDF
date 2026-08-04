import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_POWER,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

MhiBinarySensors = mhi_ns.class_("MhiBinarySensors", cg.Component)

CONF_POWER = "power"
CONF_DEFROST = "defrost"
CONF_VANES_3D_AUTO_ENABLED = "vanes_3d_auto_enabled"
CONF_TRANSPORT_HEALTHY = "transport_healthy"
CONF_TRANSPORT_RECOVERY_ACTIVE = "transport_recovery_active"
CONF_TRANSPORT_SAFE_MODE = "transport_safe_mode"
CONF_OPDATA_FRESH = "opdata_fresh"

ICON_3D = "mdi:video-3d"
ICON_SNOWFLAKE_MELT = "mdi:snowflake-melt"
ICON_BACKUP_RESTORE = "mdi:backup-restore"
ICON_SHIELD_ALERT = "mdi:shield-alert"
ICON_DATABASE_CHECK = "mdi:database-check"

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
        cv.Optional(CONF_TRANSPORT_HEALTHY): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_TRANSPORT_RECOVERY_ACTIVE): binary_sensor.binary_sensor_schema(
            icon=ICON_BACKUP_RESTORE,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_TRANSPORT_SAFE_MODE): binary_sensor.binary_sensor_schema(
            icon=ICON_SHIELD_ALERT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_OPDATA_FRESH): binary_sensor.binary_sensor_schema(
            icon=ICON_DATABASE_CHECK,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
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

    if CONF_TRANSPORT_HEALTHY in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_TRANSPORT_HEALTHY])
        cg.add(var.set_transport_healthy(sens))
        cg.add(parent.set_transport_healthy_binary_sensor(sens))

    if CONF_TRANSPORT_RECOVERY_ACTIVE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_TRANSPORT_RECOVERY_ACTIVE])
        cg.add(var.set_transport_recovery_active(sens))
        cg.add(parent.set_transport_recovery_active_binary_sensor(sens))

    if CONF_TRANSPORT_SAFE_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_TRANSPORT_SAFE_MODE])
        cg.add(var.set_transport_safe_mode(sens))
        cg.add(parent.set_transport_safe_mode_binary_sensor(sens))

    if CONF_OPDATA_FRESH in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_OPDATA_FRESH])
        cg.add(var.set_opdata_fresh(sens))
        cg.add(parent.set_opdata_fresh_binary_sensor(sens))
