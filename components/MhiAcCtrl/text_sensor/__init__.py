import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

from .. import CONF_MHI_AC_CTRL_ID, MhiAcCtrl, mhi_ns

MhiTextSensors = mhi_ns.class_("MhiTextSensors", cg.Component)

CONF_ERROR_CODE = "error_code"
CONF_PROTECTION_STATE = "protection_state"
CONF_ACTIVE_TRANSPORT = "active_transport"
CONF_TRANSPORT_STATE = "transport_state"
CONF_LAST_TRANSPORT_ERROR = "last_transport_error"

ICON_ALERT_OUTLINE = "mdi:shield-alert-outline"
ICON_TRANSIT_CONNECTION = "mdi:transit-connection-variant"
ICON_STATE_MACHINE = "mdi:state-machine"

MHI_OPDATA_REQ_PROTECTION_NO = 1 << 14

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MhiTextSensors),
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_ERROR_CODE): text_sensor.text_sensor_schema(
            icon=ICON_ALERT_OUTLINE,
        ),
        cv.Optional(CONF_PROTECTION_STATE): text_sensor.text_sensor_schema(
            icon=ICON_ALERT_OUTLINE,
        ),
        cv.Optional(CONF_ACTIVE_TRANSPORT): text_sensor.text_sensor_schema(
            icon=ICON_TRANSIT_CONNECTION,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_TRANSPORT_STATE): text_sensor.text_sensor_schema(
            icon=ICON_STATE_MACHINE,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_LAST_TRANSPORT_ERROR): text_sensor.text_sensor_schema(
            icon=ICON_ALERT_OUTLINE,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    await cg.register_component(var, config)
    await cg.register_parented(var, parent)

    if CONF_ERROR_CODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ERROR_CODE])
        cg.add(var.set_error_code(sens))
        cg.add(parent.set_error_code_text_sensor(sens))

    if CONF_PROTECTION_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PROTECTION_STATE])
        cg.add(var.set_protection_state(sens))
        cg.add(parent.set_protection_state_text_sensor(sens))
        cg.add(parent.add_opdata_mask(MHI_OPDATA_REQ_PROTECTION_NO))

    if CONF_ACTIVE_TRANSPORT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ACTIVE_TRANSPORT])
        cg.add(var.set_active_transport(sens))
        cg.add(parent.set_active_transport_text_sensor(sens))

    if CONF_TRANSPORT_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_TRANSPORT_STATE])
        cg.add(var.set_transport_state(sens))
        cg.add(parent.set_transport_state_text_sensor(sens))

    if CONF_LAST_TRANSPORT_ERROR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_TRANSPORT_ERROR])
        cg.add(var.set_last_transport_error(sens))
        cg.add(parent.set_last_transport_error_text_sensor(sens))
