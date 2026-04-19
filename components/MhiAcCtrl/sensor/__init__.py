import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_HERTZ,
    UNIT_AMPERE,
    UNIT_KILOWATT_HOURS,
    ICON_THERMOMETER,
    ICON_FAN,
)
from .. import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

mhi_ns = cg.esphome_ns.namespace('mhi')
MhiSensors = mhi_ns.class_('MhiSensors', cg.Component)

CONF_ERROR_CODE = "error_code"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_RETURN_AIR_TEMPERATURE = "return_air_temperature"
CONF_OUTDOOR_UNIT_FAN_SPEED = "outdoor_unit_fan_speed"
CONF_INDOOR_UNIT_FAN_SPEED = "indoor_unit_fan_speed"
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_INDOOR_UNIT_TOTAL_RUN_TIME = "indoor_unit_total_run_time"
CONF_COMPRESSOR_TOTAL_RUN_TIME = "compressor_total_run_time"
CONF_CURRENT_POWER = "current_power"
CONF_VANES_POS = "vanes_pos"
CONF_VANES_POS_OLD = "vanes_pos_old"
CONF_ENERGY_USED = "energy_used"
CONF_INDOOR_UNIT_THI_R1 = "indoor_unit_thi_r1"
CONF_INDOOR_UNIT_THI_R2 = "indoor_unit_thi_r2"
CONF_INDOOR_UNIT_THI_R3 = "indoor_unit_thi_r3"
CONF_OUTDOOR_UNIT_THO_R1 = "outdoor_unit_tho_r1"
CONF_OUTDOOR_UNIT_EXPANSION_VALVE = "outdoor_unit_expansion_valve"
CONF_OUTDOOR_UNIT_DISCHARGE_PIPE = "outdoor_unit_discharge_pipe"
CONF_OUTDOOR_UNIT_DISCHARGE_PIPE_SUPER_HEAT = "outdoor_unit_discharge_pipe_super_heat"
CONF_PROTECTION_STATE_NUMBER = "protection_state_number"
CONF_VANESLR_POS = "vanesLR_pos"
CONF_VANESLR_POS_OLD = "vanesLR_pos_old"
CONF_THREED_AUTO = "threeD_auto_enabled"

ICON_SINE = "mdi:sine-wave"
ICON_CLOCK = "mdi:clock"
ICON_CURRENT = "mdi:current-ac"
ICON_AIR = "mdi:air-filter"
ICON_VALVE = "mdi:valve"
ICON_LIGHTNING_BOLT = "mdi:lightning-bolt"
ICON_ALERT_OUTLINE = "mdi:shield-alert-outline"
UNIT_PULSE = "pulse"

MHI_OPDATA_REQ_RETURN_AIR = 1 << 2
MHI_OPDATA_REQ_THI_R1 = 1 << 3
MHI_OPDATA_REQ_THI_R2 = 1 << 4
MHI_OPDATA_REQ_THI_R3 = 1 << 5
MHI_OPDATA_REQ_IU_FANSPEED = 1 << 6
MHI_OPDATA_REQ_TOTAL_IU_RUN = 1 << 7
MHI_OPDATA_REQ_OUTDOOR = 1 << 8
MHI_OPDATA_REQ_THO_R1 = 1 << 9
MHI_OPDATA_REQ_COMP = 1 << 10
MHI_OPDATA_REQ_TD = 1 << 11
MHI_OPDATA_REQ_CT = 1 << 12
MHI_OPDATA_REQ_TDSH = 1 << 13
MHI_OPDATA_REQ_PROTECTION_NO = 1 << 14
MHI_OPDATA_REQ_OU_FANSPEED = 1 << 15
MHI_OPDATA_REQ_DEFROST = 1 << 16
MHI_OPDATA_REQ_TOTAL_COMP_RUN = 1 << 17
MHI_OPDATA_REQ_OU_EEV1 = 1 << 18
MHI_OPDATA_REQ_KWH = 1 << 19

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MhiSensors),
    cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
    cv.Optional(CONF_ERROR_CODE): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_RETURN_AIR_TEMPERATURE): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_OUTDOOR_UNIT_FAN_SPEED): sensor.sensor_schema(
        icon=ICON_FAN,
    ),
    cv.Optional(CONF_INDOOR_UNIT_FAN_SPEED): sensor.sensor_schema(
        icon=ICON_FAN,
    ),
    cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(
        icon=ICON_SINE,
        unit_of_measurement=UNIT_HERTZ,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_FREQUENCY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_INDOOR_UNIT_TOTAL_RUN_TIME): sensor.sensor_schema(
        icon=ICON_CLOCK,
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=1,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    cv.Optional(CONF_COMPRESSOR_TOTAL_RUN_TIME): sensor.sensor_schema(
        icon=ICON_CLOCK,
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=1,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    cv.Optional(CONF_CURRENT_POWER): sensor.sensor_schema(
        icon=ICON_CURRENT,
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_VANES_POS): sensor.sensor_schema(
        icon=ICON_AIR,
    ),
    cv.Optional(CONF_ENERGY_USED): sensor.sensor_schema(
        icon=ICON_LIGHTNING_BOLT,
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    cv.Optional(CONF_INDOOR_UNIT_THI_R1): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_INDOOR_UNIT_THI_R2): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_INDOOR_UNIT_THI_R3): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_OUTDOOR_UNIT_THO_R1): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_OUTDOOR_UNIT_EXPANSION_VALVE): sensor.sensor_schema(
        icon=ICON_VALVE,
        unit_of_measurement=UNIT_PULSE,
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_OUTDOOR_UNIT_DISCHARGE_PIPE): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_OUTDOOR_UNIT_DISCHARGE_PIPE_SUPER_HEAT): sensor.sensor_schema(
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_PROTECTION_STATE_NUMBER): sensor.sensor_schema(
        icon=ICON_ALERT_OUTLINE,
    ),
    cv.Optional(CONF_VANESLR_POS): sensor.sensor_schema(
        icon=ICON_AIR,
    )
})


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    mhi = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])

    await cg.register_component(var, config)
    await cg.register_parented(var, mhi)

    opdata_mask = 0

    if CONF_ERROR_CODE in config:
        conf = config[CONF_ERROR_CODE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_error_code(sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        conf = config[CONF_OUTDOOR_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_temperature(sens))
        opdata_mask |= MHI_OPDATA_REQ_OUTDOOR
    if CONF_RETURN_AIR_TEMPERATURE in config:
        conf = config[CONF_RETURN_AIR_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_return_air_temperature(sens))
        opdata_mask |= MHI_OPDATA_REQ_RETURN_AIR
    if CONF_OUTDOOR_UNIT_FAN_SPEED in config:
        conf = config[CONF_OUTDOOR_UNIT_FAN_SPEED]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_unit_fan_speed(sens))
        opdata_mask |= MHI_OPDATA_REQ_OU_FANSPEED
    if CONF_INDOOR_UNIT_FAN_SPEED in config:
        conf = config[CONF_INDOOR_UNIT_FAN_SPEED]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_unit_fan_speed(sens))
        opdata_mask |= MHI_OPDATA_REQ_IU_FANSPEED
    if CONF_COMPRESSOR_FREQUENCY in config:
        conf = config[CONF_COMPRESSOR_FREQUENCY]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compressor_frequency(sens))
        opdata_mask |= MHI_OPDATA_REQ_COMP
    if CONF_INDOOR_UNIT_TOTAL_RUN_TIME in config:
        conf = config[CONF_INDOOR_UNIT_TOTAL_RUN_TIME]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_unit_total_run_time(sens))
        opdata_mask |= MHI_OPDATA_REQ_TOTAL_IU_RUN
    if CONF_COMPRESSOR_TOTAL_RUN_TIME in config:
        conf = config[CONF_COMPRESSOR_TOTAL_RUN_TIME]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compressor_total_run_time(sens))
        opdata_mask |= MHI_OPDATA_REQ_TOTAL_COMP_RUN
    if CONF_CURRENT_POWER in config:
        conf = config[CONF_CURRENT_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_current_power(sens))
        opdata_mask |= MHI_OPDATA_REQ_CT
    if CONF_VANES_POS in config:
        conf = config[CONF_VANES_POS]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_vanes_pos(sens))
    if CONF_ENERGY_USED in config:
        conf = config[CONF_ENERGY_USED]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_energy_used(sens))
        opdata_mask |= MHI_OPDATA_REQ_KWH
    if CONF_INDOOR_UNIT_THI_R1 in config:
        conf = config[CONF_INDOOR_UNIT_THI_R1]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_unit_thi_r1(sens))
        opdata_mask |= MHI_OPDATA_REQ_THI_R1
    if CONF_INDOOR_UNIT_THI_R2 in config:
        conf = config[CONF_INDOOR_UNIT_THI_R2]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_unit_thi_r2(sens))
        opdata_mask |= MHI_OPDATA_REQ_THI_R2
    if CONF_INDOOR_UNIT_THI_R3 in config:
        conf = config[CONF_INDOOR_UNIT_THI_R3]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_unit_thi_r3(sens))
        opdata_mask |= MHI_OPDATA_REQ_THI_R3
    if CONF_OUTDOOR_UNIT_THO_R1 in config:
        conf = config[CONF_OUTDOOR_UNIT_THO_R1]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_unit_tho_r1(sens))
        opdata_mask |= MHI_OPDATA_REQ_THO_R1
    if CONF_OUTDOOR_UNIT_EXPANSION_VALVE in config:
        conf = config[CONF_OUTDOOR_UNIT_EXPANSION_VALVE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_unit_expansion_valve(sens))
        opdata_mask |= MHI_OPDATA_REQ_OU_EEV1
    if CONF_OUTDOOR_UNIT_DISCHARGE_PIPE in config:
        conf = config[CONF_OUTDOOR_UNIT_DISCHARGE_PIPE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_unit_discharge_pipe(sens))
        opdata_mask |= MHI_OPDATA_REQ_TD
    if CONF_OUTDOOR_UNIT_DISCHARGE_PIPE_SUPER_HEAT in config:
        conf = config[CONF_OUTDOOR_UNIT_DISCHARGE_PIPE_SUPER_HEAT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_unit_discharge_pipe_super_heat(sens))
        opdata_mask |= MHI_OPDATA_REQ_TDSH
    if CONF_PROTECTION_STATE_NUMBER in config:
        conf = config[CONF_PROTECTION_STATE_NUMBER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_protection_state_number(sens))
        opdata_mask |= MHI_OPDATA_REQ_PROTECTION_NO
    if CONF_VANESLR_POS in config:
        conf = config[CONF_VANESLR_POS]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_vanesLR_pos(sens))

    if opdata_mask != 0:
        cg.add(mhi.add_opdata_mask(opdata_mask))