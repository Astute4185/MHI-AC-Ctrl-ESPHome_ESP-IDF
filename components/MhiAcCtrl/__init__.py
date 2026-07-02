import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor
from esphome.const import CONF_ID

CONF_MHI_AC_CTRL_ID = "mhi_ac_ctrl_id"
CONF_FRAME_SIZE = "frame_size"
CONF_ROOM_TEMP_TIMEOUT = "room_temp_timeout"
CONF_VANES_UD = "initial_vertical_vanes_position"
CONF_VANES_LR = "initial_horizontal_vanes_position"
CONF_SCK_PIN = "sck_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_RX_DRIVER = "rx_driver"
CONF_TX_DRIVER = "tx_driver"
CONF_FRAME_START_IDLE_MS = "frame_start_idle_ms"
CONF_EXTERNAL_CLOCK_BYTE_GAP_US = "external_clock_byte_gap_us"
CONF_EXTERNAL_CLOCK_FRAME_GAP_US = "external_clock_frame_gap_us"
CONF_EXTERNAL_CLOCK_MIN_EDGE_GAP_US = "external_clock_min_edge_gap_us"
CONF_EXTERNAL_CLOCK_EDGE = "external_clock_edge"
CONF_EXTERNAL_CLOCK_SAMPLE_DELAY_NOPS = "external_clock_sample_delay_nops"
CONF_TX_BACKGROUND_INTERVAL_MS = "tx_background_interval_ms"
CONF_TX_BUS_WINDOW_MIN_RX_AGE_MS = "tx_bus_window_min_rx_age_ms"
CONF_TX_BUS_WINDOW_MAX_RX_AGE_MS = "tx_bus_window_max_rx_age_ms"
CONF_TX_BACKGROUND_FAILURE_BACKOFF_MS = "tx_background_failure_backoff_ms"

CONF_VANES_POSITION = "position"
CONF_TEMPERATURE = "temperature"
CONF_EXTERNAL_TEMPERATURE_SENSOR = "external_temperature_sensor"

DEPENDENCIES = ["climate"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]

mhi_ns = cg.esphome_ns.namespace("mhi_ac_ctrl")
MhiAcCtrl = mhi_ns.class_("MhiAcCtrl", cg.Component)


SetVerticalVanesAction = mhi_ns.class_("SetVerticalVanesAction", automation.Action)
SetHorizontalVanesAction = mhi_ns.class_("SetHorizontalVanesAction", automation.Action)
SetExternalRoomTemperatureAction = mhi_ns.class_("SetExternalRoomTemperatureAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MhiAcCtrl),
        cv.Optional(CONF_EXTERNAL_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FRAME_SIZE, default=20): cv.one_of(20, 33, int=True),
        cv.Optional(CONF_ROOM_TEMP_TIMEOUT, default=60): cv.int_range(min=0, max=3600),
        cv.Optional(CONF_VANES_UD): cv.int_range(min=0, max=5),
        cv.Optional(CONF_VANES_LR): cv.int_range(min=0, max=8),
        cv.Optional(CONF_SCK_PIN): cv.int_,
        cv.Optional(CONF_MOSI_PIN): cv.int_,
        cv.Optional(CONF_MISO_PIN): cv.int_,
        cv.Optional(CONF_RX_DRIVER, default="fast_gpio_rx"): cv.one_of("fast_gpio_rx", "external_clock_rx", lower=True),
        cv.Optional(CONF_TX_DRIVER, default="fast_gpio_tx"): cv.one_of("fast_gpio_tx", "none", lower=True),
        cv.Optional(CONF_FRAME_START_IDLE_MS, default=10): cv.int_range(min=1, max=50),
        cv.Optional(CONF_EXTERNAL_CLOCK_BYTE_GAP_US): cv.int_range(min=20, max=1000),
        cv.Optional(CONF_EXTERNAL_CLOCK_FRAME_GAP_US): cv.int_range(min=1000, max=50000),
        cv.Optional(CONF_EXTERNAL_CLOCK_MIN_EDGE_GAP_US): cv.int_range(min=1, max=50),
        cv.Optional(CONF_EXTERNAL_CLOCK_EDGE): cv.one_of("rising", "falling", lower=True),
        cv.Optional(CONF_EXTERNAL_CLOCK_SAMPLE_DELAY_NOPS): cv.int_range(min=0, max=32),
        cv.Optional(CONF_TX_BACKGROUND_INTERVAL_MS): cv.int_range(min=0, max=5000),
        cv.Optional(CONF_TX_BUS_WINDOW_MIN_RX_AGE_MS): cv.int_range(min=0, max=200),
        cv.Optional(CONF_TX_BUS_WINDOW_MAX_RX_AGE_MS): cv.int_range(min=0, max=500),
        cv.Optional(CONF_TX_BACKGROUND_FAILURE_BACKOFF_MS): cv.int_range(min=0, max=10000),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_frame_size(config[CONF_FRAME_SIZE]))
    cg.add(var.set_room_temp_api_timeout(config[CONF_ROOM_TEMP_TIMEOUT]))
    cg.add(var.set_rx_driver(config[CONF_RX_DRIVER]))
    cg.add(var.set_tx_driver(config[CONF_TX_DRIVER]))
    cg.add(var.set_frame_start_idle_ms(config[CONF_FRAME_START_IDLE_MS]))
    if CONF_EXTERNAL_CLOCK_BYTE_GAP_US in config:
        cg.add(var.set_external_clock_byte_gap_us(config[CONF_EXTERNAL_CLOCK_BYTE_GAP_US]))
    if CONF_EXTERNAL_CLOCK_FRAME_GAP_US in config:
        cg.add(var.set_external_clock_frame_gap_us(config[CONF_EXTERNAL_CLOCK_FRAME_GAP_US]))
    if CONF_EXTERNAL_CLOCK_MIN_EDGE_GAP_US in config:
        cg.add(var.set_external_clock_min_edge_gap_us(config[CONF_EXTERNAL_CLOCK_MIN_EDGE_GAP_US]))
    if CONF_EXTERNAL_CLOCK_EDGE in config:
        cg.add(var.set_external_clock_edge(config[CONF_EXTERNAL_CLOCK_EDGE]))
    if CONF_EXTERNAL_CLOCK_SAMPLE_DELAY_NOPS in config:
        cg.add(var.set_external_clock_sample_delay_nops(config[CONF_EXTERNAL_CLOCK_SAMPLE_DELAY_NOPS]))
    if CONF_TX_BACKGROUND_INTERVAL_MS in config:
        cg.add(var.set_tx_background_interval_ms(config[CONF_TX_BACKGROUND_INTERVAL_MS]))
    if CONF_TX_BUS_WINDOW_MIN_RX_AGE_MS in config:
        cg.add(var.set_tx_bus_window_min_rx_age_ms(config[CONF_TX_BUS_WINDOW_MIN_RX_AGE_MS]))
    if CONF_TX_BUS_WINDOW_MAX_RX_AGE_MS in config:
        cg.add(var.set_tx_bus_window_max_rx_age_ms(config[CONF_TX_BUS_WINDOW_MAX_RX_AGE_MS]))
    if CONF_TX_BACKGROUND_FAILURE_BACKOFF_MS in config:
        cg.add(var.set_tx_background_failure_backoff_ms(config[CONF_TX_BACKGROUND_FAILURE_BACKOFF_MS]))
    if CONF_EXTERNAL_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_EXTERNAL_TEMPERATURE_SENSOR])
        cg.add(var.set_external_room_temperature_sensor(sens))
    if CONF_VANES_UD in config:
        cg.add(var.set_vanes(config[CONF_VANES_UD]))
    if CONF_VANES_LR in config:
        cg.add(var.set_vanesLR(config[CONF_VANES_LR]))
    if CONF_SCK_PIN in config:
        cg.add(var.set_sck_pin(config[CONF_SCK_PIN]))
    if CONF_MOSI_PIN in config:
        cg.add(var.set_mosi_pin(config[CONF_MOSI_PIN]))
    if CONF_MISO_PIN in config:
        cg.add(var.set_miso_pin(config[CONF_MISO_PIN]))


@automation.register_action(
    "climate.mhi.set_vertical_vanes",
    SetVerticalVanesAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Required(CONF_VANES_POSITION): cv.templatable(cv.int_range(min=1, max=5)),
        }
    ),
    synchronous=True,
)
async def set_vertical_vanes_to_code(config, action_id, template_arg, args):
    mhi = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    var = cg.new_Pvariable(action_id, template_arg, mhi)
    template_ = await cg.templatable(config[CONF_VANES_POSITION], args, int)
    cg.add(var.set_position(template_))
    return var


@automation.register_action(
    "climate.mhi.set_horizontal_vanes",
    SetHorizontalVanesAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Required(CONF_VANES_POSITION): cv.templatable(cv.int_range(min=1, max=8)),
        }
    ),
    synchronous=True,
)
async def set_horizontal_vanes_to_code(config, action_id, template_arg, args):
    mhi = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    var = cg.new_Pvariable(action_id, template_arg, mhi)
    template_ = await cg.templatable(config[CONF_VANES_POSITION], args, int)
    cg.add(var.set_position(template_))
    return var


@automation.register_action(
    "climate.mhi.set_external_room_temperature",
    SetExternalRoomTemperatureAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Required(CONF_TEMPERATURE): cv.templatable(cv.float_),
        }
    ),
    synchronous=True,
)
async def set_external_room_temperature_to_code(config, action_id, template_arg, args):
    mhi = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    var = cg.new_Pvariable(action_id, template_arg, mhi)
    template_ = await cg.templatable(config[CONF_TEMPERATURE], args, float)
    cg.add(var.set_temperature(template_))
    return var
