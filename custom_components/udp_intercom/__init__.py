"""
UDP Intercom Component for ESPHome
Full-duplex audio streaming over UDP for ESP32-S3 with ES8311 codec
Direct I2S management for true full duplex
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import automation, pins

CODEOWNERS = ["@n-IA-hane"]
DEPENDENCIES = []

CONF_SERVER_IP = "server_ip"
CONF_SERVER_PORT = "server_port"
CONF_LISTEN_PORT = "listen_port"
CONF_SAMPLE_RATE = "sample_rate"
CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_MCLK_PIN = "i2s_mclk_pin"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_I2S_DOUT_PIN = "i2s_dout_pin"
CONF_SPEAKER_ENABLE_PIN = "speaker_enable_pin"
CONF_ON_RING = "on_ring"
CONF_ON_CALL_START = "on_call_start"
CONF_ON_CALL_END = "on_call_end"

udp_intercom_ns = cg.esphome_ns.namespace("udp_intercom")
UDPIntercom = udp_intercom_ns.class_("UDPIntercom", cg.Component)

# Actions
StartStreamingAction = udp_intercom_ns.class_("StartStreamingAction", automation.Action)
StopStreamingAction = udp_intercom_ns.class_("StopStreamingAction", automation.Action)
RingDoorbellAction = udp_intercom_ns.class_("RingDoorbellAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UDPIntercom),
        cv.Required(CONF_SERVER_IP): cv.string,
        cv.Optional(CONF_SERVER_PORT, default=12345): cv.port,
        cv.Optional(CONF_LISTEN_PORT, default=12346): cv.port,
        cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=8000, max=48000),
        cv.Required(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_MCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SPEAKER_ENABLE_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_ON_RING): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_CALL_START): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_CALL_END): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_server_ip(config[CONF_SERVER_IP]))
    cg.add(var.set_server_port(config[CONF_SERVER_PORT]))
    cg.add(var.set_listen_port(config[CONF_LISTEN_PORT]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))

    cg.add(var.set_i2s_lrclk_pin(config[CONF_I2S_LRCLK_PIN]))
    cg.add(var.set_i2s_bclk_pin(config[CONF_I2S_BCLK_PIN]))
    cg.add(var.set_i2s_mclk_pin(config[CONF_I2S_MCLK_PIN]))
    cg.add(var.set_i2s_din_pin(config[CONF_I2S_DIN_PIN]))
    cg.add(var.set_i2s_dout_pin(config[CONF_I2S_DOUT_PIN]))

    if CONF_SPEAKER_ENABLE_PIN in config:
        cg.add(var.set_speaker_enable_pin(config[CONF_SPEAKER_ENABLE_PIN]))

    if CONF_ON_RING in config:
        await automation.build_automation(
            var.get_ring_trigger(), [], config[CONF_ON_RING]
        )
    if CONF_ON_CALL_START in config:
        await automation.build_automation(
            var.get_call_start_trigger(), [], config[CONF_ON_CALL_START]
        )
    if CONF_ON_CALL_END in config:
        await automation.build_automation(
            var.get_call_end_trigger(), [], config[CONF_ON_CALL_END]
        )


# Action schemas
UDP_INTERCOM_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(UDPIntercom),
    }
)


@automation.register_action(
    "udp_intercom.start_streaming", StartStreamingAction, UDP_INTERCOM_ACTION_SCHEMA
)
async def start_streaming_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "udp_intercom.stop_streaming", StopStreamingAction, UDP_INTERCOM_ACTION_SCHEMA
)
async def stop_streaming_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "udp_intercom.ring_doorbell", RingDoorbellAction, UDP_INTERCOM_ACTION_SCHEMA
)
async def ring_doorbell_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
