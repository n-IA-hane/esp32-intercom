"""Text sensor platform for I2S Audio UDP."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from . import I2SAudioUDP, i2s_audio_udp_ns

CONF_I2S_AUDIO_UDP_ID = "i2s_audio_udp_id"
CONF_AUDIO_MODE = "audio_mode"

I2SAudioUDPTextSensor = i2s_audio_udp_ns.class_(
    "I2SAudioUDPTextSensor", text_sensor.TextSensor, cg.PollingComponent
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_I2S_AUDIO_UDP_ID): cv.use_id(I2SAudioUDP),
        cv.Optional(CONF_AUDIO_MODE): text_sensor.text_sensor_schema(
            I2SAudioUDPTextSensor,
        ).extend(cv.polling_component_schema("5s")),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_I2S_AUDIO_UDP_ID])

    if CONF_AUDIO_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_AUDIO_MODE])
        await cg.register_component(sens, config[CONF_AUDIO_MODE])
        cg.add(sens.set_parent(parent))
