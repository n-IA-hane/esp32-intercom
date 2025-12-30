"""Number platform for I2S Audio UDP volume control."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID, CONF_UNIT_OF_MEASUREMENT

from . import I2SAudioUDP, i2s_audio_udp_ns

CONF_I2S_AUDIO_UDP_ID = "i2s_audio_udp_id"
CONF_VOLUME = "volume"

I2SAudioUDPVolume = i2s_audio_udp_ns.class_(
    "I2SAudioUDPVolume", number.Number, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_I2S_AUDIO_UDP_ID): cv.use_id(I2SAudioUDP),
        cv.Optional(CONF_VOLUME): number.number_schema(
            I2SAudioUDPVolume,
        ).extend({
            cv.Optional("min_value", default=0): cv.float_,
            cv.Optional("max_value", default=100): cv.float_,
            cv.Optional("step", default=5): cv.float_,
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default="%"): cv.string,
        }),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_I2S_AUDIO_UDP_ID])

    if CONF_VOLUME in config:
        conf = config[CONF_VOLUME]
        var = await number.new_number(
            conf,
            min_value=conf.get("min_value", 0),
            max_value=conf.get("max_value", 100),
            step=conf.get("step", 5),
        )
        await cg.register_component(var, conf)
        cg.add(var.set_parent(parent))
