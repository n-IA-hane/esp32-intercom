"""Sensor platform for I2S Audio UDP."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, STATE_CLASS_MEASUREMENT

from . import I2SAudioUDP, i2s_audio_udp_ns

CONF_I2S_AUDIO_UDP_ID = "i2s_audio_udp_id"
CONF_TX_PACKETS = "tx_packets"
CONF_RX_PACKETS = "rx_packets"

I2SAudioUDPSensor = i2s_audio_udp_ns.class_("I2SAudioUDPSensor", sensor.Sensor, cg.PollingComponent)
SensorType = I2SAudioUDPSensor.enum("SensorType")
SENSOR_TYPE_TX = SensorType.TX_PACKETS
SENSOR_TYPE_RX = SensorType.RX_PACKETS

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_I2S_AUDIO_UDP_ID): cv.use_id(I2SAudioUDP),
        cv.Optional(CONF_TX_PACKETS): sensor.sensor_schema(
            I2SAudioUDPSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(cv.polling_component_schema("1s")),
        cv.Optional(CONF_RX_PACKETS): sensor.sensor_schema(
            I2SAudioUDPSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(cv.polling_component_schema("1s")),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_I2S_AUDIO_UDP_ID])

    if CONF_TX_PACKETS in config:
        sens = await sensor.new_sensor(config[CONF_TX_PACKETS])
        await cg.register_component(sens, config[CONF_TX_PACKETS])
        cg.add(sens.set_parent(parent))
        cg.add(sens.set_sensor_type(SENSOR_TYPE_TX))

    if CONF_RX_PACKETS in config:
        sens = await sensor.new_sensor(config[CONF_RX_PACKETS])
        await cg.register_component(sens, config[CONF_RX_PACKETS])
        cg.add(sens.set_parent(parent))
        cg.add(sens.set_sensor_type(SENSOR_TYPE_RX))
