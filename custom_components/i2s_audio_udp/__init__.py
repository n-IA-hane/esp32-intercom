"""
I2S Audio UDP Component for ESPHome
Bidirectional audio streaming over UDP with I2S hardware support
Supports single bus (ES8311) and dual bus (INMP441+MAX98357A) configurations
Audio mode (TX/RX/Full Duplex) and I2S mode automatically deduced from pin configuration
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.const import CONF_ID

CODEOWNERS = ["@n-IA-hane"]
DEPENDENCIES = ["network"]
AUTO_LOAD = ["sensor", "text_sensor", "number"]

# Pin configuration - Single bus (ES8311)
CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_MCLK_PIN = "i2s_mclk_pin"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_I2S_DOUT_PIN = "i2s_dout_pin"

# Pin configuration - Dual bus (INMP441 + MAX98357A)
CONF_MIC_LRCLK_PIN = "mic_lrclk_pin"
CONF_MIC_BCLK_PIN = "mic_bclk_pin"
CONF_MIC_DIN_PIN = "mic_din_pin"
CONF_SPEAKER_LRCLK_PIN = "speaker_lrclk_pin"
CONF_SPEAKER_BCLK_PIN = "speaker_bclk_pin"
CONF_SPEAKER_DOUT_PIN = "speaker_dout_pin"

# Audio configuration
CONF_SAMPLE_RATE = "sample_rate"
CONF_MIC_BITS_PER_SAMPLE = "mic_bits_per_sample"
CONF_MIC_CHANNEL = "mic_channel"
CONF_MIC_GAIN = "mic_gain"
CONF_SPEAKER_ENABLE_PIN = "speaker_enable_pin"

# Network configuration - TEMPLATABLE
CONF_REMOTE_IP = "remote_ip"
CONF_REMOTE_PORT = "remote_port"
CONF_LISTEN_PORT = "listen_port"

# AEC reference
CONF_AEC_ID = "aec_id"

# Triggers
CONF_ON_START = "on_start"
CONF_ON_STOP = "on_stop"
CONF_ON_ERROR = "on_error"

# Namespace
i2s_audio_udp_ns = cg.esphome_ns.namespace("i2s_audio_udp")
I2SAudioUDP = i2s_audio_udp_ns.class_("I2SAudioUDP", cg.Component)

# Enums
MicChannel = i2s_audio_udp_ns.enum("MicChannel")
MIC_CHANNELS = {
    "left": MicChannel.MIC_CHANNEL_LEFT,
    "right": MicChannel.MIC_CHANNEL_RIGHT,
}

# Actions
StartAction = i2s_audio_udp_ns.class_("StartAction", automation.Action)
StopAction = i2s_audio_udp_ns.class_("StopAction", automation.Action)

# Forward declare esp_aec
esp_aec_ns = cg.esphome_ns.namespace("esp_aec")
EspAec = esp_aec_ns.class_("EspAec")


def validate_config(config):
    """Validate and deduce I2S mode from pin configuration."""
    single_bus_pins = [CONF_I2S_LRCLK_PIN, CONF_I2S_BCLK_PIN, CONF_I2S_DIN_PIN, CONF_I2S_DOUT_PIN, CONF_I2S_MCLK_PIN]
    dual_bus_pins = [CONF_MIC_LRCLK_PIN, CONF_MIC_BCLK_PIN, CONF_MIC_DIN_PIN,
                     CONF_SPEAKER_LRCLK_PIN, CONF_SPEAKER_BCLK_PIN, CONF_SPEAKER_DOUT_PIN]

    has_single_bus_pins = any(pin in config for pin in single_bus_pins)
    has_dual_bus_pins = any(pin in config for pin in dual_bus_pins)

    if has_single_bus_pins and has_dual_bus_pins:
        raise cv.Invalid(
            "Cannot mix single bus pins (i2s_*) with dual bus pins (mic_*/speaker_*). "
            "Use i2s_* for shared bus (ES8311) or mic_*/speaker_* for separate buses (INMP441+MAX98357A)."
        )

    if not has_single_bus_pins and not has_dual_bus_pins:
        raise cv.Invalid(
            "Must specify audio pins. Use i2s_* for single bus or mic_*/speaker_* for dual bus."
        )

    if has_dual_bus_pins:
        has_mic = CONF_MIC_DIN_PIN in config
        has_speaker = CONF_SPEAKER_DOUT_PIN in config

        if has_mic:
            for pin in [CONF_MIC_DIN_PIN, CONF_MIC_BCLK_PIN, CONF_MIC_LRCLK_PIN]:
                if pin not in config:
                    raise cv.Invalid(f"'{pin}' required when using mic")

        if has_speaker:
            for pin in [CONF_SPEAKER_DOUT_PIN, CONF_SPEAKER_BCLK_PIN, CONF_SPEAKER_LRCLK_PIN]:
                if pin not in config:
                    raise cv.Invalid(f"'{pin}' required when using speaker")

        if not has_mic and not has_speaker:
            raise cv.Invalid("Must specify mic_din_pin and/or speaker_dout_pin")

    if has_single_bus_pins:
        has_mic = CONF_I2S_DIN_PIN in config
        has_speaker = CONF_I2S_DOUT_PIN in config

        for pin in [CONF_I2S_LRCLK_PIN, CONF_I2S_BCLK_PIN]:
            if pin not in config:
                raise cv.Invalid(f"'{pin}' required for single bus mode")

        if not has_mic and not has_speaker:
            raise cv.Invalid("Must specify i2s_din_pin and/or i2s_dout_pin")

    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(I2SAudioUDP),

            cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=8000, max=48000),

            # Single bus pins
            cv.Optional(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_MCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,

            # Dual bus pins
            cv.Optional(CONF_MIC_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_MIC_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_MIC_DIN_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_SPEAKER_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SPEAKER_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SPEAKER_DOUT_PIN): pins.internal_gpio_output_pin_number,

            # Mic config
            cv.Optional(CONF_MIC_BITS_PER_SAMPLE, default=16): cv.one_of(16, 32, int=True),
            cv.Optional(CONF_MIC_CHANNEL, default="left"): cv.enum(MIC_CHANNELS, lower=True),
            cv.Optional(CONF_MIC_GAIN, default=1): cv.int_range(min=1, max=16),

            # Hardware
            cv.Optional(CONF_SPEAKER_ENABLE_PIN): pins.internal_gpio_output_pin_number,

            # Network - TEMPLATABLE
            cv.Required(CONF_REMOTE_IP): cv.templatable(cv.string),
            cv.Required(CONF_REMOTE_PORT): cv.templatable(cv.port),
            cv.Required(CONF_LISTEN_PORT): cv.templatable(cv.port),

            # AEC
            cv.Optional(CONF_AEC_ID): cv.use_id(EspAec),

            # Triggers
            cv.Optional(CONF_ON_START): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_STOP): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_config,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))

    # Single bus pins
    if CONF_I2S_LRCLK_PIN in config:
        cg.add(var.set_i2s_lrclk_pin(config[CONF_I2S_LRCLK_PIN]))
    if CONF_I2S_BCLK_PIN in config:
        cg.add(var.set_i2s_bclk_pin(config[CONF_I2S_BCLK_PIN]))
    if CONF_I2S_MCLK_PIN in config:
        cg.add(var.set_i2s_mclk_pin(config[CONF_I2S_MCLK_PIN]))
    if CONF_I2S_DIN_PIN in config:
        cg.add(var.set_i2s_din_pin(config[CONF_I2S_DIN_PIN]))
    if CONF_I2S_DOUT_PIN in config:
        cg.add(var.set_i2s_dout_pin(config[CONF_I2S_DOUT_PIN]))

    # Dual bus pins
    if CONF_MIC_LRCLK_PIN in config:
        cg.add(var.set_mic_lrclk_pin(config[CONF_MIC_LRCLK_PIN]))
    if CONF_MIC_BCLK_PIN in config:
        cg.add(var.set_mic_bclk_pin(config[CONF_MIC_BCLK_PIN]))
    if CONF_MIC_DIN_PIN in config:
        cg.add(var.set_mic_din_pin(config[CONF_MIC_DIN_PIN]))
    if CONF_SPEAKER_LRCLK_PIN in config:
        cg.add(var.set_speaker_lrclk_pin(config[CONF_SPEAKER_LRCLK_PIN]))
    if CONF_SPEAKER_BCLK_PIN in config:
        cg.add(var.set_speaker_bclk_pin(config[CONF_SPEAKER_BCLK_PIN]))
    if CONF_SPEAKER_DOUT_PIN in config:
        cg.add(var.set_speaker_dout_pin(config[CONF_SPEAKER_DOUT_PIN]))

    # Mic config
    cg.add(var.set_mic_bits_per_sample(config[CONF_MIC_BITS_PER_SAMPLE]))
    cg.add(var.set_mic_channel(config[CONF_MIC_CHANNEL]))
    cg.add(var.set_mic_gain(config[CONF_MIC_GAIN]))

    # Hardware
    if CONF_SPEAKER_ENABLE_PIN in config:
        cg.add(var.set_speaker_enable_pin(config[CONF_SPEAKER_ENABLE_PIN]))

    # Network - check if templatable or static
    if isinstance(config[CONF_REMOTE_IP], cv.Lambda):
        template = await cg.templatable(config[CONF_REMOTE_IP], [], cg.std_string)
        cg.add(var.set_remote_ip_lambda(template))
    else:
        cg.add(var.set_remote_ip(config[CONF_REMOTE_IP]))

    if isinstance(config[CONF_REMOTE_PORT], cv.Lambda):
        template = await cg.templatable(config[CONF_REMOTE_PORT], [], cg.uint16)
        cg.add(var.set_remote_port_lambda(template))
    else:
        cg.add(var.set_remote_port(config[CONF_REMOTE_PORT]))

    if isinstance(config[CONF_LISTEN_PORT], cv.Lambda):
        template = await cg.templatable(config[CONF_LISTEN_PORT], [], cg.uint16)
        cg.add(var.set_listen_port_lambda(template))
    else:
        cg.add(var.set_listen_port(config[CONF_LISTEN_PORT]))

    # AEC
    if CONF_AEC_ID in config:
        aec = await cg.get_variable(config[CONF_AEC_ID])
        cg.add(var.set_aec(aec))

    # Triggers
    if CONF_ON_START in config:
        await automation.build_automation(
            var.get_on_start_trigger(), [], config[CONF_ON_START]
        )
    if CONF_ON_STOP in config:
        await automation.build_automation(
            var.get_on_stop_trigger(), [], config[CONF_ON_STOP]
        )
    if CONF_ON_ERROR in config:
        await automation.build_automation(
            var.get_on_error_trigger(), [(cg.std_string, "error")], config[CONF_ON_ERROR]
        )


# Register actions
@automation.register_action(
    "i2s_audio_udp.start",
    StartAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(I2SAudioUDP),
    }),
)
async def start_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "i2s_audio_udp.stop",
    StopAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(I2SAudioUDP),
    }),
)
async def stop_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
