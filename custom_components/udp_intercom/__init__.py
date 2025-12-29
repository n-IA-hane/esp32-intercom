"""
UDP Intercom Component for ESPHome
Full-duplex audio streaming over UDP for ESP32-S3
Supports both single-bus (ES8311) and dual-bus (INMP441+MAX98357A) configurations
Includes P2P mode with mDNS discovery for device-to-device communication
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MODE
from esphome import automation, pins
from esphome.components import mdns

CODEOWNERS = ["@n-IA-hane"]
DEPENDENCIES = ["network"]
AUTO_LOAD = ["mdns"]

# =============================================================================
# Configuration Constants
# =============================================================================

# Network
CONF_SERVER_IP = "server_ip"
CONF_SERVER_PORT = "server_port"
CONF_LISTEN_PORT = "listen_port"

# I2S Mode
CONF_I2S_MODE = "i2s_mode"
CONF_SAMPLE_RATE = "sample_rate"

# Single bus I2S pins (ES8311, shared bus)
CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_MCLK_PIN = "i2s_mclk_pin"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_I2S_DOUT_PIN = "i2s_dout_pin"

# Dual bus I2S pins (INMP441 + MAX98357A, separate buses)
CONF_MIC_LRCLK_PIN = "mic_lrclk_pin"
CONF_MIC_BCLK_PIN = "mic_bclk_pin"
CONF_MIC_DIN_PIN = "mic_din_pin"
CONF_SPEAKER_LRCLK_PIN = "speaker_lrclk_pin"
CONF_SPEAKER_BCLK_PIN = "speaker_bclk_pin"
CONF_SPEAKER_DOUT_PIN = "speaker_dout_pin"

# Hardware
CONF_SPEAKER_ENABLE_PIN = "speaker_enable_pin"

# Audio Processing
CONF_AEC_ENABLED = "aec_enabled"
CONF_VOLUME_MODE = "volume_mode"

# P2P Mode
CONF_P2P_ENABLED = "p2p_enabled"
CONF_P2P_SERVICE_NAME = "p2p_service_name"
CONF_P2P_AUTO_ANSWER = "p2p_auto_answer"
CONF_P2P_TIMEOUT = "p2p_timeout"

# Triggers
CONF_ON_RING = "on_ring"
CONF_ON_CALL_START = "on_call_start"
CONF_ON_CALL_END = "on_call_end"
CONF_ON_PEER_DISCOVERED = "on_peer_discovered"
CONF_ON_PEER_LOST = "on_peer_lost"

# =============================================================================
# Component Definition
# =============================================================================

udp_intercom_ns = cg.esphome_ns.namespace("udp_intercom")
UDPIntercom = udp_intercom_ns.class_("UDPIntercom", cg.Component)

# Enums
I2SMode = udp_intercom_ns.enum("I2SMode")
I2S_MODES = {
    "single": I2SMode.I2S_MODE_SINGLE,
    "dual": I2SMode.I2S_MODE_DUAL,
}

VolumeMode = udp_intercom_ns.enum("VolumeMode")
VOLUME_MODES = {
    "auto": VolumeMode.VOLUME_MODE_AUTO,
    "hardware": VolumeMode.VOLUME_MODE_HARDWARE,
    "software": VolumeMode.VOLUME_MODE_SOFTWARE,
}

OperatingMode = udp_intercom_ns.enum("OperatingMode")
OPERATING_MODES = {
    "go2rtc": OperatingMode.OPERATING_MODE_GO2RTC,
    "p2p": OperatingMode.OPERATING_MODE_P2P,
}

# Actions
StartStreamingAction = udp_intercom_ns.class_("StartStreamingAction", automation.Action)
StopStreamingAction = udp_intercom_ns.class_("StopStreamingAction", automation.Action)
RingDoorbellAction = udp_intercom_ns.class_("RingDoorbellAction", automation.Action)
CallPeerAction = udp_intercom_ns.class_("CallPeerAction", automation.Action)
RefreshPeersAction = udp_intercom_ns.class_("RefreshPeersAction", automation.Action)

# Triggers
PeerDiscoveredTrigger = udp_intercom_ns.class_(
    "PeerDiscoveredTrigger", automation.Trigger.template(cg.std_string, cg.std_string)
)
PeerLostTrigger = udp_intercom_ns.class_(
    "PeerLostTrigger", automation.Trigger.template(cg.std_string)
)

# =============================================================================
# Validation Helpers
# =============================================================================

def validate_i2s_config(config):
    """Validate I2S pin configuration based on mode."""
    mode = config.get(CONF_I2S_MODE, "single")

    if mode == "single":
        # Single bus mode requires shared pins
        required = [CONF_I2S_LRCLK_PIN, CONF_I2S_BCLK_PIN, CONF_I2S_DIN_PIN, CONF_I2S_DOUT_PIN]
        for pin in required:
            if pin not in config:
                raise cv.Invalid(f"'{pin}' is required for single I2S bus mode")
    elif mode == "dual":
        # Dual bus mode requires separate pins for mic and speaker
        required_mic = [CONF_MIC_LRCLK_PIN, CONF_MIC_BCLK_PIN, CONF_MIC_DIN_PIN]
        required_spk = [CONF_SPEAKER_LRCLK_PIN, CONF_SPEAKER_BCLK_PIN, CONF_SPEAKER_DOUT_PIN]
        for pin in required_mic:
            if pin not in config:
                raise cv.Invalid(f"'{pin}' is required for dual I2S bus mode")
        for pin in required_spk:
            if pin not in config:
                raise cv.Invalid(f"'{pin}' is required for dual I2S bus mode")

    return config

# =============================================================================
# Configuration Schema
# =============================================================================

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UDPIntercom),

            # Network - go2rtc mode
            cv.Optional(CONF_SERVER_IP, default=""): cv.string,
            cv.Optional(CONF_SERVER_PORT, default=12345): cv.port,
            cv.Optional(CONF_LISTEN_PORT, default=12346): cv.port,

            # I2S Configuration
            cv.Optional(CONF_I2S_MODE, default="single"): cv.enum(I2S_MODES, lower=True),
            cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=8000, max=48000),

            # Single bus pins (optional based on mode)
            cv.Optional(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_MCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,

            # Dual bus pins (optional based on mode)
            cv.Optional(CONF_MIC_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_MIC_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_MIC_DIN_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_SPEAKER_LRCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SPEAKER_BCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SPEAKER_DOUT_PIN): pins.internal_gpio_output_pin_number,

            # Hardware
            cv.Optional(CONF_SPEAKER_ENABLE_PIN): pins.internal_gpio_output_pin_number,

            # Audio Processing
            cv.Optional(CONF_AEC_ENABLED, default=False): cv.boolean,
            cv.Optional(CONF_VOLUME_MODE, default="auto"): cv.enum(VOLUME_MODES, lower=True),

            # P2P Mode
            cv.Optional(CONF_P2P_ENABLED, default=False): cv.boolean,
            cv.Optional(CONF_P2P_SERVICE_NAME, default="udp-intercom"): cv.string,
            cv.Optional(CONF_P2P_AUTO_ANSWER, default=False): cv.boolean,
            cv.Optional(CONF_P2P_TIMEOUT, default="10s"): cv.positive_time_period_milliseconds,

            # Triggers
            cv.Optional(CONF_ON_RING): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_CALL_START): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_CALL_END): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_PEER_DISCOVERED): automation.validate_automation(
                {
                    cv.GenerateID(): cv.declare_id(PeerDiscoveredTrigger),
                }
            ),
            cv.Optional(CONF_ON_PEER_LOST): automation.validate_automation(
                {
                    cv.GenerateID(): cv.declare_id(PeerLostTrigger),
                }
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_i2s_config,
)


# =============================================================================
# Code Generation
# =============================================================================

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Network configuration
    if config[CONF_SERVER_IP]:
        cg.add(var.set_server_ip(config[CONF_SERVER_IP]))
    cg.add(var.set_server_port(config[CONF_SERVER_PORT]))
    cg.add(var.set_listen_port(config[CONF_LISTEN_PORT]))

    # I2S Mode
    cg.add(var.set_i2s_mode(config[CONF_I2S_MODE]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))

    # I2S Pins based on mode (check by pin presence since enum comparison is tricky)
    if CONF_MIC_LRCLK_PIN in config:
        # Dual bus mode (INMP441 + MAX98357A)
        cg.add(var.set_mic_lrclk_pin(config[CONF_MIC_LRCLK_PIN]))
        cg.add(var.set_mic_bclk_pin(config[CONF_MIC_BCLK_PIN]))
        cg.add(var.set_mic_din_pin(config[CONF_MIC_DIN_PIN]))
        cg.add(var.set_speaker_lrclk_pin(config[CONF_SPEAKER_LRCLK_PIN]))
        cg.add(var.set_speaker_bclk_pin(config[CONF_SPEAKER_BCLK_PIN]))
        cg.add(var.set_speaker_dout_pin(config[CONF_SPEAKER_DOUT_PIN]))
    else:
        # Single bus mode (ES8311)
        cg.add(var.set_i2s_lrclk_pin(config[CONF_I2S_LRCLK_PIN]))
        cg.add(var.set_i2s_bclk_pin(config[CONF_I2S_BCLK_PIN]))
        if CONF_I2S_MCLK_PIN in config:
            cg.add(var.set_i2s_mclk_pin(config[CONF_I2S_MCLK_PIN]))
        cg.add(var.set_i2s_din_pin(config[CONF_I2S_DIN_PIN]))
        cg.add(var.set_i2s_dout_pin(config[CONF_I2S_DOUT_PIN]))

    # Hardware
    if CONF_SPEAKER_ENABLE_PIN in config:
        cg.add(var.set_speaker_enable_pin(config[CONF_SPEAKER_ENABLE_PIN]))

    # Audio Processing
    cg.add(var.set_aec_enabled(config[CONF_AEC_ENABLED]))
    cg.add(var.set_volume_mode(config[CONF_VOLUME_MODE]))

    # P2P Mode
    cg.add(var.set_p2p_enabled(config[CONF_P2P_ENABLED]))
    cg.add(var.set_p2p_service_name(config[CONF_P2P_SERVICE_NAME]))
    cg.add(var.set_p2p_auto_answer(config[CONF_P2P_AUTO_ANSWER]))
    cg.add(var.set_p2p_timeout(config[CONF_P2P_TIMEOUT]))

    # Triggers
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
    if CONF_ON_PEER_DISCOVERED in config:
        for conf in config[CONF_ON_PEER_DISCOVERED]:
            trigger = cg.new_Pvariable(conf[CONF_ID], var)
            await automation.build_automation(
                trigger, [(cg.std_string, "name"), (cg.std_string, "ip")], conf
            )
    if CONF_ON_PEER_LOST in config:
        for conf in config[CONF_ON_PEER_LOST]:
            trigger = cg.new_Pvariable(conf[CONF_ID], var)
            await automation.build_automation(
                trigger, [(cg.std_string, "name")], conf
            )


# =============================================================================
# Actions
# =============================================================================

UDP_INTERCOM_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(UDPIntercom),
    }
)

CALL_PEER_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(UDPIntercom),
        cv.Required("peer"): cv.templatable(cv.string),
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


@automation.register_action(
    "udp_intercom.call_peer", CallPeerAction, CALL_PEER_ACTION_SCHEMA
)
async def call_peer_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config["peer"], args, cg.std_string)
    cg.add(var.set_peer(template_))
    return var


@automation.register_action(
    "udp_intercom.refresh_peers", RefreshPeersAction, UDP_INTERCOM_ACTION_SCHEMA
)
async def refresh_peers_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
