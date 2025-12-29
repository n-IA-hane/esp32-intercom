# CLAUDE.md - Project Documentation for AI Assistants

This document provides context for AI assistants working on this codebase.

## Project Overview

**ESPHome UDP Intercom** is a full-duplex audio intercom system for ESP32-S3 devices with ESPHome and Home Assistant integration. It supports two operating modes:
- **go2rtc mode**: Stream audio to browsers via WebRTC (requires go2rtc server)
- **P2P mode**: Direct device-to-device communication via mDNS discovery

## Repository Structure

```
esphome-intercom/
├── intercom.yaml              # Xiaozhi Ball V3 configuration (ES8311 codec, display)
├── intercom_mini.yaml         # ESP32-S3 Mini configuration (INMP441 + MAX98357A)
├── secrets.yaml               # WiFi credentials (user creates this)
├── CLAUDE.md                  # This file - AI assistant documentation
├── README.md                  # User documentation
├── readme_img/                # Images for documentation
└── custom_components/
    └── udp_intercom/
        ├── __init__.py        # ESPHome component definition (Python)
        ├── udp_intercom.h     # C++ header - classes, state machine, triggers
        └── udp_intercom.cpp   # C++ implementation - I2S, UDP, AEC, P2P
```

## Supported Hardware

### 1. Xiaozhi Ball V3 (小智球 V3)
- **Config file**: `intercom.yaml`
- **Audio**: ES8311 codec (single I2S bus, shared for mic+speaker)
- **I2S Mode**: SINGLE (simplex with ES8311 managing both directions)
- **Display**: GC9A01A 240x240 round display
- **Features**: Hardware volume control, display UI, doorbell button, touch sensor

### 2. ESP32-S3 Mini + INMP441 + MAX98357A
- **Config file**: `intercom_mini.yaml`
- **Audio**: INMP441 microphone + MAX98357A amplifier (dual I2S bus)
- **I2S Mode**: DUAL (separate buses for mic and speaker)
- **Features**: Software volume control, status LED, no display

## Key Technical Concepts

### I2S Configuration

**Single Bus (ES8311)**:
- One I2S peripheral handles both TX (speaker) and RX (mic)
- ES8311 codec configured via I2C
- 16-bit samples at 16kHz

**Dual Bus (INMP441 + MAX98357A)**:
- I2S_NUM_0 for microphone (RX only)
- I2S_NUM_1 for speaker (TX only)
- Configurable bit width (16 or 32 bit) via `mic_bits_per_sample`
- Configurable channel (left/right/stereo) via `mic_channel`
- Configurable gain (1-16x) via `mic_gain`
- MAX98357A receives 16-bit samples directly

**Universal Microphone Support** (v3.0.1+):
```yaml
mic_bits_per_sample: 32  # 16 or 32 bit
mic_channel: left        # left, right, stereo
mic_gain: 4              # 1-16 gain multiplier
```

### Operating Modes

**go2rtc Mode** (`OPERATING_MODE_GO2RTC`):
- ESP sends mic audio to go2rtc server via UDP (port 12345/12347)
- go2rtc converts to WebRTC for browser streaming
- Browser audio sent back to ESP via UDP (port 12346)
- Requires go2rtc server running on Home Assistant

**P2P Mode** (`OPERATING_MODE_P2P`):
- Devices discover each other via mDNS (`_udp-intercom._udp`)
- Direct UDP streaming between ESP devices
- No server required
- Auto-answer option for hands-free operation
- Target IP stored in text input entity

### Audio Processing

- **Sample Rate**: 16000 Hz
- **Format**: 16-bit signed PCM, mono
- **Jitter Buffer**: 8KB circular buffer with 2KB pre-buffer threshold
- **AEC**: ESP-AFE acoustic echo cancellation (optional, ~22% CPU)
- **UDP Packet Size**: 1024 bytes audio + 4 bytes sequence number

### State Machine

```
IDLE -> RINGING (on doorbell) -> STREAMING_DUPLEX (on answer)
IDLE -> STREAMING_DUPLEX (on start_streaming)
STREAMING_DUPLEX -> IDLE (on stop_streaming or timeout)
Any -> ERROR (on failure)
```

## Common Development Tasks

### Adding a New Trigger/Callback
1. Add trigger declaration in `udp_intercom.h`:
   ```cpp
   Trigger<> *get_my_trigger() { return &this->my_trigger_; }
   ```
2. Add trigger member variable:
   ```cpp
   Trigger<> my_trigger_;
   ```
3. Fire trigger in `udp_intercom.cpp`:
   ```cpp
   this->my_trigger_.trigger();
   ```
4. Register in `__init__.py`:
   ```python
   CONF_ON_MY_EVENT = "on_my_event"
   ```

### Adding a New Configuration Option
1. Add to schema in `__init__.py`
2. Add setter in `udp_intercom.h`
3. Parse and call setter in `to_code()` function
4. Use the value in `udp_intercom.cpp`

### Debugging Audio Issues
- Check I2S initialization logs: `[udp_intercom]: I2S initialized`
- Monitor packet counters: TX/RX packets sensors
- Use `ESP_LOGD` for detailed audio buffer status
- For INMP441: verify 32-bit slot width and left channel selection

### P2P Discovery Issues
- Check mDNS service registration in logs
- Verify both devices use same service name (`_udp-intercom._udp`)
- Check "Discovered Peers" sensor for peer list
- Target IP must be set (auto-fills from discovery or manual entry)

## Build Commands

```bash
# Create virtual environment (first time)
python3 -m venv venv
source venv/bin/activate
pip install esphome

# Compile only
./venv/bin/esphome compile intercom.yaml
./venv/bin/esphome compile intercom_mini.yaml

# Compile and upload via OTA
./venv/bin/esphome upload intercom.yaml --device 192.168.1.31
./venv/bin/esphome upload intercom_mini.yaml --device 192.168.1.18

# View logs
./venv/bin/esphome logs intercom.yaml --device 192.168.1.31
```

## Important Code Locations

| Feature | File | Function/Section |
|---------|------|------------------|
| I2S init (single) | udp_intercom.cpp | `init_i2s_single_bus_()` |
| I2S init (dual) | udp_intercom.cpp | `init_i2s_dual_bus_()` |
| Audio task | udp_intercom.cpp | `audio_task()` |
| P2P discovery | udp_intercom.cpp | `query_mdns_peers_()` |
| Start P2P call | udp_intercom.cpp | `start_streaming_to_peer()` |
| Jitter buffer | udp_intercom.cpp | `jitter_buffer_*` functions |
| State machine | udp_intercom.h | `IntercomState` enum |
| ESPHome config | __init__.py | `CONFIG_SCHEMA`, `to_code()` |

## Version History

- **v1.0**: Initial release - go2rtc streaming, jitter buffer
- **v2.0**: Echo cancellation (ESP-AFE AEC), improved stability
- **v3.0**: P2P mode with mDNS discovery, dual hardware support, auto-answer

## Notes for AI Assistants

1. **YAML lambdas**: ESPHome uses C++ in lambda blocks. Access components via `id(component_name)`
2. **Text input state**: Use `.state` property (e.g., `id(manual_peer_ip).state`)
3. **Select state**: Use `.state` for current value, check deprecation warnings for `current_option()`
4. **Operating mode check**: `id(intercom_component).get_operating_mode() == udp_intercom::OPERATING_MODE_P2P`
5. **Streaming switch**: Must call `start_streaming_to_peer(ip)` for P2P, `start_streaming()` for go2rtc
