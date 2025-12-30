# CLAUDE.md - Project Documentation

## Overview

ESP32-S3 intercom system with ESPHome and Home Assistant. Supports go2rtc (WebRTC) and P2P modes.

## Components

```
custom_components/
├── i2s_audio_udp/     # Core audio: I2S + UDP streaming
├── mdns_discovery/    # P2P peer discovery via mDNS
└── esp_aec/           # Echo cancellation (ESP-AFE)
```

## Hardware

Works with any ESP32 + I2S audio. Example configs provided:

| Example | Audio Type | Config |
|---------|------------|--------|
| Xiaozhi Ball V3 | ES8311 codec (single I2S) | `intercom.yaml` |
| ESP32-S3 Mini | INMP441 + MAX98357A (dual I2S) | `intercom_mini.yaml` |

## i2s_audio_udp Component

### Single Bus (ES8311)
```yaml
i2s_audio_udp:
  id: audio
  i2s_lrclk_pin: GPIO45
  i2s_bclk_pin: GPIO9
  i2s_mclk_pin: GPIO16
  i2s_din_pin: GPIO10
  i2s_dout_pin: GPIO8
  sample_rate: 16000
  speaker_enable_pin: GPIO46
  remote_ip: !lambda 'return id(target_ip).state;'
  remote_port: 12346
  listen_port: 12346
```

### Dual Bus (INMP441 + MAX98357A)
```yaml
i2s_audio_udp:
  id: audio
  mic_lrclk_pin: GPIO3
  mic_bclk_pin: GPIO2
  mic_din_pin: GPIO4
  speaker_lrclk_pin: GPIO6
  speaker_bclk_pin: GPIO7
  speaker_dout_pin: GPIO8
  sample_rate: 16000
  mic_bits_per_sample: 32
  mic_channel: left
  mic_gain: 4
  remote_ip: !lambda 'return id(target_ip).state;'
  remote_port: 12346
  listen_port: 12346
```

### Actions & Triggers
- `i2s_audio_udp.start` / `i2s_audio_udp.stop`
- `on_start`, `on_stop`, `on_error`
- `is_streaming()`, `set_volume(float)`

### Sensors
```yaml
sensor:
  - platform: i2s_audio_udp
    tx_packets:
      name: "TX"
    rx_packets:
      name: "RX"
text_sensor:
  - platform: i2s_audio_udp
    audio_mode:
      name: "Mode"
```

## mdns_discovery Component

```yaml
mdns_discovery:
  id: discovery
  service_type: "_udp-intercom._udp"
  scan_interval: 30s
  on_peer_found:
    - lambda: 'ESP_LOGI("p2p", "Found: %s at %s", name.c_str(), ip.c_str());'
  on_peer_lost:
    - lambda: 'ESP_LOGI("p2p", "Lost: %s", name.c_str());'
```

### Methods
- `scan_now()`, `get_peers()`, `get_peer_count()`

## P2P Call Signaling

Uses ESPHome's native `udp:` component on port 12350:
- `CALL:name:ip:target` - initiate call
- `HANGUP:name:ip` - end call

## Build Commands

```bash
./venv/bin/esphome compile intercom.yaml
./venv/bin/esphome upload intercom.yaml --device 192.168.1.31
./venv/bin/esphome upload intercom_mini.yaml --device 192.168.1.18
```

## Version History

- **v4.0**: Modular architecture, mono-directional audio, P2P call/hangup signaling
- **v3.0**: P2P mode, dual hardware support
- **v2.0**: Echo cancellation
- **v1.0**: Initial go2rtc streaming
