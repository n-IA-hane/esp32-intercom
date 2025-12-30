# I2S Audio UDP Component

Bidirectional audio streaming over UDP with I2S hardware support for ESP32.

## Features

- **Auto-deduced modes**: I2S bus mode (single/dual) and audio mode (TX/RX/Full Duplex) automatically determined from pin configuration
- **Templatable network config**: `remote_ip`, `remote_port`, `listen_port` evaluated at start() time
- **Flexible microphone support**: Configurable bit width (16/32), channel (left/right), and gain (1-16x)
- **Software volume control**: Built-in volume adjustment
- **AEC integration**: Optional link to `esp_aec` component for echo cancellation
- **Platform sensors**: TX/RX packet counters, audio mode text sensor, volume number

## Configuration

```yaml
i2s_audio_udp:
  id: audio_bridge

  # Single bus (ES8311 style)
  i2s_lrclk_pin: 8
  i2s_bclk_pin: 18
  i2s_mclk_pin: 16
  i2s_din_pin: 17
  i2s_dout_pin: 15

  # OR Dual bus (INMP441 + MAX98357A style)
  mic_din_pin: 4
  mic_bclk_pin: 5
  mic_lrclk_pin: 6
  speaker_dout_pin: 15
  speaker_bclk_pin: 16
  speaker_lrclk_pin: 17

  # Mic configuration (dual bus only)
  mic_bits_per_sample: 32  # 16 or 32
  mic_channel: left        # left or right
  mic_gain: 4              # 1-16

  # Optional speaker enable
  speaker_enable_pin: 48

  # Network - can use lambdas!
  remote_ip: "192.168.1.10"
  remote_port: 12345
  listen_port: 12346

  # Or with templates:
  remote_ip: !lambda 'return id(target_ip).state;'

  # Optional AEC
  aec_id: my_aec

  # Triggers
  on_start:
    - logger.log: "Started"
  on_stop:
    - logger.log: "Stopped"
  on_error:
    - logger.log:
        format: "Error: %s"
        args: [error.c_str()]
```

## Actions

```yaml
# Start streaming
- i2s_audio_udp.start:
    id: audio_bridge

# Stop streaming
- i2s_audio_udp.stop:
    id: audio_bridge
```

## Sensors

```yaml
sensor:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    tx_packets:
      name: "TX Packets"
    rx_packets:
      name: "RX Packets"

text_sensor:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    audio_mode:
      name: "Audio Mode"

number:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    volume:
      name: "Volume"
```

## Lambda Access

```yaml
- lambda: |-
    if (id(audio_bridge).is_streaming()) {
      ESP_LOGI("test", "TX: %d, RX: %d",
               id(audio_bridge).get_tx_packets(),
               id(audio_bridge).get_rx_packets());
    }
```
