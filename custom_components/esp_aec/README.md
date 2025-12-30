# ESP AEC Component

Acoustic Echo Cancellation wrapper for ESP-AFE (ESP-SR).

## Requirements

- ESP32-S3 with PSRAM
- Sufficient flash (8MB+ recommended)
- ESP-IDF framework

## Features

- Wraps ESP-SR AFE acoustic echo cancellation
- Configurable sample rate and filter length
- Integrates with `i2s_audio_udp` component

## Configuration

```yaml
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_SPIRAM_MODE_OCT: y
      CONFIG_SPIRAM_SPEED_80M: y

psram:
  mode: octal
  speed: 80MHz

esp_aec:
  id: echo_canceller
  sample_rate: 16000
  filter_length: 4  # 1-10, higher = more echo removal but more CPU

# Link to audio component
i2s_audio_udp:
  id: audio_bridge
  aec_id: echo_canceller
  # ... other config
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| sample_rate | 16000 | Audio sample rate in Hz |
| filter_length | 4 | AEC filter length (1-10) |

## Notes

- AEC requires full-duplex mode (both mic and speaker)
- Uses ~22% CPU on ESP32-S3
- Frame size determined automatically by ESP-SR
- Falls back to passthrough if initialization fails
