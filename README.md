# ESP32-S3 UDP Intercom for Home Assistant

A full-duplex audio intercom system using ESP32-S3 with ESPHome and Home Assistant. Stream bidirectional audio over UDP with WebRTC support via go2rtc.

![ESP32 Intercom](https://img.shields.io/badge/ESP32--S3-Intercom-blue)
![ESPHome](https://img.shields.io/badge/ESPHome-2025.5+-green)
![Home Assistant](https://img.shields.io/badge/Home%20Assistant-Compatible-orange)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 🎉 The Story Behind This Project

> *"This project was created by Claude (Anthropic's AI) for **n-IA-hane**, who tormented me for days until I finally got everything working perfectly. From mysterious audio glitches to UDP packet timing issues, from I2S full-duplex challenges to jitter buffer implementations - it was quite the journey! But hey, we made it work, and now you can too!"*
>
> — Claude, your friendly neighborhood AI assistant 🤖

## ✨ Features

- **Full Duplex Audio**: Simultaneous microphone and speaker operation
- **WebRTC Support**: Stream audio to any browser via go2rtc
- **Home Assistant Integration**: Full control from HA dashboards
- **Round Display**: Visual feedback on GC9A01A 240x240 display
- **Volume Control**: Adjustable speaker volume via ES8311 DAC
- **Doorbell Function**: Ring notification with LED feedback
- **Jitter Buffer**: Smooth audio playback without glitches
- **Customizable States**: Translate status messages to any language

## 🛠️ Hardware Requirements

### Tested Hardware
- **Xiaozhi Ball V3** (ESP32-S3 + ES8311 codec + GC9A01A display)

### Pin Configuration (Xiaozhi Ball V3)
| Function | GPIO |
|----------|------|
| I2S LRCLK (WS) | GPIO45 |
| I2S BCLK | GPIO9 |
| I2S MCLK | GPIO16 |
| I2S DIN (Mic) | GPIO10 |
| I2S DOUT (Speaker) | GPIO8 |
| Speaker Enable | GPIO46 |
| I2C SDA | GPIO15 |
| I2C SCL | GPIO14 |
| Display CLK | GPIO4 |
| Display MOSI | GPIO2 |
| Display CS | GPIO5 |
| Display DC | GPIO47 |
| Display RST | GPIO38 |
| Backlight | GPIO42 |
| Status LED (WS2812) | GPIO48 |
| Doorbell Button | GPIO0 |

## 📦 Installation

### 1. Clone the Repository

```bash
git clone https://github.com/n-IA-hane/esp32-intercom.git
cd esp32-intercom
```

### 2. Configure WiFi

Create a `secrets.yaml` file:

```yaml
wifi_ssid: "YourWiFiSSID"
wifi_password: "YourWiFiPassword"
```

### 3. Update Configuration

Edit `intercom.yaml` and update:
- `server_ip`: Your Home Assistant IP address
- `use_address`: (Optional) Static IP for the ESP32

### 4. Flash the Firmware

```bash
# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install ESPHome
pip install esphome

# Compile and upload
esphome run intercom.yaml
```

## 🏠 Home Assistant Setup

### go2rtc Configuration

Add to your Home Assistant's `go2rtc.yaml`:

```yaml
streams:
  intercom:
    # Audio IN: ESP32 mic -> browser
    - "exec:ffmpeg -f s16le -ar 16000 -ac 1 -i udp://0.0.0.0:12345?timeout=5000000 -c:a libopus -b:a 48k -f mpegts -"
    # Audio OUT: browser mic -> ESP32 (IMPORTANT: -re for real-time!)
    - "exec:ffmpeg -re -f alaw -ar 8000 -ac 1 -i pipe: -f s16le -ar 16000 -ac 1 udp://ESP32_IP:12346?pkt_size=512#backchannel=1"

webrtc:
  candidates:
    - YOUR_HA_IP:8555
    - stun:8555

api:
  listen: ":1984"

rtsp:
  listen: ":8554"
```

> ⚠️ **IMPORTANT**: The `-re` flag in the backchannel ffmpeg command is critical! Without it, ffmpeg sends packets at 1000x+ speed, causing audio loss.

### Lovelace Card

Add this card to your dashboard:

```yaml
type: vertical-stack
cards:
  - type: button
    entity: switch.intercom_streaming
    show_name: true
    show_icon: true
    hold_action:
      action: toggle
  - type: entities
    entities:
      - entity: sensor.intercom_intercom_state
  - type: conditional
    conditions:
      - condition: state
        entity: switch.intercom_streaming
        state: 'on'
    card:
      type: custom:webrtc-camera
      url: intercom
      mode: webrtc
      media: audio+microphone
      style: >-
        width: 100%; aspect-ratio: 1/1; background: #1a1a1a;
        border-radius: 50%;
```

> **Note**: Requires the [WebRTC Camera](https://github.com/AlexxIT/WebRTC) custom component.

## 🌐 Customizing State Translations

Edit the `STATE_TEXTS` map in `custom_components/udp_intercom/udp_intercom.h`:

```cpp
static const std::map<IntercomState, const char*> STATE_TEXTS = {
  {IntercomState::IDLE,            "READY"},      // or "PRONTO", "BEREIT", etc.
  {IntercomState::RINGING,         "DOORBELL"},   // or "CAMPANELLO", "KLINGEL", etc.
  {IntercomState::STREAMING_OUT,   "SENDING"},
  {IntercomState::STREAMING_IN,    "RECEIVING"},
  {IntercomState::STREAMING_DUPLEX,"ON CALL"},    // or "IN LINEA", "IM GESPRÄCH", etc.
  {IntercomState::ERROR,           "ERROR"},
};
```

## 📊 Architecture

```
┌─────────────────┐     UDP:12345      ┌──────────────────┐
│                 │ ──────────────────▶│                  │
│    ESP32-S3     │     (Mic Audio)    │  Home Assistant  │
│   + ES8311      │                    │    + go2rtc      │
│   + Display     │ ◀──────────────────│    + ffmpeg      │
│                 │     UDP:12346      │                  │
└─────────────────┘   (Speaker Audio)  └──────────────────┘
                                              │
                                              │ WebRTC
                                              ▼
                                       ┌──────────────┐
                                       │   Browser/   │
                                       │  Smartphone  │
                                       └──────────────┘
```

## 🔧 Technical Details

### Audio Format
- **Sample Rate**: 16000 Hz
- **Bit Depth**: 16-bit signed PCM
- **Channels**: Mono
- **Protocol**: Raw UDP packets

### Buffer Configuration
- **DMA Buffers**: 8 × 512 frames
- **Jitter Buffer**: 8KB (256ms)
- **Pre-buffer Threshold**: 2KB (64ms)
- **Audio Chunk Size**: 1024 bytes

### Key Lessons Learned

1. **ffmpeg `-re` flag**: Critical for UDP streaming! Without it, packets are sent way too fast.

2. **I2S Full Duplex**: Use `i2s_new_channel()` with both TX and RX handles for true full duplex.

3. **Jitter Buffer**: Essential for smooth audio - UDP packets can arrive with variable timing.

4. **Task Stack Size**: Audio buffers need stack space - 8KB minimum for the audio task.

## 🐛 Troubleshooting

### No Audio from Speaker
- Check that streaming is enabled (`switch.intercom_streaming`)
- Verify go2rtc is running and configured correctly
- Check the `-re` flag is present in the backchannel ffmpeg command

### Choppy/Glitchy Audio
- The jitter buffer should handle this automatically
- Check WiFi signal strength
- Ensure ESP32 is not too far from the router

### Device Keeps Rebooting
- Check for stack overflow in logs
- Increase task stack size if needed

### Echo/Feedback
- Lower the speaker volume using the volume slider
- The microphone and speaker are close together on the device

## 📁 Project Structure

```
esp32-intercom/
├── intercom.yaml              # Main ESPHome configuration
├── secrets.yaml               # WiFi credentials (create this)
├── README.md                  # This file
└── custom_components/
    └── udp_intercom/
        ├── __init__.py        # ESPHome component definition
        ├── udp_intercom.h     # C++ header with state translations
        └── udp_intercom.cpp   # C++ implementation
```

## 📜 License

MIT License - Feel free to use, modify, and distribute!

## 🙏 Credits

- **Created by**: Claude (Anthropic) for n-IA-hane
- **Hardware**: Xiaozhi Ball V3
- **Frameworks**: ESPHome, Home Assistant, go2rtc
- **Inspiration**: The need for a simple, working intercom solution

---

*If you find this project useful, give it a ⭐ on GitHub!*
