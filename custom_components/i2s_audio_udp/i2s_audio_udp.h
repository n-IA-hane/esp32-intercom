#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"

#include <string>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

// Forward declare AEC
namespace esphome {
namespace esp_aec {
class EspAec;
}
}

namespace esphome {
namespace i2s_audio_udp {

// I2S bus configuration mode (auto-deduced from pins)
enum I2SBusMode : uint8_t {
  I2S_BUS_SINGLE,  // Shared bus for mic and speaker (ES8311)
  I2S_BUS_DUAL,    // Separate buses (INMP441 + MAX98357A)
};

// Audio direction mode (auto-deduced from pins)
enum AudioMode : uint8_t {
  AUDIO_MODE_TX_ONLY,     // Only send (mic only)
  AUDIO_MODE_RX_ONLY,     // Only receive (speaker only)
  AUDIO_MODE_FULL_DUPLEX, // Both (mic + speaker)
};

// Microphone channel
enum MicChannel : uint8_t {
  MIC_CHANNEL_LEFT,
  MIC_CHANNEL_RIGHT,
};

class I2SAudioUDP : public Component {
 public:
  void setup() override;
  void loop() override {}  // Empty - no business logic
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ─────────────────────────────────────────────────────────────────────────
  // Pin Configuration - Single Bus
  // ─────────────────────────────────────────────────────────────────────────
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_mclk_pin(int pin) { this->i2s_mclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_i2s_dout_pin(int pin) { this->i2s_dout_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Pin Configuration - Dual Bus
  // ─────────────────────────────────────────────────────────────────────────
  void set_mic_lrclk_pin(int pin) { this->mic_lrclk_pin_ = pin; }
  void set_mic_bclk_pin(int pin) { this->mic_bclk_pin_ = pin; }
  void set_mic_din_pin(int pin) { this->mic_din_pin_ = pin; }
  void set_speaker_lrclk_pin(int pin) { this->speaker_lrclk_pin_ = pin; }
  void set_speaker_bclk_pin(int pin) { this->speaker_bclk_pin_ = pin; }
  void set_speaker_dout_pin(int pin) { this->speaker_dout_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Audio Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_mic_bits_per_sample(int bits) { this->mic_bits_per_sample_ = bits; }
  void set_mic_channel(MicChannel channel) { this->mic_channel_ = channel; }
  void set_mic_gain(int gain) { this->mic_gain_ = gain; }
  void set_speaker_enable_pin(int pin) { this->speaker_enable_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Network Configuration (templatable)
  // ─────────────────────────────────────────────────────────────────────────
  void set_remote_ip(const std::string &ip) { this->remote_ip_ = ip; }
  void set_remote_port(uint16_t port) { this->remote_port_ = port; }
  void set_listen_port(uint16_t port) { this->listen_port_ = port; }

  // Lambda setters (evaluated at start())
  void set_remote_ip_lambda(std::function<std::string()> func) { this->remote_ip_func_ = func; }
  void set_remote_port_lambda(std::function<uint16_t()> func) { this->remote_port_func_ = func; }
  void set_listen_port_lambda(std::function<uint16_t()> func) { this->listen_port_func_ = func; }

  // ─────────────────────────────────────────────────────────────────────────
  // AEC Integration
  // ─────────────────────────────────────────────────────────────────────────
  void set_aec(esp_aec::EspAec *aec) { this->aec_ = aec; }

  // ─────────────────────────────────────────────────────────────────────────
  // Control Methods (called from automations)
  // ─────────────────────────────────────────────────────────────────────────
  void start();
  void stop();
  bool is_streaming() const { return this->streaming_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Volume Control
  // ─────────────────────────────────────────────────────────────────────────
  void set_volume(float volume);
  float get_volume() const { return this->volume_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Statistics (for sensors)
  // ─────────────────────────────────────────────────────────────────────────
  uint32_t get_tx_packets() const { return this->tx_packets_; }
  uint32_t get_rx_packets() const { return this->rx_packets_; }
  const char* get_audio_mode_text() const;
  I2SBusMode get_bus_mode() const { return this->bus_mode_; }
  AudioMode get_audio_mode() const { return this->audio_mode_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Triggers
  // ─────────────────────────────────────────────────────────────────────────
  Trigger<> *get_on_start_trigger() { return &this->on_start_trigger_; }
  Trigger<> *get_on_stop_trigger() { return &this->on_stop_trigger_; }
  Trigger<std::string> *get_on_error_trigger() { return &this->on_error_trigger_; }

 protected:
  void deduce_modes_();
  bool init_i2s_single_bus_();
  bool init_i2s_dual_bus_();
  void deinit_i2s_();
  bool init_sockets_();
  void close_sockets_();
  void apply_software_volume_(int16_t *buffer, size_t samples);

  static void audio_task(void *params);

  // Pin configuration - Single bus
  int i2s_lrclk_pin_{-1};
  int i2s_bclk_pin_{-1};
  int i2s_mclk_pin_{-1};
  int i2s_din_pin_{-1};
  int i2s_dout_pin_{-1};

  // Pin configuration - Dual bus
  int mic_lrclk_pin_{-1};
  int mic_bclk_pin_{-1};
  int mic_din_pin_{-1};
  int speaker_lrclk_pin_{-1};
  int speaker_bclk_pin_{-1};
  int speaker_dout_pin_{-1};

  // Audio config
  uint32_t sample_rate_{16000};
  int mic_bits_per_sample_{16};
  MicChannel mic_channel_{MIC_CHANNEL_LEFT};
  int mic_gain_{1};
  int speaker_enable_pin_{-1};
  float volume_{1.0f};

  // Network - runtime values (evaluated from templates)
  std::string remote_ip_;
  uint16_t remote_port_{0};
  uint16_t listen_port_{0};

  // Network - template functions
  std::function<std::string()> remote_ip_func_;
  std::function<uint16_t()> remote_port_func_;
  std::function<uint16_t()> listen_port_func_;

  // AEC
  esp_aec::EspAec *aec_{nullptr};

  // Deduced modes
  I2SBusMode bus_mode_{I2S_BUS_SINGLE};
  AudioMode audio_mode_{AUDIO_MODE_FULL_DUPLEX};

  // Runtime state
  volatile bool streaming_{false};
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  int send_socket_{-1};
  int recv_socket_{-1};
  struct sockaddr_in remote_addr_;
  TaskHandle_t audio_task_handle_{nullptr};

  // Statistics
  volatile uint32_t tx_packets_{0};
  volatile uint32_t rx_packets_{0};

  // Triggers
  Trigger<> on_start_trigger_;
  Trigger<> on_stop_trigger_;
  Trigger<std::string> on_error_trigger_;
};

// Actions
template<typename... Ts>
class StartAction : public Action<Ts...>, public Parented<I2SAudioUDP> {
 public:
  void play(Ts... x) override { this->parent_->start(); }
};

template<typename... Ts>
class StopAction : public Action<Ts...>, public Parented<I2SAudioUDP> {
 public:
  void play(Ts... x) override { this->parent_->stop(); }
};

// Sensors
class I2SAudioUDPSensor : public sensor::Sensor, public PollingComponent {
 public:
  enum SensorType { TX_PACKETS, RX_PACKETS };

  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void set_sensor_type(SensorType type) { this->type_ = type; }
  void update() override {
    if (this->parent_ == nullptr) return;
    if (this->type_ == TX_PACKETS) {
      this->publish_state(this->parent_->get_tx_packets());
    } else {
      this->publish_state(this->parent_->get_rx_packets());
    }
  }

 protected:
  I2SAudioUDP *parent_{nullptr};
  SensorType type_{TX_PACKETS};
};

class I2SAudioUDPTextSensor : public text_sensor::TextSensor, public PollingComponent {
 public:
  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void update() override {
    if (this->parent_ != nullptr) {
      this->publish_state(this->parent_->get_audio_mode_text());
    }
  }

 protected:
  I2SAudioUDP *parent_{nullptr};
};

// Volume Number
class I2SAudioUDPVolume : public number::Number, public Component {
 public:
  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void setup() override { this->publish_state(this->parent_->get_volume() * 100); }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr) {
      this->parent_->set_volume(value / 100.0f);
    }
  }

  I2SAudioUDP *parent_{nullptr};
};

}  // namespace i2s_audio_udp
}  // namespace esphome
