#pragma once

// =============================================================================
// ESP32-S3 UDP Intercom - Full Duplex Audio Component for ESPHome
// =============================================================================
// This component provides bidirectional audio streaming over UDP for use with
// Home Assistant and go2rtc. It uses I2S in full duplex mode for simultaneous
// microphone input and speaker output.
//
// Created by Claude (Anthropic) for n-IA-hane
// https://github.com/n-IA-hane/esp32-intercom
// =============================================================================

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <driver/i2s_std.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <map>
#include <string>

// ESP-AFE for Acoustic Echo Cancellation (official Espressif solution)
#include "esp_aec.h"

namespace esphome {
namespace udp_intercom {

// =============================================================================
// Intercom States
// =============================================================================
enum class IntercomState : uint8_t {
  IDLE = 0,           // Ready, waiting for action
  RINGING,            // Doorbell was pressed
  STREAMING_OUT,      // Mic -> UDP only
  STREAMING_IN,       // UDP -> Speaker only
  STREAMING_DUPLEX,   // Full duplex audio
  ERROR               // Something went wrong
};

// =============================================================================
// State Text Translations
// You can customize these strings to your language
// =============================================================================
static const std::map<IntercomState, const char*> STATE_TEXTS = {
  {IntercomState::IDLE,            "IDLE"},       // or "READY", "PRONTO", etc.
  {IntercomState::RINGING,         "RINGING"},    // or "DOORBELL", "SQUILLA", etc.
  {IntercomState::STREAMING_OUT,   "TX ONLY"},    // or "SENDING", etc.
  {IntercomState::STREAMING_IN,    "RX ONLY"},    // or "RECEIVING", etc.
  {IntercomState::STREAMING_DUPLEX,"STREAMING"},  // or "ON CALL", "IN LINEA", etc.
  {IntercomState::ERROR,           "ERROR"},      // or "ERRORE", etc.
};

// Helper function to get state text
static const char* get_state_text_static(IntercomState state) {
  auto it = STATE_TEXTS.find(state);
  if (it != STATE_TEXTS.end()) {
    return it->second;
  }
  return "UNKNOWN";
}

// =============================================================================
// UDPIntercom Component Class
// =============================================================================
class UDPIntercom : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // I2S pin configuration setters
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_mclk_pin(int pin) { this->i2s_mclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_i2s_dout_pin(int pin) { this->i2s_dout_pin_ = pin; }
  void set_speaker_enable_pin(int pin) { this->speaker_enable_pin_ = pin; }

  // Network configuration setters
  void set_server_ip(const std::string &ip) { this->server_ip_ = ip; }
  void set_server_port(uint16_t port) { this->server_port_ = port; }
  void set_listen_port(uint16_t port) { this->listen_port_ = port; }
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }

  // AEC (Echo Cancellation) control
  void set_aec_enabled(bool enabled) { this->aec_enabled_ = enabled; }
  bool get_aec_enabled() const { return this->aec_enabled_; }

  // Actions
  void start_streaming();
  void stop_streaming();
  void ring_doorbell();

  // State getters
  IntercomState get_state() const { return this->state_; }
  bool is_streaming() const {
    return this->state_ == IntercomState::STREAMING_OUT ||
           this->state_ == IntercomState::STREAMING_IN ||
           this->state_ == IntercomState::STREAMING_DUPLEX;
  }

  // Get translated state text
  const char* get_state_text(IntercomState state) const {
    return get_state_text_static(state);
  }

  // Debug counters
  uint32_t get_tx_packets() const { return this->tx_packets_; }
  uint32_t get_rx_packets() const { return this->rx_packets_; }
  void reset_counters() { this->tx_packets_ = 0; this->rx_packets_ = 0; }

  // Triggers for automations
  Trigger<> *get_ring_trigger() { return &this->ring_trigger_; }
  Trigger<> *get_call_start_trigger() { return &this->call_start_trigger_; }
  Trigger<> *get_call_end_trigger() { return &this->call_end_trigger_; }

 protected:
  // I2S pin configuration
  int i2s_lrclk_pin_{-1};
  int i2s_bclk_pin_{-1};
  int i2s_mclk_pin_{-1};
  int i2s_din_pin_{-1};
  int i2s_dout_pin_{-1};
  int speaker_enable_pin_{-1};

  // Network configuration
  std::string server_ip_;
  uint16_t server_port_{12345};
  uint16_t listen_port_{12346};
  uint32_t sample_rate_{16000};

  // I2S handles for full duplex
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};

  // State
  IntercomState state_{IntercomState::IDLE};
  bool streaming_active_{false};

  // UDP sockets
  int send_socket_{-1};
  int recv_socket_{-1};
  struct sockaddr_in server_addr_;

  // Audio task handle
  TaskHandle_t audio_task_handle_{nullptr};

  // AEC (Acoustic Echo Cancellation) - ESP-AFE
  bool aec_enabled_{true};  // Can be toggled via switch before streaming
  aec_handle_t *aec_handle_{nullptr};
  int aec_frame_size_{0};  // Set by aec_get_chunksize()

  // Debug counters
  uint32_t tx_packets_{0};
  uint32_t rx_packets_{0};

  // Automation triggers
  Trigger<> ring_trigger_;
  Trigger<> call_start_trigger_;
  Trigger<> call_end_trigger_;

  // Internal methods
  bool init_i2s_full_duplex_();
  void deinit_i2s_();
  bool init_sockets_();
  void close_sockets_();
  void set_state_(IntercomState new_state);

  // Audio task (runs in separate FreeRTOS task)
  static void audio_task(void *params);
};

// =============================================================================
// Action Templates for ESPHome Automations
// =============================================================================
template<typename... Ts> class StartStreamingAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->start_streaming(); }
};

template<typename... Ts> class StopStreamingAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->stop_streaming(); }
};

template<typename... Ts> class RingDoorbellAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->ring_doorbell(); }
};

}  // namespace udp_intercom
}  // namespace esphome
