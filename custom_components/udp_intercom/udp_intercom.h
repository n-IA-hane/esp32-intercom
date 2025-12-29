#pragma once

// =============================================================================
// ESP32-S3 UDP Intercom - Full Duplex Audio Component for ESPHome
// =============================================================================
// Supports:
// - Single I2S bus (ES8311 codec) - full-duplex on one port
// - Dual I2S bus (INMP441 + MAX98357A) - separate ports for mic/speaker
// - go2rtc mode for Home Assistant integration
// - P2P mode for device-to-device communication with mDNS discovery
// - AEC (Acoustic Echo Cancellation) via esp-sr AFE
// - Software volume control when hardware DAC not available
//
// Created by Claude (Anthropic) for n-IA-hane
// https://github.com/n-IA-hane/esp32-intercom
// =============================================================================

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <string>
#include <vector>
#include <map>

#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

namespace esphome {
namespace udp_intercom {

// =============================================================================
// Enums
// =============================================================================

enum IntercomState : uint8_t {
  INTERCOM_STATE_IDLE = 0,
  INTERCOM_STATE_RINGING,
  INTERCOM_STATE_STREAMING_DUPLEX,
  INTERCOM_STATE_STREAMING_IN,
  INTERCOM_STATE_STREAMING_OUT,
  INTERCOM_STATE_ERROR,
  INTERCOM_STATE_SCANNING,       // P2P: scanning for peers
  INTERCOM_STATE_CALLING,        // P2P: calling a peer
};

enum I2SMode : uint8_t {
  I2S_MODE_SINGLE,   // Single bus full-duplex (ES8311)
  I2S_MODE_DUAL,     // Dual bus (INMP441 + MAX98357A)
};

enum VolumeMode : uint8_t {
  VOLUME_MODE_AUTO,      // Auto-detect (hardware if available, else software)
  VOLUME_MODE_HARDWARE,  // Force hardware (ES8311 I2C)
  VOLUME_MODE_SOFTWARE,  // Force software (multiply samples)
};

enum OperatingMode : uint8_t {
  OPERATING_MODE_GO2RTC,  // Stream to go2rtc server
  OPERATING_MODE_P2P,     // Direct device-to-device
};

// =============================================================================
// State Text Translations
// =============================================================================
static const std::map<IntercomState, const char*> STATE_TEXTS = {
  {INTERCOM_STATE_IDLE,            "IDLE"},
  {INTERCOM_STATE_RINGING,         "RINGING"},
  {INTERCOM_STATE_STREAMING_OUT,   "TX ONLY"},
  {INTERCOM_STATE_STREAMING_IN,    "RX ONLY"},
  {INTERCOM_STATE_STREAMING_DUPLEX,"STREAMING"},
  {INTERCOM_STATE_ERROR,           "ERROR"},
  {INTERCOM_STATE_SCANNING,        "SCANNING"},
  {INTERCOM_STATE_CALLING,         "CALLING"},
};

static const char* get_state_text_static(IntercomState state) {
  auto it = STATE_TEXTS.find(state);
  if (it != STATE_TEXTS.end()) {
    return it->second;
  }
  return "UNKNOWN";
}

// =============================================================================
// Peer Info Structure (for P2P discovery)
// =============================================================================

struct PeerInfo {
  std::string name;
  std::string ip;
  uint16_t port;
  uint32_t last_seen;  // millis() timestamp
  bool active;
};

// =============================================================================
// Main Component Class
// =============================================================================

class UDPIntercom : public Component {
 public:
  // ─────────────────────────────────────────────────────────────────────────
  // ESPHome Component Interface
  // ─────────────────────────────────────────────────────────────────────────
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ─────────────────────────────────────────────────────────────────────────
  // Network Configuration (go2rtc mode)
  // ─────────────────────────────────────────────────────────────────────────
  void set_server_ip(const std::string &ip) { this->server_ip_ = ip; }
  void set_server_port(uint16_t port) { this->server_port_ = port; }
  void set_listen_port(uint16_t port) { this->listen_port_ = port; }

  // ─────────────────────────────────────────────────────────────────────────
  // I2S Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_i2s_mode(I2SMode mode) { this->i2s_mode_ = mode; }
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }

  // Single bus pins (ES8311)
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_mclk_pin(int pin) { this->i2s_mclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_i2s_dout_pin(int pin) { this->i2s_dout_pin_ = pin; }

  // Dual bus pins (INMP441 + MAX98357A)
  void set_mic_lrclk_pin(int pin) { this->mic_lrclk_pin_ = pin; }
  void set_mic_bclk_pin(int pin) { this->mic_bclk_pin_ = pin; }
  void set_mic_din_pin(int pin) { this->mic_din_pin_ = pin; }
  void set_speaker_lrclk_pin(int pin) { this->speaker_lrclk_pin_ = pin; }
  void set_speaker_bclk_pin(int pin) { this->speaker_bclk_pin_ = pin; }
  void set_speaker_dout_pin(int pin) { this->speaker_dout_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Hardware Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_speaker_enable_pin(int pin) { this->speaker_enable_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Audio Processing Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_aec_enabled(bool enabled) { this->aec_enabled_ = enabled; }
  bool get_aec_enabled() const { return this->aec_enabled_; }
  void set_volume_mode(VolumeMode mode) { this->volume_mode_ = mode; }

  // Runtime volume control (0.0 - 1.0)
  void set_volume(float volume);
  float get_volume() const { return this->volume_; }

  // ─────────────────────────────────────────────────────────────────────────
  // P2P Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_p2p_enabled(bool enabled) { this->p2p_enabled_ = enabled; }
  void set_p2p_service_name(const std::string &name) { this->p2p_service_name_ = name; }
  void set_p2p_auto_answer(bool enabled) { this->p2p_auto_answer_ = enabled; }
  void set_p2p_timeout(uint32_t timeout_ms) { this->p2p_timeout_ms_ = timeout_ms; }

  // ─────────────────────────────────────────────────────────────────────────
  // Operating Mode
  // ─────────────────────────────────────────────────────────────────────────
  void set_operating_mode(OperatingMode mode);
  OperatingMode get_operating_mode() const { return this->operating_mode_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Streaming Control
  // ─────────────────────────────────────────────────────────────────────────
  void start_streaming();
  void start_streaming_to_peer(const std::string &peer_name);
  void stop_streaming();
  bool is_streaming() const {
    return this->state_ == INTERCOM_STATE_STREAMING_DUPLEX ||
           this->state_ == INTERCOM_STATE_STREAMING_IN ||
           this->state_ == INTERCOM_STATE_STREAMING_OUT;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Doorbell / Ring
  // ─────────────────────────────────────────────────────────────────────────
  void ring_doorbell();

  // ─────────────────────────────────────────────────────────────────────────
  // P2P Peer Management
  // ─────────────────────────────────────────────────────────────────────────
  void refresh_peers();
  void call_peer(const std::string &peer_name);
  const std::vector<PeerInfo>& get_peers() const { return this->discovered_peers_; }
  std::string get_peers_list() const;
  int get_peer_count() const { return this->discovered_peers_.size(); }

  // Get peer names for select component
  std::vector<std::string> get_peer_names() const {
    std::vector<std::string> names;
    for (const auto &peer : this->discovered_peers_) {
      names.push_back(peer.name);
    }
    return names;
  }

  // Get IP by peer name (returns empty string if not found)
  std::string get_peer_ip_by_name(const std::string &name) const {
    for (const auto &peer : this->discovered_peers_) {
      if (peer.name == name) {
        return peer.ip;
      }
    }
    return "";
  }

  // Peer selection (for UI navigation)
  void select_next_peer();
  void select_previous_peer();
  int get_selected_peer_index() const { return this->selected_peer_index_; }
  const PeerInfo* get_selected_peer() const;

  // Target selection (0 = manual IP, 1+ = discovered peer index)
  void set_target_index(int index) { this->target_index_ = index; }
  int get_target_index() const { return this->target_index_; }
  std::string get_target_description(const std::string &manual_ip) const;
  bool call_target(const std::string &manual_ip);

  // ─────────────────────────────────────────────────────────────────────────
  // State and Statistics
  // ─────────────────────────────────────────────────────────────────────────
  IntercomState get_state() const { return this->state_; }
  const char* get_state_text(IntercomState state) const { return get_state_text_static(state); }

  uint32_t get_tx_packets() const { return this->tx_packets_; }
  uint32_t get_rx_packets() const { return this->rx_packets_; }
  void reset_counters() { this->tx_packets_ = 0; this->rx_packets_ = 0; }

  // ─────────────────────────────────────────────────────────────────────────
  // Automation Triggers
  // ─────────────────────────────────────────────────────────────────────────
  Trigger<> *get_ring_trigger() { return &this->ring_trigger_; }
  Trigger<> *get_call_start_trigger() { return &this->call_start_trigger_; }
  Trigger<> *get_call_end_trigger() { return &this->call_end_trigger_; }

  void add_on_peer_discovered_callback(std::function<void(std::string, std::string)> callback) {
    this->peer_discovered_callbacks_.add(std::move(callback));
  }
  void add_on_peer_lost_callback(std::function<void(std::string)> callback) {
    this->peer_lost_callbacks_.add(std::move(callback));
  }

 protected:
  // ─────────────────────────────────────────────────────────────────────────
  // Internal Methods
  // ─────────────────────────────────────────────────────────────────────────
  void set_state_(IntercomState new_state);

  // I2S initialization
  bool init_i2s_single_bus_();    // For ES8311 (shared bus)
  bool init_i2s_dual_bus_();      // For INMP441 + MAX98357A
  void deinit_i2s_();

  // Socket management
  bool init_sockets_();
  bool init_sockets_for_peer_(const std::string &peer_ip, uint16_t peer_port);
  void close_sockets_();

  // mDNS discovery (service announced via ESPHome YAML config)
  void init_mdns_();
  void mdns_query_peers_();
  void cleanup_stale_peers_();

  // Audio task
  static void audio_task(void *params);

  // Volume
  void apply_software_volume_(int16_t *buffer, size_t samples);

  // ─────────────────────────────────────────────────────────────────────────
  // Configuration Variables
  // ─────────────────────────────────────────────────────────────────────────

  // Network
  std::string server_ip_;
  uint16_t server_port_{12345};
  uint16_t listen_port_{12346};

  // I2S Mode
  I2SMode i2s_mode_{I2S_MODE_SINGLE};
  uint32_t sample_rate_{16000};

  // Single bus pins
  int i2s_lrclk_pin_{-1};
  int i2s_bclk_pin_{-1};
  int i2s_mclk_pin_{-1};
  int i2s_din_pin_{-1};
  int i2s_dout_pin_{-1};

  // Dual bus pins (microphone)
  int mic_lrclk_pin_{-1};
  int mic_bclk_pin_{-1};
  int mic_din_pin_{-1};

  // Dual bus pins (speaker)
  int speaker_lrclk_pin_{-1};
  int speaker_bclk_pin_{-1};
  int speaker_dout_pin_{-1};

  // Hardware
  int speaker_enable_pin_{-1};

  // Audio processing
  bool aec_enabled_{false};
  VolumeMode volume_mode_{VOLUME_MODE_AUTO};
  float volume_{0.7f};  // 0.0 - 1.0

  // P2P configuration
  bool p2p_enabled_{false};
  std::string p2p_service_name_{"udp-intercom"};
  bool p2p_auto_answer_{false};
  uint32_t p2p_timeout_ms_{10000};

  // Operating mode
  OperatingMode operating_mode_{OPERATING_MODE_GO2RTC};

  // ─────────────────────────────────────────────────────────────────────────
  // Runtime State
  // ─────────────────────────────────────────────────────────────────────────
  IntercomState state_{INTERCOM_STATE_IDLE};
  volatile bool streaming_active_{false};

  // I2S handles - single bus mode
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};

  // Sockets
  int send_socket_{-1};
  int recv_socket_{-1};
  struct sockaddr_in server_addr_;
  struct sockaddr_in peer_addr_;  // For P2P mode

  // Task
  TaskHandle_t audio_task_handle_{nullptr};

  // AEC (esp-sr)
  void *aec_handle_{nullptr};
  int aec_frame_size_{0};

  // Statistics
  volatile uint32_t tx_packets_{0};
  volatile uint32_t rx_packets_{0};
  volatile uint32_t last_rx_time_{0};  // For auto-hangup in P2P (volatile for thread safety)
  uint32_t last_hangup_time_{0};  // Cooldown after hangup to prevent immediate auto-answer

  // P2P peer discovery
  std::vector<PeerInfo> discovered_peers_;
  int selected_peer_index_{0};
  int target_index_{0};  // 0 = manual IP, 1+ = discovered peer
  uint32_t last_peer_query_{0};
  std::string current_peer_ip_;
  uint16_t current_peer_port_{0};

  // Triggers
  Trigger<> ring_trigger_;
  Trigger<> call_start_trigger_;
  Trigger<> call_end_trigger_;
  CallbackManager<void(std::string, std::string)> peer_discovered_callbacks_;
  CallbackManager<void(std::string)> peer_lost_callbacks_;
};

// =============================================================================
// Automation Triggers
// =============================================================================

class PeerDiscoveredTrigger : public Trigger<std::string, std::string> {
 public:
  explicit PeerDiscoveredTrigger(UDPIntercom *parent) {
    parent->add_on_peer_discovered_callback([this](std::string name, std::string ip) {
      this->trigger(name, ip);
    });
  }
};

class PeerLostTrigger : public Trigger<std::string> {
 public:
  explicit PeerLostTrigger(UDPIntercom *parent) {
    parent->add_on_peer_lost_callback([this](std::string name) {
      this->trigger(name);
    });
  }
};

// =============================================================================
// Automation Actions
// =============================================================================

template<typename... Ts>
class StartStreamingAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->start_streaming(); }
};

template<typename... Ts>
class StopStreamingAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->stop_streaming(); }
};

template<typename... Ts>
class RingDoorbellAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->ring_doorbell(); }
};

template<typename... Ts>
class CallPeerAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  TEMPLATABLE_VALUE(std::string, peer)
  void play(Ts... x) override {
    this->parent_->call_peer(this->peer_.value(x...));
  }
};

template<typename... Ts>
class RefreshPeersAction : public Action<Ts...>, public Parented<UDPIntercom> {
 public:
  void play(Ts... x) override { this->parent_->refresh_peers(); }
};

}  // namespace udp_intercom
}  // namespace esphome
