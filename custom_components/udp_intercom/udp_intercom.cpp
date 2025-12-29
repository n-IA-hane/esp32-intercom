// =============================================================================
// ESP32-S3 UDP Intercom - Full Duplex Audio Component for ESPHome
// =============================================================================
// Supports single-bus (ES8311) and dual-bus (INMP441+MAX98357A) configurations
// with go2rtc and P2P modes, AEC, and software volume control.
//
// Created by Claude (Anthropic) for n-IA-hane
// https://github.com/n-IA-hane/esp32-intercom
// =============================================================================

#include "udp_intercom.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>
#include <algorithm>
#include <errno.h>
#include <driver/gpio.h>
#include <mdns.h>

// ESP-AFE for AEC (only if enabled)
#ifdef USE_ESP_AEC
#include "esp_aec.h"
#endif
#include "esp_heap_caps.h"

namespace esphome {
namespace udp_intercom {

static const char *TAG = "udp_intercom";

// =============================================================================
// Audio Buffer Configuration
// =============================================================================
static const size_t AUDIO_BUFFER_SIZE = 1024;   // Bytes per chunk
static const size_t DMA_BUFFER_COUNT = 8;
static const size_t DMA_BUFFER_SIZE = 512;

// =============================================================================
// Jitter Buffer
// =============================================================================
static const size_t JITTER_BUFFER_SIZE = 8192;  // 256ms of audio buffer
static uint8_t jitter_buffer[JITTER_BUFFER_SIZE];
static volatile size_t jitter_write_pos = 0;
static volatile size_t jitter_read_pos = 0;
static volatile size_t jitter_available = 0;

static size_t jitter_write(const uint8_t *data, size_t len) {
  size_t written = 0;
  while (written < len && jitter_available < JITTER_BUFFER_SIZE) {
    jitter_buffer[jitter_write_pos] = data[written];
    jitter_write_pos = (jitter_write_pos + 1) % JITTER_BUFFER_SIZE;
    jitter_available++;
    written++;
  }
  return written;
}

static size_t jitter_read(uint8_t *data, size_t len) {
  size_t read = 0;
  while (read < len && jitter_available > 0) {
    data[read] = jitter_buffer[jitter_read_pos];
    jitter_read_pos = (jitter_read_pos + 1) % JITTER_BUFFER_SIZE;
    jitter_available--;
    read++;
  }
  return read;
}

// =============================================================================
// Component Setup
// =============================================================================
void UDPIntercom::setup() {
  ESP_LOGI(TAG, "Setting up UDP Intercom...");
  ESP_LOGI(TAG, "  I2S Mode: %s", this->i2s_mode_ == I2S_MODE_SINGLE ? "SINGLE" : "DUAL");
  ESP_LOGI(TAG, "  Sample rate: %d Hz", this->sample_rate_);

  if (this->i2s_mode_ == I2S_MODE_SINGLE) {
    ESP_LOGI(TAG, "  I2S pins: LRCLK=%d, BCLK=%d, MCLK=%d, DIN=%d, DOUT=%d",
             this->i2s_lrclk_pin_, this->i2s_bclk_pin_, this->i2s_mclk_pin_,
             this->i2s_din_pin_, this->i2s_dout_pin_);
  } else {
    ESP_LOGI(TAG, "  Mic pins: LRCLK=%d, BCLK=%d, DIN=%d",
             this->mic_lrclk_pin_, this->mic_bclk_pin_, this->mic_din_pin_);
    ESP_LOGI(TAG, "  Speaker pins: LRCLK=%d, BCLK=%d, DOUT=%d",
             this->speaker_lrclk_pin_, this->speaker_bclk_pin_, this->speaker_dout_pin_);
  }

  if (!this->server_ip_.empty()) {
    ESP_LOGI(TAG, "  go2rtc Server: %s:%d", this->server_ip_.c_str(), this->server_port_);
  }
  ESP_LOGI(TAG, "  Listen port: %d", this->listen_port_);
  ESP_LOGI(TAG, "  P2P enabled: %s", this->p2p_enabled_ ? "YES" : "NO");
  ESP_LOGI(TAG, "  AEC enabled: %s", this->aec_enabled_ ? "YES" : "NO");

  // Initialize server address for go2rtc mode
  if (!this->server_ip_.empty()) {
    memset(&this->server_addr_, 0, sizeof(this->server_addr_));
    this->server_addr_.sin_family = AF_INET;
    this->server_addr_.sin_port = htons(this->server_port_);
    inet_pton(AF_INET, this->server_ip_.c_str(), &this->server_addr_.sin_addr);
  }

  // Enable speaker amplifier if configured
  if (this->speaker_enable_pin_ >= 0) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << this->speaker_enable_pin_);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)this->speaker_enable_pin_, 1);
    ESP_LOGI(TAG, "Speaker amplifier enabled on GPIO%d", this->speaker_enable_pin_);
  }

  // mDNS announcement is done in loop() after delay to ensure ESPHome mDNS is ready

  this->set_state_(INTERCOM_STATE_IDLE);
  ESP_LOGI(TAG, "UDP Intercom ready");
}

void UDPIntercom::loop() {
  // P2P: periodic peer discovery, auto-answer, and cleanup
  if (this->p2p_enabled_) {
    uint32_t now = millis();

    // Query for peers every 10 seconds
    // mDNS service is announced via ESPHome YAML config (mdns: services:)
    if (now - this->last_peer_query_ > 10000) {
      ESP_LOGD(TAG, "Querying for P2P peers...");
      this->mdns_query_peers_();
      this->cleanup_stale_peers_();
      this->last_peer_query_ = now;
    }

    // P2P Auto-Answer: check for incoming packets when idle
    // Log every 10 seconds to debug
    static uint32_t last_auto_answer_log = 0;
    if (now - last_auto_answer_log > 10000) {
      ESP_LOGI(TAG, "Auto-answer: %s, state: %s, socket: %d",
               this->p2p_auto_answer_ ? "ON" : "OFF",
               get_state_text_static(this->state_),
               this->recv_socket_);
      last_auto_answer_log = now;
    }
    // Check cooldown after hangup (5 seconds)
    bool cooldown_active = (now - this->last_hangup_time_) < 5000;

    if (this->p2p_auto_answer_ && this->state_ == INTERCOM_STATE_IDLE && !cooldown_active) {
      // Create temporary socket to check for incoming calls
      if (this->recv_socket_ < 0) {
        this->recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (this->recv_socket_ >= 0) {
          int reuse = 1;
          setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

          struct sockaddr_in listen_addr;
          memset(&listen_addr, 0, sizeof(listen_addr));
          listen_addr.sin_family = AF_INET;
          listen_addr.sin_addr.s_addr = INADDR_ANY;
          listen_addr.sin_port = htons(this->listen_port_);

          if (bind(this->recv_socket_, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
            close(this->recv_socket_);
            this->recv_socket_ = -1;
          } else {
            int flags = fcntl(this->recv_socket_, F_GETFL, 0);
            fcntl(this->recv_socket_, F_SETFL, flags | O_NONBLOCK);
            ESP_LOGI(TAG, "Auto-answer listener active on port %d", this->listen_port_);
          }
        }
      }

      // Check for incoming packets
      if (this->recv_socket_ >= 0) {
        uint8_t probe_buf[1024];  // Larger buffer for audio packets
        struct sockaddr_in caller_addr;
        socklen_t addr_len = sizeof(caller_addr);

        ssize_t received = recvfrom(this->recv_socket_, probe_buf, sizeof(probe_buf),
                                     0, (struct sockaddr *)&caller_addr, &addr_len);  // Remove MSG_PEEK

        if (received > 0) {
          // Someone is calling! Get their IP
          char caller_ip[16];
          inet_ntoa_r(caller_addr.sin_addr, caller_ip, sizeof(caller_ip));

          ESP_LOGI(TAG, ">>> INCOMING PACKET from %s:%d - %d bytes - auto-answering!",
                   caller_ip, ntohs(caller_addr.sin_port), (int)received);

          // Close the probe socket BEFORE starting streaming
          close(this->recv_socket_);
          this->recv_socket_ = -1;

          // Small delay to ensure socket is fully released
          vTaskDelay(pdMS_TO_TICKS(50));

          // Answer the call - this will create new sockets
          this->start_streaming_to_peer(std::string(caller_ip));

          // IMPORTANT: Return immediately to avoid timeout check in same loop iteration
          return;
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          ESP_LOGW(TAG, "Auto-answer recvfrom error: %d (%s)", errno, strerror(errno));
        }
      }
    } else if (this->recv_socket_ >= 0 && !this->is_streaming()) {
      // Close listener socket if:
      // - auto-answer is disabled, OR
      // - cooldown is active (just hung up)
      if (!this->p2p_auto_answer_ || cooldown_active) {
        close(this->recv_socket_);
        this->recv_socket_ = -1;
        if (cooldown_active) {
          ESP_LOGD(TAG, "Cooldown active - ignoring incoming packets");
        }
      }
    }

    // P2P timeout disabled - manual hangup only
    // The auto-answer will handle reconnection if needed
  }
}

void UDPIntercom::dump_config() {
  ESP_LOGCONFIG(TAG, "UDP Intercom:");
  ESP_LOGCONFIG(TAG, "  I2S Mode: %s", this->i2s_mode_ == I2S_MODE_SINGLE ? "SINGLE" : "DUAL");
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  if (!this->server_ip_.empty()) {
    ESP_LOGCONFIG(TAG, "  Server: %s:%d", this->server_ip_.c_str(), this->server_port_);
  }
  ESP_LOGCONFIG(TAG, "  Listen Port: %d", this->listen_port_);
  ESP_LOGCONFIG(TAG, "  P2P Enabled: %s", this->p2p_enabled_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  AEC Enabled: %s", this->aec_enabled_ ? "YES" : "NO");
}

// =============================================================================
// I2S Initialization - Single Bus (ES8311)
// =============================================================================
bool UDPIntercom::init_i2s_single_bus_() {
  ESP_LOGI(TAG, "Initializing I2S Single Bus (Full Duplex)...");

  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  esp_err_t err = i2s_new_channel(&chan_cfg, &this->tx_handle_, &this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = this->sample_rate_,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = (gpio_num_t)this->i2s_mclk_pin_,
      .bclk = (gpio_num_t)this->i2s_bclk_pin_,
      .ws = (gpio_num_t)this->i2s_lrclk_pin_,
      .dout = (gpio_num_t)this->i2s_dout_pin_,
      .din = (gpio_num_t)this->i2s_din_pin_,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    return false;
  }

  err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    return false;
  }

  i2s_channel_enable(this->tx_handle_);
  i2s_channel_enable(this->rx_handle_);

  ESP_LOGI(TAG, "I2S Single Bus initialized");
  return true;
}

// =============================================================================
// I2S Initialization - Dual Bus (INMP441 + MAX98357A)
// =============================================================================
bool UDPIntercom::init_i2s_dual_bus_() {
  ESP_LOGI(TAG, "Initializing I2S Dual Bus...");

  // ─────────────────────────────────────────────────────────────────────────
  // I2S_NUM_0: Microphone (RX only) - INMP441
  // ─────────────────────────────────────────────────────────────────────────
  i2s_chan_config_t mic_chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  esp_err_t err = i2s_new_channel(&mic_chan_cfg, nullptr, &this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create mic I2S channel: %s", esp_err_to_name(err));
    return false;
  }

  // Microphone configuration - uses configurable bit width and channel
  // Supports any I2S microphone (INMP441, SPH0645, ICS-43434, etc.)
  i2s_data_bit_width_t mic_bit_width = (this->mic_bits_per_sample_ == 32)
      ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT;

  i2s_std_config_t mic_std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = this->sample_rate_,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(mic_bit_width, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = GPIO_NUM_NC,
      .bclk = (gpio_num_t)this->mic_bclk_pin_,
      .ws = (gpio_num_t)this->mic_lrclk_pin_,
      .dout = GPIO_NUM_NC,
      .din = (gpio_num_t)this->mic_din_pin_,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  // Set slot bit width to match sample bit width
  mic_std_cfg.slot_cfg.slot_bit_width = (this->mic_bits_per_sample_ == 32)
      ? I2S_SLOT_BIT_WIDTH_32BIT : I2S_SLOT_BIT_WIDTH_16BIT;

  // Set channel based on configuration
  switch (this->mic_channel_) {
    case MIC_CHANNEL_RIGHT:
      mic_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
      break;
    case MIC_CHANNEL_STEREO:
      mic_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
      break;
    case MIC_CHANNEL_LEFT:
    default:
      mic_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
      break;
  }

  ESP_LOGI(TAG, "Mic config: %d-bit, channel=%d, gain=%dx",
           this->mic_bits_per_sample_, this->mic_channel_, this->mic_gain_);

  err = i2s_channel_init_std_mode(this->rx_handle_, &mic_std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init mic RX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->rx_handle_);
    return false;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // I2S_NUM_1: Speaker (TX only) - MAX98357A
  // ─────────────────────────────────────────────────────────────────────────
  i2s_chan_config_t spk_chan_cfg = {
    .id = I2S_NUM_1,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  err = i2s_new_channel(&spk_chan_cfg, &this->tx_handle_, nullptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create speaker I2S channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->rx_handle_);
    return false;
  }

  i2s_std_config_t spk_std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = this->sample_rate_,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = GPIO_NUM_NC,  // MAX98357A doesn't need MCLK
      .bclk = (gpio_num_t)this->speaker_bclk_pin_,
      .ws = (gpio_num_t)this->speaker_lrclk_pin_,
      .dout = (gpio_num_t)this->speaker_dout_pin_,
      .din = GPIO_NUM_NC,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  spk_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  err = i2s_channel_init_std_mode(this->tx_handle_, &spk_std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init speaker TX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->rx_handle_);
    i2s_del_channel(this->tx_handle_);
    return false;
  }

  // Enable both channels
  i2s_channel_enable(this->rx_handle_);
  i2s_channel_enable(this->tx_handle_);

  ESP_LOGI(TAG, "I2S Dual Bus initialized (Mic on I2S0, Speaker on I2S1)");
  return true;
}

void UDPIntercom::deinit_i2s_() {
  if (this->tx_handle_ != nullptr) {
    i2s_channel_disable(this->tx_handle_);
    i2s_del_channel(this->tx_handle_);
    this->tx_handle_ = nullptr;
  }
  if (this->rx_handle_ != nullptr) {
    i2s_channel_disable(this->rx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
  }
  ESP_LOGI(TAG, "I2S deinitialized");
}

// =============================================================================
// Socket Management
// =============================================================================
bool UDPIntercom::init_sockets_() {
  this->send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->send_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create send socket: %d", errno);
    return false;
  }

  int flags = fcntl(this->send_socket_, F_GETFL, 0);
  fcntl(this->send_socket_, F_SETFL, flags | O_NONBLOCK);

  this->recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->recv_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create receive socket: %d", errno);
    close(this->send_socket_);
    this->send_socket_ = -1;
    return false;
  }

  int reuse = 1;
  setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));

  struct sockaddr_in listen_addr;
  memset(&listen_addr, 0, sizeof(listen_addr));
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_addr.s_addr = INADDR_ANY;
  listen_addr.sin_port = htons(this->listen_port_);

  if (bind(this->recv_socket_, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind receive socket to port %d: %s", this->listen_port_, strerror(errno));
    close(this->send_socket_);
    close(this->recv_socket_);
    this->send_socket_ = -1;
    this->recv_socket_ = -1;
    return false;
  }

  flags = fcntl(this->recv_socket_, F_GETFL, 0);
  fcntl(this->recv_socket_, F_SETFL, flags | O_NONBLOCK);

  int rcvbuf = 32768;
  setsockopt(this->recv_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  ESP_LOGI(TAG, "UDP sockets initialized");
  return true;
}

bool UDPIntercom::init_sockets_for_peer_(const std::string &peer_ip, uint16_t peer_port) {
  if (!this->init_sockets_()) {
    return false;
  }

  // Set up peer address for P2P
  memset(&this->peer_addr_, 0, sizeof(this->peer_addr_));
  this->peer_addr_.sin_family = AF_INET;
  this->peer_addr_.sin_port = htons(peer_port);
  inet_pton(AF_INET, peer_ip.c_str(), &this->peer_addr_.sin_addr);

  this->current_peer_ip_ = peer_ip;
  this->current_peer_port_ = peer_port;

  ESP_LOGI(TAG, "Sockets configured for peer: %s:%d", peer_ip.c_str(), peer_port);
  return true;
}

void UDPIntercom::close_sockets_() {
  if (this->send_socket_ >= 0) {
    close(this->send_socket_);
    this->send_socket_ = -1;
  }
  if (this->recv_socket_ >= 0) {
    close(this->recv_socket_);
    this->recv_socket_ = -1;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  ESP_LOGI(TAG, "UDP sockets closed");
}

// =============================================================================
// mDNS Discovery
// =============================================================================
void UDPIntercom::init_mdns_() {
  // Not used - announcement is done in loop() after delay
}

void UDPIntercom::mdns_query_peers_() {
  mdns_result_t *results = nullptr;

  // Query format: ESPHome registers with "_" prefix, so we need to match
  // Try both formats: with and without underscore prefix
  std::string service_name = this->p2p_service_name_;
  if (service_name[0] != '_') {
    service_name = "_" + service_name;
  }

  ESP_LOGD(TAG, "mDNS query for service: %s._udp", service_name.c_str());
  esp_err_t err = mdns_query_ptr(service_name.c_str(), "_udp", 1000, 10, &results);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "mDNS query failed: %s", esp_err_to_name(err));
    return;
  }

  if (results == nullptr) {
    ESP_LOGD(TAG, "mDNS query: no results");
    return;
  }

  ESP_LOGI(TAG, "mDNS query returned results");

  std::string my_name = App.get_name();

  mdns_result_t *r = results;
  while (r) {
    if (r->hostname && r->addr) {
      std::string peer_name = r->hostname;

      // Skip ourselves
      if (peer_name != my_name) {
        char ip_str[16];
        inet_ntoa_r(r->addr->addr.u_addr.ip4, ip_str, sizeof(ip_str));

        // Check if we already know this peer
        bool found = false;
        for (auto &peer : this->discovered_peers_) {
          if (peer.name == peer_name) {
            peer.last_seen = millis();
            peer.ip = ip_str;
            peer.port = r->port;
            peer.active = true;
            found = true;
            break;
          }
        }

        if (!found) {
          PeerInfo new_peer;
          new_peer.name = peer_name;
          new_peer.ip = ip_str;
          new_peer.port = r->port;
          new_peer.last_seen = millis();
          new_peer.active = true;
          this->discovered_peers_.push_back(new_peer);

          ESP_LOGI(TAG, "Discovered peer: %s (%s:%d)", peer_name.c_str(), ip_str, r->port);
          this->peer_discovered_callbacks_.call(peer_name, std::string(ip_str));
        }
      }
    }
    r = r->next;
  }

  mdns_query_results_free(results);
}

void UDPIntercom::cleanup_stale_peers_() {
  uint32_t now = millis();
  const uint32_t STALE_TIMEOUT = 60000;  // 60 seconds

  for (auto it = this->discovered_peers_.begin(); it != this->discovered_peers_.end();) {
    if (now - it->last_seen > STALE_TIMEOUT) {
      ESP_LOGI(TAG, "Peer lost: %s", it->name.c_str());
      this->peer_lost_callbacks_.call(it->name);
      it = this->discovered_peers_.erase(it);

      // Adjust selected index if needed
      if (this->selected_peer_index_ >= (int)this->discovered_peers_.size()) {
        this->selected_peer_index_ = std::max(0, (int)this->discovered_peers_.size() - 1);
      }
    } else {
      ++it;
    }
  }
}

// =============================================================================
// Peer Management
// =============================================================================
void UDPIntercom::refresh_peers() {
  ESP_LOGI(TAG, "Refreshing peer list...");
  this->mdns_query_peers_();
}

void UDPIntercom::call_peer(const std::string &peer_name) {
  for (const auto &peer : this->discovered_peers_) {
    if (peer.name == peer_name) {
      ESP_LOGI(TAG, "Calling peer: %s (%s:%d)", peer.name.c_str(), peer.ip.c_str(), peer.port);
      this->start_streaming_to_peer(peer.ip);
      return;
    }
  }
  ESP_LOGW(TAG, "Peer not found: %s", peer_name.c_str());
}

std::string UDPIntercom::get_peers_list() const {
  std::string result;
  for (size_t i = 0; i < this->discovered_peers_.size(); i++) {
    if (i > 0) result += ", ";
    result += this->discovered_peers_[i].name;
    result += " (";
    result += this->discovered_peers_[i].ip;
    result += ")";
  }
  return result.empty() ? "No peers found" : result;
}

void UDPIntercom::select_next_peer() {
  if (!this->discovered_peers_.empty()) {
    this->selected_peer_index_ = (this->selected_peer_index_ + 1) % this->discovered_peers_.size();
  }
}

void UDPIntercom::select_previous_peer() {
  if (!this->discovered_peers_.empty()) {
    this->selected_peer_index_ = (this->selected_peer_index_ - 1 + this->discovered_peers_.size()) % this->discovered_peers_.size();
  }
}

const PeerInfo* UDPIntercom::get_selected_peer() const {
  if (this->selected_peer_index_ >= 0 && this->selected_peer_index_ < (int)this->discovered_peers_.size()) {
    return &this->discovered_peers_[this->selected_peer_index_];
  }
  return nullptr;
}

std::string UDPIntercom::get_target_description(const std::string &manual_ip) const {
  if (this->target_index_ == 0) {
    // Manual IP mode
    if (manual_ip.empty()) {
      return "Manual (no IP set)";
    }
    return "Manual: " + manual_ip;
  }

  // Discovered peer mode (target_index_ - 1 = peer index)
  int peer_idx = this->target_index_ - 1;
  if (peer_idx >= 0 && peer_idx < (int)this->discovered_peers_.size()) {
    const auto &peer = this->discovered_peers_[peer_idx];
    return peer.name + " (" + peer.ip + ")";
  }

  return "No peer at index " + std::to_string(this->target_index_);
}

bool UDPIntercom::call_target(const std::string &manual_ip) {
  std::string target_ip;
  uint16_t target_port = this->listen_port_;  // Default to our listen port

  if (this->target_index_ == 0) {
    // Manual IP mode
    if (manual_ip.empty()) {
      ESP_LOGE(TAG, "Cannot call: manual IP not set");
      return false;
    }
    target_ip = manual_ip;
    ESP_LOGI(TAG, "Calling manual IP: %s:%d", target_ip.c_str(), target_port);
  } else {
    // Discovered peer mode
    int peer_idx = this->target_index_ - 1;
    if (peer_idx < 0 || peer_idx >= (int)this->discovered_peers_.size()) {
      ESP_LOGE(TAG, "Cannot call: invalid peer index %d (have %d peers)",
               this->target_index_, (int)this->discovered_peers_.size());
      return false;
    }
    const auto &peer = this->discovered_peers_[peer_idx];
    target_ip = peer.ip;
    target_port = peer.port;
    ESP_LOGI(TAG, "Calling peer: %s (%s:%d)", peer.name.c_str(), target_ip.c_str(), target_port);
  }

  // Start streaming to the target
  this->start_streaming_to_peer(target_ip);
  return true;
}

// =============================================================================
// Volume Control
// =============================================================================
void UDPIntercom::set_volume(float volume) {
  this->volume_ = std::clamp(volume, 0.0f, 1.0f);
  ESP_LOGI(TAG, "Volume set to %.0f%%", this->volume_ * 100);
}

void UDPIntercom::apply_software_volume_(int16_t *buffer, size_t samples) {
  if (this->volume_ >= 0.99f) return;  // No change needed

  for (size_t i = 0; i < samples; i++) {
    int32_t sample = buffer[i];
    sample = (int32_t)(sample * this->volume_);
    buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
  }
}

// =============================================================================
// Operating Mode
// =============================================================================
void UDPIntercom::set_operating_mode(OperatingMode mode) {
  if (this->is_streaming()) {
    ESP_LOGW(TAG, "Cannot change mode while streaming");
    return;
  }
  this->operating_mode_ = mode;
  ESP_LOGI(TAG, "Operating mode set to: %s", mode == OPERATING_MODE_GO2RTC ? "go2rtc" : "P2P");
}

// =============================================================================
// State Management
// =============================================================================
void UDPIntercom::set_state_(IntercomState new_state) {
  if (this->state_ != new_state) {
    ESP_LOGI(TAG, "State: %s -> %s",
             get_state_text_static(this->state_),
             get_state_text_static(new_state));
    this->state_ = new_state;
  }
}

// =============================================================================
// Audio Processing Task
// =============================================================================
void UDPIntercom::audio_task(void *params) {
  UDPIntercom *intercom = (UDPIntercom *)params;

  // Determine frame size (use AEC chunk size if enabled, else default)
  int frame_size = 256;
#ifdef USE_ESP_AEC
  if (intercom->aec_handle_ != nullptr) {
    frame_size = intercom->aec_frame_size_;
  }
#endif
  if (frame_size <= 0) frame_size = 256;
  size_t frame_bytes = frame_size * sizeof(int16_t);

  ESP_LOGI(TAG, "Audio task started: frame_size=%d samples (%d bytes)", frame_size, (int)frame_bytes);

  // Allocate buffers
  // For dual bus (INMP441), we read 32-bit samples and convert to 16-bit
  bool is_dual_bus = (intercom->i2s_mode_ == I2S_MODE_DUAL);
  size_t mic_read_bytes = is_dual_bus ? (frame_size * sizeof(int32_t)) : frame_bytes;

  int16_t *mic_buffer = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
  int32_t *mic_buffer_32 = is_dual_bus ?
    (int32_t *)heap_caps_aligned_alloc(16, mic_read_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int16_t *spk_buffer = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
  int16_t *aec_output = nullptr;
  int16_t *last_speaker = nullptr;

  if (is_dual_bus) {
    ESP_LOGI(TAG, "Dual bus mode: reading %d bytes (32-bit) from INMP441", (int)mic_read_bytes);
  }

#ifdef USE_ESP_AEC
  if (intercom->aec_handle_ != nullptr) {
    aec_output = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    last_speaker = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    if (last_speaker) memset(last_speaker, 0, frame_bytes);
  }
#endif

  if (!mic_buffer || !spk_buffer || (is_dual_bus && !mic_buffer_32)) {
    ESP_LOGE(TAG, "Failed to allocate audio buffers");
    if (mic_buffer) heap_caps_free(mic_buffer);
    if (mic_buffer_32) heap_caps_free(mic_buffer_32);
    if (spk_buffer) heap_caps_free(spk_buffer);
    if (aec_output) heap_caps_free(aec_output);
    if (last_speaker) heap_caps_free(last_speaker);
    vTaskDelete(NULL);
    return;
  }

  uint8_t udp_buffer[AUDIO_BUFFER_SIZE];
  size_t bytes_read, bytes_written;

  // Reset jitter buffer
  jitter_write_pos = 0;
  jitter_read_pos = 0;
  jitter_available = 0;

  const size_t PREBUFFER_THRESHOLD = 2048;
  bool prebuffering = true;

  // Determine target address
  struct sockaddr_in *target_addr;
  if (intercom->operating_mode_ == OPERATING_MODE_P2P) {
    target_addr = &intercom->peer_addr_;
    char target_ip[16];
    inet_ntoa_r(target_addr->sin_addr, target_ip, sizeof(target_ip));
    ESP_LOGI(TAG, "Audio task: P2P mode, sending to %s:%d",
             target_ip, ntohs(target_addr->sin_port));
  } else {
    target_addr = &intercom->server_addr_;
    char target_ip[16];
    inet_ntoa_r(target_addr->sin_addr, target_ip, sizeof(target_ip));
    ESP_LOGI(TAG, "Audio task: go2rtc mode, sending to %s:%d",
             target_ip, ntohs(target_addr->sin_port));
  }

  uint32_t last_stats_log = 0;

  while (intercom->streaming_active_) {
    // ═══════════════════════════════════════════════════════════════════════
    // UDP -> JITTER BUFFER (receive audio from network)
    // ═══════════════════════════════════════════════════════════════════════
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    for (int i = 0; i < 10; i++) {
      ssize_t received = recvfrom(
        intercom->recv_socket_,
        udp_buffer,
        AUDIO_BUFFER_SIZE,
        0,
        (struct sockaddr *)&src_addr,
        &addr_len
      );

      if (received > 0) {
        intercom->rx_packets_++;
        intercom->last_rx_time_ = millis();
        jitter_write(udp_buffer, received);

        // Log first few received packets for debugging
        if (intercom->rx_packets_ <= 5 || intercom->rx_packets_ % 500 == 0) {
          char src_ip[16];
          inet_ntoa_r(src_addr.sin_addr, src_ip, sizeof(src_ip));
          ESP_LOGI(TAG, "RX packet #%u from %s:%d - %d bytes",
                   intercom->rx_packets_, src_ip, ntohs(src_addr.sin_port), (int)received);
        }
      } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGW(TAG, "recvfrom error in audio task: %d (%s)", errno, strerror(errno));
        break;
      } else {
        break;
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // JITTER BUFFER -> SPEAKER
    // ═══════════════════════════════════════════════════════════════════════
    memset(spk_buffer, 0, frame_bytes);

    if (prebuffering) {
      if (jitter_available >= PREBUFFER_THRESHOLD) {
        prebuffering = false;
        ESP_LOGI(TAG, "Prebuffer complete (%d bytes), starting playback", jitter_available);
      } else if (jitter_available > 0 && intercom->rx_packets_ % 100 == 0) {
        ESP_LOGD(TAG, "Prebuffering: %d/%d bytes", jitter_available, PREBUFFER_THRESHOLD);
      }
    }

    if (!prebuffering && jitter_available >= frame_bytes) {
      size_t got = jitter_read((uint8_t*)spk_buffer, frame_bytes);
      if (got == frame_bytes) {
        // Apply software volume if needed
        if (intercom->volume_mode_ == VOLUME_MODE_SOFTWARE ||
            (intercom->volume_mode_ == VOLUME_MODE_AUTO &&
             intercom->i2s_mode_ == I2S_MODE_DUAL)) {
          intercom->apply_software_volume_(spk_buffer, frame_size);
        }

        esp_err_t err = i2s_channel_write(intercom->tx_handle_, spk_buffer, frame_bytes,
                                          &bytes_written, pdMS_TO_TICKS(50));
        if (err != ESP_OK) {
          ESP_LOGD(TAG, "I2S write error: %s", esp_err_to_name(err));
        }

#ifdef USE_ESP_AEC
        if (last_speaker) {
          memcpy(last_speaker, spk_buffer, frame_bytes);
        }
#endif
      }
    } else if (!prebuffering && jitter_available == 0) {
      prebuffering = true;
      ESP_LOGW(TAG, "Buffer underrun, rebuffering...");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MICROPHONE -> (AEC) -> UDP
    // ═══════════════════════════════════════════════════════════════════════
    esp_err_t err;

    static bool logged_first_mic_read = false;

    if (is_dual_bus) {
      // Dual bus mode: read from separate microphone I2S bus
      // Supports both 16-bit and 32-bit microphones via configuration
      const int32_t mic_gain = intercom->mic_gain_;  // Configurable gain
      const bool is_32bit = (intercom->mic_bits_per_sample_ == 32);

      if (is_32bit) {
        // 32-bit mic (e.g., INMP441): read 32-bit samples and convert to 16-bit
        err = i2s_channel_read(intercom->rx_handle_, mic_buffer_32, mic_read_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == mic_read_bytes) {
          // Convert 32-bit to 16-bit: most mics put data in upper bits
          // Take the upper 16 bits (>> 16) and apply configurable gain
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer_32[i] >> 16;
            sample *= mic_gain;
            // Clamp to 16-bit range
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            mic_buffer[i] = (int16_t)sample;
          }
          bytes_read = frame_bytes;  // Normalize to 16-bit size
        }
      } else {
        // 16-bit mic: read directly and apply gain
        err = i2s_channel_read(intercom->rx_handle_, mic_buffer, frame_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == frame_bytes && mic_gain > 1) {
          // Apply gain to 16-bit samples
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer[i] * mic_gain;
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            mic_buffer[i] = (int16_t)sample;
          }
        }
      }

      if (err == ESP_OK && bytes_read > 0) {
        bytes_read = frame_bytes;  // Normalize to 16-bit size for the check below
      }
    } else {
      err = i2s_channel_read(intercom->rx_handle_, mic_buffer, frame_bytes,
                              &bytes_read, pdMS_TO_TICKS(50));
    }

    if (err == ESP_OK && bytes_read == frame_bytes) {
      int16_t *send_buffer = mic_buffer;

#ifdef USE_ESP_AEC
      if (intercom->aec_handle_ != nullptr && aec_output && last_speaker) {
        aec_process(intercom->aec_handle_, mic_buffer, last_speaker, aec_output);
        send_buffer = aec_output;
      }
#endif

      ssize_t sent = sendto(
        intercom->send_socket_,
        send_buffer,
        frame_bytes,
        0,
        (struct sockaddr *)target_addr,
        sizeof(*target_addr)
      );

      if (sent > 0) {
        intercom->tx_packets_++;
      } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGW(TAG, "UDP send error: %d", errno);
      }
    }

    // Periodic stats logging
    uint32_t now = millis();
    if (now - last_stats_log > 5000) {
      char target_ip[16];
      inet_ntoa_r(target_addr->sin_addr, target_ip, sizeof(target_ip));
      ESP_LOGI(TAG, "Streaming stats: TX=%u RX=%u target=%s:%d",
               intercom->tx_packets_, intercom->rx_packets_,
               target_ip, ntohs(target_addr->sin_port));
      last_stats_log = now;
    }
  }

  // Cleanup
  heap_caps_free(mic_buffer);
  if (mic_buffer_32) heap_caps_free(mic_buffer_32);
  heap_caps_free(spk_buffer);
  if (aec_output) heap_caps_free(aec_output);
  if (last_speaker) heap_caps_free(last_speaker);

  ESP_LOGI(TAG, "Audio task stopped");
  vTaskDelete(NULL);
}

// =============================================================================
// Streaming Control
// =============================================================================
void UDPIntercom::start_streaming() {
  if (this->is_streaming()) {
    ESP_LOGW(TAG, "Already streaming");
    return;
  }

  if (this->operating_mode_ == OPERATING_MODE_GO2RTC && this->server_ip_.empty()) {
    ESP_LOGE(TAG, "No server IP configured for go2rtc mode");
    this->set_state_(INTERCOM_STATE_ERROR);
    return;
  }

  ESP_LOGI(TAG, "Starting streaming (%s mode)...",
           this->operating_mode_ == OPERATING_MODE_GO2RTC ? "go2rtc" : "P2P");

  // Initialize I2S based on mode
  bool i2s_ok = false;
  if (this->i2s_mode_ == I2S_MODE_SINGLE) {
    i2s_ok = this->init_i2s_single_bus_();
  } else {
    i2s_ok = this->init_i2s_dual_bus_();
  }

  if (!i2s_ok) {
    this->set_state_(INTERCOM_STATE_ERROR);
    return;
  }

  // Initialize sockets
  if (!this->init_sockets_()) {
    this->deinit_i2s_();
    this->set_state_(INTERCOM_STATE_ERROR);
    return;
  }

  // Initialize AEC if enabled
#ifdef USE_ESP_AEC
  if (this->aec_enabled_) {
    this->aec_handle_ = aec_create(this->sample_rate_, 4, 1, AEC_MODE_VOIP_HIGH_PERF);
    if (this->aec_handle_) {
      this->aec_frame_size_ = aec_get_chunksize(this->aec_handle_);
      ESP_LOGI(TAG, "AEC initialized: %d samples/frame", this->aec_frame_size_);
    } else {
      ESP_LOGW(TAG, "AEC creation failed");
    }
  }
#endif

  this->streaming_active_ = true;
  this->last_rx_time_ = millis();

  // Create audio task
  BaseType_t result = xTaskCreatePinnedToCore(
    audio_task,
    "udp_audio",
    16384,
    this,
    10,
    &this->audio_task_handle_,
    1
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create audio task");
    this->streaming_active_ = false;
    this->close_sockets_();
    this->deinit_i2s_();
    this->set_state_(INTERCOM_STATE_ERROR);
    return;
  }

  this->set_state_(INTERCOM_STATE_STREAMING_DUPLEX);
  this->call_start_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming started!");
}

void UDPIntercom::start_streaming_to_peer(const std::string &peer_ip) {
  ESP_LOGI(TAG, ">>> start_streaming_to_peer() called with IP: '%s'", peer_ip.c_str());

  if (peer_ip.empty()) {
    ESP_LOGE(TAG, "Cannot call peer: IP is empty!");
    return;
  }

  if (this->is_streaming()) {
    ESP_LOGW(TAG, "Already streaming");
    return;
  }

  // Switch to P2P mode
  this->operating_mode_ = OPERATING_MODE_P2P;
  ESP_LOGI(TAG, "Operating mode set to P2P");

  // Set up peer address
  memset(&this->peer_addr_, 0, sizeof(this->peer_addr_));
  this->peer_addr_.sin_family = AF_INET;
  this->peer_addr_.sin_port = htons(this->listen_port_);
  int result = inet_pton(AF_INET, peer_ip.c_str(), &this->peer_addr_.sin_addr);
  if (result != 1) {
    ESP_LOGE(TAG, "Invalid peer IP address: %s (inet_pton returned %d)", peer_ip.c_str(), result);
    return;
  }

  this->current_peer_ip_ = peer_ip;
  this->current_peer_port_ = this->listen_port_;

  ESP_LOGI(TAG, "Starting P2P streaming to %s:%d", peer_ip.c_str(), this->listen_port_);

  this->start_streaming();
}

void UDPIntercom::stop_streaming() {
  if (!this->is_streaming() && this->state_ != INTERCOM_STATE_RINGING && this->state_ != INTERCOM_STATE_ERROR) {
    ESP_LOGW(TAG, "Not streaming");
    return;
  }

  ESP_LOGI(TAG, "Stopping streaming...");

  this->streaming_active_ = false;

  // Wait for audio task
  if (this->audio_task_handle_ != nullptr) {
    int wait_count = 0;
    while (eTaskGetState(this->audio_task_handle_) != eDeleted && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }
    this->audio_task_handle_ = nullptr;
  }

  // Cleanup AEC
#ifdef USE_ESP_AEC
  if (this->aec_handle_) {
    aec_destroy(this->aec_handle_);
    this->aec_handle_ = nullptr;
    this->aec_frame_size_ = 0;
  }
#endif

  this->close_sockets_();
  this->deinit_i2s_();

  this->set_state_(INTERCOM_STATE_IDLE);
  this->call_end_trigger_.trigger();

  // Set cooldown to prevent immediate auto-answer
  this->last_hangup_time_ = millis();

  ESP_LOGI(TAG, "Streaming stopped (cooldown active for 5s)");
}

void UDPIntercom::ring_doorbell() {
  ESP_LOGI(TAG, "Doorbell ring!");
  this->set_state_(INTERCOM_STATE_RINGING);
  this->ring_trigger_.trigger();
}

}  // namespace udp_intercom
}  // namespace esphome
