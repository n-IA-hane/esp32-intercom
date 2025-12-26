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

#include "udp_intercom.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>
#include <errno.h>
#include <driver/gpio.h>

namespace esphome {
namespace udp_intercom {

static const char *TAG = "udp_intercom";

// =============================================================================
// Audio Buffer Configuration
// These values are optimized for smooth playback with minimal latency
// =============================================================================
static const size_t AUDIO_BUFFER_SIZE = 1024;   // Bytes per chunk (512 samples = 32ms at 16kHz)
static const size_t DMA_BUFFER_COUNT = 8;       // Number of DMA buffers for smooth audio
static const size_t DMA_BUFFER_SIZE = 512;      // Frames per DMA buffer

// =============================================================================
// Jitter Buffer for Incoming UDP Audio
// This buffer smooths out network timing variations
// =============================================================================
static const size_t JITTER_BUFFER_SIZE = 8192;  // 256ms of audio buffer
static uint8_t jitter_buffer[JITTER_BUFFER_SIZE];
static volatile size_t jitter_write_pos = 0;
static volatile size_t jitter_read_pos = 0;
static volatile size_t jitter_available = 0;

// =============================================================================
// Component Setup
// =============================================================================
void UDPIntercom::setup() {
  ESP_LOGI(TAG, "Setting up UDP Intercom Full Duplex...");
  ESP_LOGI(TAG, "  Server: %s:%d", this->server_ip_.c_str(), this->server_port_);
  ESP_LOGI(TAG, "  Listen port: %d", this->listen_port_);
  ESP_LOGI(TAG, "  Sample rate: %d Hz", this->sample_rate_);
  ESP_LOGI(TAG, "  I2S pins: LRCLK=%d, BCLK=%d, MCLK=%d, DIN=%d, DOUT=%d",
           this->i2s_lrclk_pin_, this->i2s_bclk_pin_, this->i2s_mclk_pin_,
           this->i2s_din_pin_, this->i2s_dout_pin_);

  // Initialize server address structure for UDP
  memset(&this->server_addr_, 0, sizeof(this->server_addr_));
  this->server_addr_.sin_family = AF_INET;
  this->server_addr_.sin_port = htons(this->server_port_);
  inet_pton(AF_INET, this->server_ip_.c_str(), &this->server_addr_.sin_addr);

  // Enable speaker amplifier if pin is configured
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

  this->set_state_(IntercomState::IDLE);
  ESP_LOGI(TAG, "UDP Intercom ready");
}

void UDPIntercom::loop() {
  // Main loop is empty - audio processing happens in dedicated FreeRTOS task
}

void UDPIntercom::dump_config() {
  ESP_LOGCONFIG(TAG, "UDP Intercom Full Duplex:");
  ESP_LOGCONFIG(TAG, "  Server IP: %s", this->server_ip_.c_str());
  ESP_LOGCONFIG(TAG, "  Server Port: %d", this->server_port_);
  ESP_LOGCONFIG(TAG, "  Listen Port: %d", this->listen_port_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  I2S LRCLK: GPIO%d", this->i2s_lrclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S BCLK: GPIO%d", this->i2s_bclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S MCLK: GPIO%d", this->i2s_mclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S DIN: GPIO%d", this->i2s_din_pin_);
  ESP_LOGCONFIG(TAG, "  I2S DOUT: GPIO%d", this->i2s_dout_pin_);
}

// =============================================================================
// I2S Full Duplex Initialization
// Creates both TX and RX channels on the same I2S port for true full duplex
// =============================================================================
bool UDPIntercom::init_i2s_full_duplex_() {
  ESP_LOGI(TAG, "Initializing I2S Full Duplex...");

  // Channel configuration for full duplex
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  // Create both TX and RX channels on the same I2S port
  esp_err_t err = i2s_new_channel(&chan_cfg, &this->tx_handle_, &this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "I2S channels created (TX and RX)");

  // Standard mode configuration - Philips format, 16-bit mono
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

  // Use left channel only for mono audio
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  // Initialize TX channel (speaker output)
  err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->tx_handle_ = nullptr;
    this->rx_handle_ = nullptr;
    return false;
  }

  // Initialize RX channel (microphone input) with same config
  err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->tx_handle_ = nullptr;
    this->rx_handle_ = nullptr;
    return false;
  }

  // Enable both channels
  err = i2s_channel_enable(this->tx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->tx_handle_ = nullptr;
    this->rx_handle_ = nullptr;
    return false;
  }

  err = i2s_channel_enable(this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(err));
    i2s_channel_disable(this->tx_handle_);
    i2s_del_channel(this->tx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->tx_handle_ = nullptr;
    this->rx_handle_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "I2S Full Duplex initialized successfully!");
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
// UDP Socket Initialization
// =============================================================================
bool UDPIntercom::init_sockets_() {
  // Create send socket for microphone audio
  this->send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->send_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create send socket: %d", errno);
    return false;
  }

  // Set send socket to non-blocking
  int flags = fcntl(this->send_socket_, F_GETFL, 0);
  fcntl(this->send_socket_, F_SETFL, flags | O_NONBLOCK);

  // Create receive socket for speaker audio
  this->recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->recv_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create receive socket: %d", errno);
    close(this->send_socket_);
    this->send_socket_ = -1;
    return false;
  }

  // Allow address reuse
  int reuse = 1;
  setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Bind receive socket to listen port
  struct sockaddr_in listen_addr;
  memset(&listen_addr, 0, sizeof(listen_addr));
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_addr.s_addr = INADDR_ANY;
  listen_addr.sin_port = htons(this->listen_port_);

  if (bind(this->recv_socket_, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind receive socket to port %d: %d", this->listen_port_, errno);
    close(this->send_socket_);
    close(this->recv_socket_);
    this->send_socket_ = -1;
    this->recv_socket_ = -1;
    return false;
  }

  // Set receive socket to non-blocking
  flags = fcntl(this->recv_socket_, F_GETFL, 0);
  fcntl(this->recv_socket_, F_SETFL, flags | O_NONBLOCK);

  // Set receive buffer size for better performance
  int rcvbuf = 32768;
  setsockopt(this->recv_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  ESP_LOGI(TAG, "UDP sockets initialized");
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
  ESP_LOGI(TAG, "UDP sockets closed");
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
// Jitter Buffer Helper Functions
// These provide a circular buffer for smoothing UDP packet timing
// =============================================================================

// Write data to jitter buffer (returns bytes written)
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

// Read data from jitter buffer (returns bytes read)
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
// Audio Processing Task
// This runs in a separate FreeRTOS task for real-time audio handling
// =============================================================================
void UDPIntercom::audio_task(void *params) {
  UDPIntercom *intercom = (UDPIntercom *)params;

  uint8_t mic_buffer[AUDIO_BUFFER_SIZE];
  uint8_t udp_buffer[AUDIO_BUFFER_SIZE];
  uint8_t spk_buffer[AUDIO_BUFFER_SIZE];
  size_t bytes_read, bytes_written;

  // Reset jitter buffer
  jitter_write_pos = 0;
  jitter_read_pos = 0;
  jitter_available = 0;

  // Pre-buffer threshold: wait until we have some audio before playing
  // This helps prevent choppy audio at the start
  const size_t PREBUFFER_THRESHOLD = 2048;  // 64ms of audio
  bool prebuffering = true;

  ESP_LOGI(TAG, "Audio task started - Full Duplex with jitter buffer");

  while (intercom->streaming_active_) {
    // === MICROPHONE -> UDP (TX) ===
    // Read from I2S mic - this naturally paces the loop
    esp_err_t err = i2s_channel_read(intercom->rx_handle_, mic_buffer, AUDIO_BUFFER_SIZE,
                                      &bytes_read, pdMS_TO_TICKS(50));
    if (err == ESP_OK && bytes_read > 0) {
      ssize_t sent = sendto(
        intercom->send_socket_,
        mic_buffer,
        bytes_read,
        0,
        (struct sockaddr *)&intercom->server_addr_,
        sizeof(intercom->server_addr_)
      );

      if (sent > 0) {
        intercom->tx_packets_++;
      } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGW(TAG, "UDP send error: %d", errno);
      }
    }

    // === UDP -> JITTER BUFFER ===
    // Read all available UDP packets into jitter buffer
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    for (int i = 0; i < 10; i++) {  // Process up to 10 packets per iteration
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
        jitter_write(udp_buffer, received);
      } else {
        break;  // No more packets available
      }
    }

    // === JITTER BUFFER -> SPEAKER ===
    if (prebuffering) {
      // Wait until we have enough audio buffered
      if (jitter_available >= PREBUFFER_THRESHOLD) {
        prebuffering = false;
        ESP_LOGI(TAG, "Prebuffer complete, starting playback (%d bytes)", jitter_available);
      }
    } else {
      // Play audio from jitter buffer
      size_t to_play = (jitter_available > AUDIO_BUFFER_SIZE) ? AUDIO_BUFFER_SIZE : jitter_available;

      if (to_play > 0) {
        size_t got = jitter_read(spk_buffer, to_play);
        if (got > 0) {
          err = i2s_channel_write(intercom->tx_handle_, spk_buffer, got,
                                  &bytes_written, pdMS_TO_TICKS(50));
          if (err != ESP_OK) {
            ESP_LOGD(TAG, "I2S write error: %s", esp_err_to_name(err));
          }
        }
      } else {
        // Buffer underrun - go back to prebuffering
        prebuffering = true;
        ESP_LOGW(TAG, "Buffer underrun, rebuffering...");
      }
    }
  }

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

  ESP_LOGI(TAG, "Starting Full Duplex streaming...");

  // Initialize I2S
  if (!this->init_i2s_full_duplex_()) {
    this->set_state_(IntercomState::ERROR);
    return;
  }

  // Initialize sockets
  if (!this->init_sockets_()) {
    this->deinit_i2s_();
    this->set_state_(IntercomState::ERROR);
    return;
  }

  this->streaming_active_ = true;

  // Create audio processing task with larger stack for buffers
  BaseType_t result = xTaskCreatePinnedToCore(
    audio_task,
    "udp_audio",
    8192,  // Stack size - needs to be large enough for audio buffers
    this,
    10,    // Priority
    &this->audio_task_handle_,
    1      // Pin to core 1 (keeps core 0 free for WiFi/system tasks)
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create audio task");
    this->streaming_active_ = false;
    this->close_sockets_();
    this->deinit_i2s_();
    this->set_state_(IntercomState::ERROR);
    return;
  }

  this->set_state_(IntercomState::STREAMING_DUPLEX);
  this->call_start_trigger_.trigger();
  ESP_LOGI(TAG, "Full Duplex streaming started!");
}

void UDPIntercom::stop_streaming() {
  if (!this->is_streaming() && this->state_ != IntercomState::RINGING && this->state_ != IntercomState::ERROR) {
    ESP_LOGW(TAG, "Not streaming");
    return;
  }

  ESP_LOGI(TAG, "Stopping streaming...");

  this->streaming_active_ = false;

  // Wait for audio task to finish
  if (this->audio_task_handle_ != nullptr) {
    vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to exit
    this->audio_task_handle_ = nullptr;
  }

  // Cleanup
  this->close_sockets_();
  this->deinit_i2s_();

  this->set_state_(IntercomState::IDLE);
  this->call_end_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming stopped");
}

void UDPIntercom::ring_doorbell() {
  ESP_LOGI(TAG, "Doorbell ring!");
  this->set_state_(IntercomState::RINGING);
  this->ring_trigger_.trigger();
}

}  // namespace udp_intercom
}  // namespace esphome
