#include "i2s_audio_udp.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>
#include <algorithm>
#include <errno.h>
#include <driver/gpio.h>

// Include AEC if available
#ifdef USE_ESP_AEC
#include "../esp_aec/esp_aec.h"
#endif

#include "esp_heap_caps.h"

namespace esphome {
namespace i2s_audio_udp {

static const char *TAG = "i2s_audio_udp";

// Audio configuration
static const size_t AUDIO_BUFFER_SIZE = 1024;
static const size_t DMA_BUFFER_COUNT = 8;
static const size_t DMA_BUFFER_SIZE = 512;
static const size_t RING_BUFFER_SIZE = 8192;

// FreeRTOS task parameters
static const size_t TASK_STACK_SIZE = 4096;
static const ssize_t TASK_PRIORITY = 19;

void I2SAudioUDP::setup() {
  ESP_LOGD(TAG, "Setting up...");

  // Deduce bus mode and audio mode from pin configuration
  this->deduce_modes_();

  // Enable speaker amplifier if configured
  if (this->speaker_enable_pin_ >= 0) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << this->speaker_enable_pin_);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)this->speaker_enable_pin_, 1);
  }

  ESP_LOGI(TAG, "Setup complete");
}

void I2SAudioUDP::dump_config() {
  ESP_LOGCONFIG(TAG, "I2S Audio UDP:");
  ESP_LOGCONFIG(TAG, "  Bus Mode: %s", this->bus_mode_ == I2S_BUS_SINGLE ? "SINGLE" : "DUAL");
  ESP_LOGCONFIG(TAG, "  Audio Mode: %s", this->get_audio_mode_text());
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  if (this->bus_mode_ == I2S_BUS_DUAL) {
    ESP_LOGCONFIG(TAG, "  Mic Config: %d-bit, channel=%s, gain=%dx",
                  this->mic_bits_per_sample_,
                  this->mic_channel_ == MIC_CHANNEL_LEFT ? "left" : "right",
                  this->mic_gain_);
  }
  if (this->aec_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  AEC: enabled");
  }
}

void I2SAudioUDP::deduce_modes_() {
  // Determine bus mode
  if (this->mic_lrclk_pin_ >= 0 || this->speaker_lrclk_pin_ >= 0) {
    this->bus_mode_ = I2S_BUS_DUAL;
  } else {
    this->bus_mode_ = I2S_BUS_SINGLE;
  }

  // Determine audio mode
  bool has_mic = false;
  bool has_speaker = false;

  if (this->bus_mode_ == I2S_BUS_SINGLE) {
    has_mic = (this->i2s_din_pin_ >= 0);
    has_speaker = (this->i2s_dout_pin_ >= 0);
  } else {
    has_mic = (this->mic_din_pin_ >= 0);
    has_speaker = (this->speaker_dout_pin_ >= 0);
  }

  if (has_mic && has_speaker) {
    this->audio_mode_ = AUDIO_MODE_FULL_DUPLEX;
  } else if (has_mic) {
    this->audio_mode_ = AUDIO_MODE_TX_ONLY;
  } else {
    this->audio_mode_ = AUDIO_MODE_RX_ONLY;
  }
}

const char* I2SAudioUDP::get_audio_mode_text() const {
  switch (this->audio_mode_) {
    case AUDIO_MODE_TX_ONLY: return "TX_ONLY";
    case AUDIO_MODE_RX_ONLY: return "RX_ONLY";
    case AUDIO_MODE_FULL_DUPLEX: return "FULL_DUPLEX";
    default: return "UNKNOWN";
  }
}

void I2SAudioUDP::set_volume(float volume) {
  this->volume_ = std::clamp(volume, 0.0f, 1.0f);
  ESP_LOGI(TAG, "Volume set to %.0f%%", this->volume_ * 100);
}

void I2SAudioUDP::apply_software_volume_(int16_t *buffer, size_t samples) {
  if (this->volume_ >= 0.99f) return;
  for (size_t i = 0; i < samples; i++) {
    int32_t sample = buffer[i];
    sample = (int32_t)(sample * this->volume_);
    buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
  }
}

bool I2SAudioUDP::init_i2s_single_bus_() {
  ESP_LOGD(TAG, "Initializing I2S Single Bus...");

  bool need_tx = (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool need_rx = (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  i2s_chan_handle_t *tx_ptr = need_tx ? &this->tx_handle_ : nullptr;
  i2s_chan_handle_t *rx_ptr = need_rx ? &this->rx_handle_ : nullptr;

  esp_err_t err = i2s_new_channel(&chan_cfg, tx_ptr, rx_ptr);
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
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };

  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  if (need_tx) {
    err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(err));
      return false;
    }
    i2s_channel_enable(this->tx_handle_);
  }

  if (need_rx) {
    err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(err));
      return false;
    }
    i2s_channel_enable(this->rx_handle_);
  }

  ESP_LOGD(TAG, "I2S Single Bus initialized");
  return true;
}

bool I2SAudioUDP::init_i2s_dual_bus_() {
  ESP_LOGD(TAG, "Initializing I2S Dual Bus...");

  bool need_mic = (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool need_speaker = (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  // Microphone on I2S_NUM_0
  if (need_mic) {
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
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };

    mic_std_cfg.slot_cfg.slot_bit_width = (this->mic_bits_per_sample_ == 32)
        ? I2S_SLOT_BIT_WIDTH_32BIT : I2S_SLOT_BIT_WIDTH_16BIT;
    mic_std_cfg.slot_cfg.slot_mask = (this->mic_channel_ == MIC_CHANNEL_RIGHT)
        ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(this->rx_handle_, &mic_std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init mic RX channel: %s", esp_err_to_name(err));
      i2s_del_channel(this->rx_handle_);
      return false;
    }

    i2s_channel_enable(this->rx_handle_);
    ESP_LOGD(TAG, "Mic initialized: %d-bit, channel=%s, gain=%dx",
             this->mic_bits_per_sample_,
             this->mic_channel_ == MIC_CHANNEL_LEFT ? "left" : "right",
             this->mic_gain_);
  }

  // Speaker on I2S_NUM_1
  if (need_speaker) {
    i2s_chan_config_t spk_chan_cfg = {
      .id = I2S_NUM_1,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = DMA_BUFFER_COUNT,
      .dma_frame_num = DMA_BUFFER_SIZE,
      .auto_clear = true,
      .intr_priority = 0,
    };

    esp_err_t err = i2s_new_channel(&spk_chan_cfg, &this->tx_handle_, nullptr);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create speaker I2S channel: %s", esp_err_to_name(err));
      if (this->rx_handle_) i2s_del_channel(this->rx_handle_);
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
        .mclk = GPIO_NUM_NC,
        .bclk = (gpio_num_t)this->speaker_bclk_pin_,
        .ws = (gpio_num_t)this->speaker_lrclk_pin_,
        .dout = (gpio_num_t)this->speaker_dout_pin_,
        .din = GPIO_NUM_NC,
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };

    spk_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(this->tx_handle_, &spk_std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init speaker TX channel: %s", esp_err_to_name(err));
      if (this->rx_handle_) i2s_del_channel(this->rx_handle_);
      i2s_del_channel(this->tx_handle_);
      return false;
    }

    i2s_channel_enable(this->tx_handle_);
  }

  ESP_LOGD(TAG, "I2S Dual Bus initialized");
  return true;
}

void I2SAudioUDP::deinit_i2s_() {
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
  ESP_LOGD(TAG, "I2S deinitialized");
}

bool I2SAudioUDP::init_sockets_() {
  // Evaluate templatable values now
  if (this->remote_ip_func_) {
    this->remote_ip_ = this->remote_ip_func_();
  }
  if (this->remote_port_func_) {
    this->remote_port_ = this->remote_port_func_();
  }
  if (this->listen_port_func_) {
    this->listen_port_ = this->listen_port_func_();
  }

  ESP_LOGD(TAG, "Network config: remote=%s:%d, listen=%d",
           this->remote_ip_.c_str(), this->remote_port_, this->listen_port_);

  if (this->remote_ip_.empty()) {
    ESP_LOGE(TAG, "Remote IP is empty");
    return false;
  }

  // Setup remote address
  memset(&this->remote_addr_, 0, sizeof(this->remote_addr_));
  this->remote_addr_.sin_family = AF_INET;
  this->remote_addr_.sin_port = htons(this->remote_port_);
  if (inet_pton(AF_INET, this->remote_ip_.c_str(), &this->remote_addr_.sin_addr) != 1) {
    ESP_LOGE(TAG, "Invalid remote IP: %s", this->remote_ip_.c_str());
    return false;
  }

  // Create send socket (for TX)
  if (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX) {
    this->send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->send_socket_ < 0) {
      ESP_LOGE(TAG, "Failed to create send socket: %d", errno);
      return false;
    }
    int flags = fcntl(this->send_socket_, F_GETFL, 0);
    fcntl(this->send_socket_, F_SETFL, flags | O_NONBLOCK);
  }

  // Create receive socket (for RX)
  if (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX) {
    this->recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->recv_socket_ < 0) {
      ESP_LOGE(TAG, "Failed to create receive socket: %d", errno);
      if (this->send_socket_ >= 0) close(this->send_socket_);
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
      ESP_LOGE(TAG, "Failed to bind to port %d: %s", this->listen_port_, strerror(errno));
      if (this->send_socket_ >= 0) close(this->send_socket_);
      close(this->recv_socket_);
      return false;
    }

    int flags = fcntl(this->recv_socket_, F_GETFL, 0);
    fcntl(this->recv_socket_, F_SETFL, flags | O_NONBLOCK);

    int rcvbuf = 32768;
    setsockopt(this->recv_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  }

  ESP_LOGD(TAG, "UDP sockets initialized");
  return true;
}

void I2SAudioUDP::close_sockets_() {
  if (this->send_socket_ >= 0) {
    close(this->send_socket_);
    this->send_socket_ = -1;
  }
  if (this->recv_socket_ >= 0) {
    close(this->recv_socket_);
    this->recv_socket_ = -1;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  ESP_LOGD(TAG, "UDP sockets closed");
}

void I2SAudioUDP::audio_task(void *params) {
  I2SAudioUDP *self = (I2SAudioUDP *)params;

  int frame_size = 256;
#ifdef USE_ESP_AEC
  if (self->aec_ != nullptr && self->aec_->is_initialized()) {
    frame_size = self->aec_->get_frame_size();
  }
#endif
  if (frame_size <= 0) frame_size = 256;
  size_t frame_bytes = frame_size * sizeof(int16_t);

  ESP_LOGD(TAG, "Audio task started: frame_size=%d samples", frame_size);

  bool is_dual_bus = (self->bus_mode_ == I2S_BUS_DUAL);
  bool has_tx = (self->audio_mode_ == AUDIO_MODE_TX_ONLY || self->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool has_rx = (self->audio_mode_ == AUDIO_MODE_RX_ONLY || self->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  size_t mic_read_bytes = is_dual_bus && (self->mic_bits_per_sample_ == 32) ?
      (frame_size * sizeof(int32_t)) : frame_bytes;

  // Allocate buffers
  int16_t *mic_buffer = has_tx ?
      (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int32_t *mic_buffer_32 = (is_dual_bus && has_tx && self->mic_bits_per_sample_ == 32) ?
      (int32_t *)heap_caps_aligned_alloc(16, mic_read_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int16_t *spk_buffer = has_rx ?
      (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int16_t *aec_output = nullptr;
  int16_t *last_speaker = nullptr;

#ifdef USE_ESP_AEC
  if (self->aec_ != nullptr && self->aec_->is_initialized() && has_tx && has_rx) {
    aec_output = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    last_speaker = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    if (last_speaker) memset(last_speaker, 0, frame_bytes);
  }
#endif

  uint8_t udp_buffer[AUDIO_BUFFER_SIZE];
  size_t bytes_read, bytes_written;

  // Create ring buffer
  self->audio_ring_buffer_ = RingBuffer::create(RING_BUFFER_SIZE);
  if (self->audio_ring_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate ring buffer");
    vTaskDelete(NULL);
    return;
  }

  const size_t PREBUFFER_THRESHOLD = 2048;
  bool prebuffering = true;

  uint32_t last_stats_log = 0;

  while (self->streaming_) {
    // ═══════════════════════════════════════════════════════════════════════
    // UDP -> JITTER BUFFER (receive audio from network)
    // ═══════════════════════════════════════════════════════════════════════
    if (has_rx && self->recv_socket_ >= 0) {
      for (int i = 0; i < 10; i++) {
        ssize_t received = recvfrom(self->recv_socket_, udp_buffer, AUDIO_BUFFER_SIZE,
                                     0, nullptr, nullptr);
        if (received > 0) {
          self->rx_packets_++;
          self->audio_ring_buffer_->write((void*)udp_buffer, received);
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          break;
        } else {
          break;
        }
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // RING BUFFER -> SPEAKER
    // ═══════════════════════════════════════════════════════════════════════
    if (has_rx && spk_buffer && self->tx_handle_) {
      memset(spk_buffer, 0, frame_bytes);
      size_t available = self->audio_ring_buffer_->available();

      if (prebuffering) {
        if (available >= PREBUFFER_THRESHOLD) {
          prebuffering = false;
          ESP_LOGD(TAG, "Prebuffer complete, starting playback");
        }
      }

      if (!prebuffering && available >= frame_bytes) {
        size_t got = self->audio_ring_buffer_->read((void*)spk_buffer, frame_bytes, 0);
        if (got == frame_bytes) {
          self->apply_software_volume_(spk_buffer, frame_size);
          i2s_channel_write(self->tx_handle_, spk_buffer, frame_bytes, &bytes_written, pdMS_TO_TICKS(50));

#ifdef USE_ESP_AEC
          if (last_speaker) {
            memcpy(last_speaker, spk_buffer, frame_bytes);
          }
#endif
        }
      } else if (!prebuffering && available == 0) {
        prebuffering = true;
        ESP_LOGW(TAG, "Buffer underrun, rebuffering...");
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MICROPHONE -> (AEC) -> UDP
    // ═══════════════════════════════════════════════════════════════════════
    if (has_tx && mic_buffer && self->rx_handle_) {
      esp_err_t err;
      bool is_32bit = (self->mic_bits_per_sample_ == 32);
      int32_t mic_gain = self->mic_gain_;

      if (is_dual_bus && is_32bit && mic_buffer_32) {
        err = i2s_channel_read(self->rx_handle_, mic_buffer_32, mic_read_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == mic_read_bytes) {
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer_32[i] >> 16;
            sample *= mic_gain;
            mic_buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
          }
          bytes_read = frame_bytes;
        }
      } else {
        err = i2s_channel_read(self->rx_handle_, mic_buffer, frame_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == frame_bytes && mic_gain > 1) {
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer[i] * mic_gain;
            mic_buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
          }
        }
      }

      if (err == ESP_OK && bytes_read == frame_bytes) {
        int16_t *send_buffer = mic_buffer;

#ifdef USE_ESP_AEC
        if (self->aec_ != nullptr && self->aec_->is_initialized() && aec_output && last_speaker) {
          self->aec_->process(mic_buffer, last_speaker, aec_output, frame_size);
          send_buffer = aec_output;
        }
#endif

        if (self->send_socket_ >= 0) {
          ssize_t sent = sendto(self->send_socket_, send_buffer, frame_bytes, 0,
                                 (struct sockaddr *)&self->remote_addr_, sizeof(self->remote_addr_));
          if (sent > 0) {
            self->tx_packets_++;
          }
        }
      }
    }

    // Periodic stats
    uint32_t now = millis();
    if (now - last_stats_log > 5000) {
      ESP_LOGD(TAG, "Stats: TX=%u RX=%u buf=%d", self->tx_packets_, self->rx_packets_,
               self->audio_ring_buffer_->available());
      last_stats_log = now;
    }
  }

  // Cleanup
  if (mic_buffer) heap_caps_free(mic_buffer);
  if (mic_buffer_32) heap_caps_free(mic_buffer_32);
  if (spk_buffer) heap_caps_free(spk_buffer);
  if (aec_output) heap_caps_free(aec_output);
  if (last_speaker) heap_caps_free(last_speaker);
  self->audio_ring_buffer_.reset();

  ESP_LOGD(TAG, "Audio task stopped");
  vTaskDelete(NULL);
}

void I2SAudioUDP::start() {
  if (this->streaming_) {
    ESP_LOGW(TAG, "Already streaming");
    return;
  }

  ESP_LOGI(TAG, "Starting audio streaming...");

  // Initialize I2S
  bool i2s_ok = false;
  if (this->bus_mode_ == I2S_BUS_SINGLE) {
    i2s_ok = this->init_i2s_single_bus_();
  } else {
    i2s_ok = this->init_i2s_dual_bus_();
  }

  if (!i2s_ok) {
    this->on_error_trigger_.trigger("I2S initialization failed");
    return;
  }

  // Initialize sockets
  if (!this->init_sockets_()) {
    this->deinit_i2s_();
    this->on_error_trigger_.trigger("Socket initialization failed");
    return;
  }

  this->streaming_ = true;
  this->tx_packets_ = 0;
  this->rx_packets_ = 0;

  // Create audio task
  BaseType_t result = xTaskCreatePinnedToCore(
    audio_task, "i2s_audio_udp", TASK_STACK_SIZE, this, TASK_PRIORITY, &this->audio_task_handle_, 1
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create audio task");
    this->streaming_ = false;
    this->close_sockets_();
    this->deinit_i2s_();
    this->on_error_trigger_.trigger("Task creation failed");
    return;
  }

  this->on_start_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming started!");
}

void I2SAudioUDP::stop() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Not streaming");
    return;
  }

  ESP_LOGD(TAG, "Stopping audio streaming...");

  this->streaming_ = false;

  // Wait for audio task
  if (this->audio_task_handle_ != nullptr) {
    int wait_count = 0;
    while (eTaskGetState(this->audio_task_handle_) != eDeleted && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }
    this->audio_task_handle_ = nullptr;
  }

  this->close_sockets_();
  this->deinit_i2s_();

  this->on_stop_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming stopped");
}

}  // namespace i2s_audio_udp
}  // namespace esphome
