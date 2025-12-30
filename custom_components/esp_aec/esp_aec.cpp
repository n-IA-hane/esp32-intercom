#include "esp_aec.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_aec {

static const char *TAG = "esp_aec";

void EspAec::setup() {
#ifdef USE_ESP_AEC
  ESP_LOGI(TAG, "Initializing AEC...");
  ESP_LOGI(TAG, "  Sample rate: %d Hz", this->sample_rate_);
  ESP_LOGI(TAG, "  Filter length: %d", this->filter_length_);

  this->aec_handle_ = aec_create(this->sample_rate_, this->filter_length_, 1, AEC_MODE_VOIP_HIGH_PERF);
  if (this->aec_handle_ != nullptr) {
    this->frame_size_ = aec_get_chunksize(this->aec_handle_);
    this->initialized_ = true;
    ESP_LOGI(TAG, "AEC initialized: %d samples/frame", this->frame_size_);
  } else {
    ESP_LOGE(TAG, "Failed to create AEC handle");
    this->mark_failed();
  }
#else
  ESP_LOGW(TAG, "AEC not available - ESP-SR not compiled in");
  this->mark_failed();
#endif
}

void EspAec::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP AEC:");
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Filter Length: %d", this->filter_length_);
  ESP_LOGCONFIG(TAG, "  Frame Size: %d samples", this->frame_size_);
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->initialized_ ? "YES" : "NO");
}

void EspAec::process(int16_t *mic_input, int16_t *speaker_ref, int16_t *output, size_t samples) {
#ifdef USE_ESP_AEC
  if (this->aec_handle_ != nullptr && samples == (size_t)this->frame_size_) {
    aec_process(this->aec_handle_, mic_input, speaker_ref, output);
  } else {
    // Fallback: copy input to output
    memcpy(output, mic_input, samples * sizeof(int16_t));
  }
#else
  memcpy(output, mic_input, samples * sizeof(int16_t));
#endif
}

}  // namespace esp_aec
}  // namespace esphome
