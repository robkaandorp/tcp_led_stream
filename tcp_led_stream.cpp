#include "tcp_led_stream.h"
#ifdef USE_NETWORK
#include <array>

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

namespace esphome {
namespace tcp_led_stream {

static const char *const TAG = "tcp_led_stream";
// Frame format:
//  0-3  Magic bytes 'LEDS'
//  4    Version (0x01)
//  5-8  Pixel count (uint32_t big endian)
//  9    Pixel format enum
// 10-?  Pixel data (count * (3|4) bytes)
// No checksum for now (could add CRC32 later)
// Connection sends full frame each time.

void TCPLedStreamComponent::setup() {
  if (this->lights_.empty()) {
    ESP_LOGE(TAG, "No lights configured");
    this->mark_failed();
    return;
  }

  this->outputs_.clear();
  this->total_led_count_ = 0;
  for (auto *light_state : this->lights_) {
    if (light_state == nullptr) {
      ESP_LOGE(TAG, "Invalid light configured");
      this->mark_failed();
      return;
    }
    auto *addr = static_cast<light::AddressableLight *>(light_state->get_output());
    if (addr == nullptr) {
      ESP_LOGE(TAG, "Configured light is not addressable");
      this->mark_failed();
      return;
    }
    this->outputs_.push_back(addr);
    this->total_led_count_ += addr->size();
  }

  this->server_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
  if (!this->server_) {
    ESP_LOGE(TAG, "Failed to create server socket");
    this->mark_failed();
    return;
  }
  int enable = 1;
  if (this->server_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) != 0) {
    ESP_LOGW(TAG, "setsockopt reuseaddr failed errno=%d", errno);
  }
  if (this->server_->setblocking(false) != 0) {
    ESP_LOGE(TAG, "Failed to set nonblocking");
    this->mark_failed();
    return;
  }
  struct sockaddr_storage addr;
  socklen_t sl = socket::set_sockaddr_any((struct sockaddr *) &addr, sizeof(addr), this->port_);
  if (sl == 0) {
    ESP_LOGE(TAG, "Failed to set sockaddr errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (this->server_->bind((struct sockaddr *) &addr, sl) != 0) {
    ESP_LOGE(TAG, "Bind failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (this->server_->listen(1) != 0) {
    ESP_LOGE(TAG, "Listen failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Listening on port %u for LED frames", this->port_);
  this->last_stats_publish_ = App.get_loop_component_start_time();
#ifdef USE_BINARY_SENSOR
  if (this->client_connected_binary_sensor_ != nullptr) {
    this->client_connected_binary_sensor_->publish_state(false);
  }
#endif
}

bool TCPLedStreamComponent::apply_pixels_(const uint8_t *data, uint32_t count) {
  if (this->outputs_.empty())
    return false;

  size_t bpp = (this->format_ == RGBW || this->format_ == GRBW) ? 4 : 3;
  uint32_t global_index = 0;
  uint8_t r, g, b, w = 0;

  for (auto *addr : this->outputs_) {
    uint32_t strip_size = addr->size();
    uint32_t remaining_pixels = (global_index < count) ? (count - global_index) : 0;
    uint32_t pixels_for_strip = std::min(strip_size, remaining_pixels);

    for (uint32_t i = 0; i < pixels_for_strip; i++) {
      uint32_t src_idx = (global_index + i) * bpp;
      switch (this->format_) {
        case RGB:
          r = data[src_idx + 0];
          g = data[src_idx + 1];
          b = data[src_idx + 2];
          w = (r + g + b) / 3;
          break;
        case GRB:
          g = data[src_idx + 0];
          r = data[src_idx + 1];
          b = data[src_idx + 2];
          w = (r + g + b) / 3;
          break;
        case BGR:
          b = data[src_idx + 0];
          g = data[src_idx + 1];
          r = data[src_idx + 2];
          w = (r + g + b) / 3;
          break;
        case RGBW:
          r = data[src_idx + 0];
          g = data[src_idx + 1];
          b = data[src_idx + 2];
          w = data[src_idx + 3];
          break;
        case GRBW:
          g = data[src_idx + 0];
          r = data[src_idx + 1];
          b = data[src_idx + 2];
          w = data[src_idx + 3];
          break;
      }
      (*addr)[i].set(Color(r, g, b, w));
    }

    global_index += pixels_for_strip;
    if (global_index >= count)
      break;
  }

  for (auto *addr : this->outputs_) {
    addr->schedule_show();
  }

  return true;
}

bool TCPLedStreamComponent::read_frame_() {
  while (true) {
    if (this->receive_state_ == WAITING_HEADER) {
      // Try to read more header bytes
      size_t remaining = sizeof(this->header_buffer_) - this->header_bytes_received_;
      ssize_t r = this->client_->read(this->header_buffer_ + this->header_bytes_received_, remaining);

      if (r == -1) {
        return true;  // No data available, try again next loop
      }
      if (r == 0) {
        ESP_LOGW(TAG, "Client closed connection during header");
        return false;
      }

      this->header_bytes_received_ += r;

      // Check if we have complete header
      if (this->header_bytes_received_ < sizeof(this->header_buffer_)) {
        return true;  // Still waiting for complete header
      }

      // Validate header
      if (memcmp(this->header_buffer_, "LEDS", 4) != 0) {
        ESP_LOGW(TAG, "Bad magic: got %02X %02X %02X %02X, expected 'LEDS'", this->header_buffer_[0],
                 this->header_buffer_[1], this->header_buffer_[2], this->header_buffer_[3]);
        this->reset_receive_state_();
        return false;
      }

      if (this->header_buffer_[4] != 0x01) {
        ESP_LOGW(TAG, "Unsupported protocol version %u", this->header_buffer_[4]);
        this->reset_receive_state_();
        return false;
      }

      uint32_t count = (this->header_buffer_[5] << 24) | (this->header_buffer_[6] << 16) |
                       (this->header_buffer_[7] << 8) | this->header_buffer_[8];
      PixelFormat frame_fmt = (PixelFormat) this->header_buffer_[9];

      if (count == 0 || count > 5000) {
        ESP_LOGW(TAG, "Invalid pixel count %u", (unsigned) count);
        this->reset_receive_state_();
        return false;
      }

      // Calculate expected payload size
      size_t bpp = (frame_fmt == RGBW || frame_fmt == GRBW) ? 4 : 3;
      this->expected_payload_size_ = count * bpp;
      this->payload_bytes_received_ = 0;
      this->rx_buffer_.resize(this->expected_payload_size_);

      // Store frame format for processing
      this->format_ = frame_fmt;

      // Transition to payload reading
      this->receive_state_ = WAITING_PAYLOAD;
      this->last_activity_ = App.get_loop_component_start_time();

      // Continue to payload reading in same loop iteration
      continue;
    }

    if (this->receive_state_ == WAITING_PAYLOAD) {
      // Try to read more payload bytes
      size_t remaining = this->expected_payload_size_ - this->payload_bytes_received_;
      ssize_t r = this->client_->read(this->rx_buffer_.data() + this->payload_bytes_received_, remaining);

      if (r == -1) {
        return true;  // No data available, try again next loop
      }
      if (r == 0) {
        ESP_LOGW(TAG, "Client closed connection during payload");
        this->reset_receive_state_();
        return false;
      }

      this->payload_bytes_received_ += r;

      // Check if we have complete payload
      if (this->payload_bytes_received_ < this->expected_payload_size_) {
        return true;  // Still waiting for complete payload
      }

      // Process complete frame
      uint32_t count = (this->header_buffer_[5] << 24) | (this->header_buffer_[6] << 16) |
                       (this->header_buffer_[7] << 8) | this->header_buffer_[8];

      // Overlap detection
      uint32_t now = App.get_loop_component_start_time();
      uint32_t window_ms = this->frame_completion_interval_ms_;
      if (this->completion_mode_ == "estimate") {
        if (this->total_led_count_ > 0) {
          uint32_t est = (uint64_t) this->total_led_count_ * this->show_time_per_led_us_ / 1000ULL + 2;
          window_ms = est > 1 ? est : 1;
        }
      }
      if (this->frame_in_progress_ && (now - this->last_frame_time_ < window_ms)) {
        this->overlaps_++;
      }

      this->frame_in_progress_ = true;
      this->last_frame_time_ = now;
      this->apply_pixels_(this->rx_buffer_.data(), count);
      this->frame_count_++;
      this->bytes_received_ += (uint32_t) (10 + this->rx_buffer_.size());
      this->last_activity_ = now;

      // Reset state for next frame
      this->reset_receive_state_();
      return true;
    }
  }
}

void TCPLedStreamComponent::loop() {
  // Accept new client if none
  if (!this->client_ && this->server_ && this->server_->ready()) {
    struct sockaddr_storage src;
    socklen_t sl = sizeof(src);
    auto sock = this->server_->accept_loop_monitored((struct sockaddr *) &src, &sl);
    if (sock) {
      this->client_ = std::move(sock);
      this->client_->setblocking(false);
      this->last_activity_ = App.get_loop_component_start_time();
      this->reset_receive_state_();  // Reset buffering state for new connection
      std::array<char, socket::SOCKADDR_STR_LEN> peer{};
      if (this->client_->getpeername_to(peer) > 0) {
        ESP_LOGI(TAG, "Client connected %s", peer.data());
      } else {
        ESP_LOGI(TAG, "Client connected");
      }
      this->connects_++;
#ifdef USE_BINARY_SENSOR
      if (this->client_connected_binary_sensor_ != nullptr) {
        this->client_connected_binary_sensor_->publish_state(true);
      }
#endif
    }
  }
  if (this->client_) {
    if (!this->read_frame_()) {
      // Connection error or protocol violation - close connection
      ESP_LOGI(TAG, "Closing connection due to error");
      this->client_->close();
      this->client_.reset();
      this->disconnects_++;
      this->frame_in_progress_ = false;
      this->reset_receive_state_();
#ifdef USE_BINARY_SENSOR
      if (this->client_connected_binary_sensor_ != nullptr) {
        this->client_connected_binary_sensor_->publish_state(false);
      }
#endif
    } else if (this->timeout_ms_ && (App.get_loop_component_start_time() - this->last_activity_ > this->timeout_ms_)) {
      ESP_LOGI(TAG, "Connection timeout");
      this->client_->close();
      this->client_.reset();
      this->disconnects_++;
      this->frame_in_progress_ = false;
      this->reset_receive_state_();
#ifdef USE_BINARY_SENSOR
      if (this->client_connected_binary_sensor_ != nullptr) {
        this->client_connected_binary_sensor_->publish_state(false);
      }
#endif
    }
  }

  // Heuristic: mark frame complete when window elapsed
  uint32_t window_ms2 = this->frame_completion_interval_ms_;
  if (this->completion_mode_ == "estimate") {
    if (this->total_led_count_ > 0) {
      uint32_t est = (uint64_t) this->total_led_count_ * this->show_time_per_led_us_ / 1000ULL + 2;
      window_ms2 = est > 1 ? est : 1;
    }
  }
  if (this->frame_in_progress_ && (App.get_loop_component_start_time() - this->last_frame_time_ >= window_ms2)) {
    this->frame_in_progress_ = false;
  }

  this->publish_stats_();
}

void TCPLedStreamComponent::publish_stats_() {
  uint32_t now = App.get_loop_component_start_time();
  if (now - this->last_stats_publish_ < 1000)
    return;
  float seconds = (now - this->last_stats_publish_) / 1000.0f;
  this->last_stats_publish_ = now;
#ifdef USE_SENSOR
  static uint32_t last_frame_count = 0;  // acceptable static for diff calculation
  if (this->frame_rate_sensor_ != nullptr) {
    uint32_t diff = this->frame_count_ - last_frame_count;
    this->frame_rate_sensor_->publish_state(diff / seconds);
  }
  last_frame_count = this->frame_count_;
  if (this->bytes_received_sensor_ != nullptr) {
    this->bytes_received_sensor_->publish_state(this->bytes_received_);
  }
  if (this->connects_sensor_ != nullptr) {
    this->connects_sensor_->publish_state(this->connects_);
  }
  if (this->disconnects_sensor_ != nullptr) {
    this->disconnects_sensor_->publish_state(this->disconnects_);
  }
  if (this->overlaps_sensor_ != nullptr) {
    this->overlaps_sensor_->publish_state(this->overlaps_);
  }
#endif
}

void TCPLedStreamComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TCP LED Stream:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Timeout (ms): %u", this->timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Completion mode: %s", this->completion_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Frame completion interval (ms): %u", this->frame_completion_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Show time per LED (us): %u", this->show_time_per_led_us_);
}

void TCPLedStreamComponent::reset_receive_state_() {
  this->receive_state_ = WAITING_HEADER;
  this->header_bytes_received_ = 0;
  this->payload_bytes_received_ = 0;
  this->expected_payload_size_ = 0;
}

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
