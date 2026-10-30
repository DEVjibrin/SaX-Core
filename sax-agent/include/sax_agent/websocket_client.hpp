// Non-blocking WebSocket client with supervised reconnects, built on ixwebsocket.
//
// A supervisor thread owns one ix::WebSocket per connection attempt. Each attempt ends when the
// socket closes, errors, never opens, or stalls (its send buffer stays full). The supervisor then
// waits an exponentially growing, jittered delay before the next attempt. Telemetry is real-time
// data, so frames are never queued: send_text() drops the frame instead of buffering stale data.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <string_view>
#include <thread>

namespace ix {
class WebSocket;
}

namespace sax {

enum class LinkState { Stopped, Connecting, Connected, Backoff };

[[nodiscard]] std::string_view to_string(LinkState state) noexcept;

struct WebSocketClientConfig {
  std::string url;           // for example ws://bridge.example.com:8000/ws/agent/robot_01
  std::string bearer_token;  // sent as "Authorization: Bearer <token>" when not empty
  std::chrono::milliseconds backoff_min{500};
  std::chrono::milliseconds backoff_max{30'000};
  double backoff_jitter{0.2};  // +/- fraction applied to each delay, so a fleet does not reconnect in lockstep
  std::chrono::milliseconds connect_timeout{15'000};
  std::chrono::milliseconds stable_after{5'000};  // sessions shorter than this keep growing the backoff
  std::chrono::milliseconds stall_timeout{8'000};  // reconnect when the send buffer stays full this long
  std::size_t max_buffered_bytes{256 * 1024};
  int ping_interval_s{10};
  bool compress{true};  // permessage-deflate; SX1 text compresses roughly 3-5x
};

struct LinkStats {
  std::uint64_t messages_sent{0};
  std::uint64_t messages_dropped{0};
  std::uint64_t messages_received{0};
  std::uint64_t bytes_sent{0};
  std::uint64_t connects{0};
  std::uint64_t disconnects{0};
  std::uint64_t handler_errors{0};
};

class WebSocketClient {
 public:
  /// Called on the WebSocket thread for every inbound text message. Exceptions are caught and counted.
  using MessageHandler = std::function<void(std::string_view)>;
  /// Called on the supervisor or WebSocket thread. Must not call stop().
  using StateHandler = std::function<void(LinkState, std::string_view detail)>;

  /// Throws std::invalid_argument for an empty URL or inconsistent backoff settings.
  WebSocketClient(WebSocketClientConfig config, MessageHandler on_message, StateHandler on_state = {});
  ~WebSocketClient();

  WebSocketClient(const WebSocketClient&) = delete;
  WebSocketClient& operator=(const WebSocketClient&) = delete;

  void start();
  /// Stops the supervisor and closes the socket. Safe to call repeatedly; start() may be called again.
  void stop();

  /// Non-blocking. Returns false (and counts a drop) when the link is down or congested.
  [[nodiscard]] bool send_text(std::string_view payload);

  [[nodiscard]] LinkState state() const noexcept { return state_.load(std::memory_order_acquire); }
  [[nodiscard]] LinkStats stats() const noexcept;

  /// Delay before reconnect attempt number `attempt` (0 is the first retry). `unit_random` in [0, 1] drives the jitter.
  [[nodiscard]] static std::chrono::milliseconds backoff_delay(const WebSocketClientConfig& config, unsigned attempt,
                                                               double unit_random) noexcept;

 private:
  struct Session;
  struct SessionResult;

  void supervise();
  SessionResult run_session(const std::shared_ptr<ix::WebSocket>& socket, const std::shared_ptr<Session>& session);
  void configure(ix::WebSocket& socket, const std::shared_ptr<Session>& session);
  void set_state(LinkState state, std::string_view detail);
  void deliver(std::string_view text) noexcept;

  WebSocketClientConfig config_;
  MessageHandler on_message_;
  StateHandler on_state_;

  std::atomic<LinkState> state_{LinkState::Stopped};
  std::atomic<bool> stop_requested_{false};
  std::mutex lifecycle_mutex_;
  std::thread supervisor_;

  std::mutex socket_mutex_;
  std::shared_ptr<ix::WebSocket> socket_;

  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::mt19937_64 rng_;

  std::atomic<std::uint64_t> messages_sent_{0};
  std::atomic<std::uint64_t> messages_dropped_{0};
  std::atomic<std::uint64_t> messages_received_{0};
  std::atomic<std::uint64_t> bytes_sent_{0};
  std::atomic<std::uint64_t> connects_{0};
  std::atomic<std::uint64_t> disconnects_{0};
  std::atomic<std::uint64_t> handler_errors_{0};
};

}  // namespace sax
