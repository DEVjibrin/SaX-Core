#include "sax_agent/websocket_client.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>
#include <stdexcept>
#include <utility>

#include <ixwebsocket/IXWebSocket.h>

namespace sax {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::chrono::milliseconds kPollInterval{100};

bool starts_with(const std::string_view text, const std::string_view prefix) noexcept {
  return text.size() >= prefix.size() && text.substr(0, prefix.size()) == prefix;
}

}  // namespace

struct WebSocketClient::Session {
  std::mutex mutex;
  std::condition_variable cv;
  bool opened{false};
  bool ended{false};
  Clock::time_point opened_at{};
  std::string detail;

  void finish(std::string reason) {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!ended) {
        ended = true;
        detail = std::move(reason);
      }
    }
    cv.notify_all();
  }
};

struct WebSocketClient::SessionResult {
  bool opened{false};
  Clock::duration open_for{};
  std::string detail;
};

std::string_view to_string(const LinkState state) noexcept {
  switch (state) {
    case LinkState::Stopped:
      return "stopped";
    case LinkState::Connecting:
      return "connecting";
    case LinkState::Connected:
      return "connected";
    case LinkState::Backoff:
      return "backoff";
  }
  return "unknown";
}

WebSocketClient::WebSocketClient(WebSocketClientConfig config, MessageHandler on_message, StateHandler on_state)
    : config_(std::move(config)),
      on_message_(std::move(on_message)),
      on_state_(std::move(on_state)),
      rng_(std::random_device{}()) {
  if (config_.url.empty()) {
    throw std::invalid_argument("WebSocket URL must not be empty");
  }
  if (config_.backoff_min.count() <= 0 || config_.backoff_max < config_.backoff_min) {
    throw std::invalid_argument("backoff_min must be positive and not exceed backoff_max");
  }
}

WebSocketClient::~WebSocketClient() {
  stop();
}

std::chrono::milliseconds WebSocketClient::backoff_delay(const WebSocketClientConfig& config, const unsigned attempt,
                                                         const double unit_random) noexcept {
  const double base = static_cast<double>(config.backoff_min.count());
  const double cap = static_cast<double>(config.backoff_max.count());
  double delay = std::min(cap, base * std::pow(2.0, static_cast<double>(std::min(attempt, 20U))));
  const double jitter = std::clamp(config.backoff_jitter, 0.0, 1.0);
  delay *= 1.0 + jitter * (2.0 * std::clamp(unit_random, 0.0, 1.0) - 1.0);
  delay = std::clamp(delay, 0.0, cap);
  return std::chrono::milliseconds(static_cast<std::int64_t>(delay));
}

void WebSocketClient::start() {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (supervisor_.joinable()) {
    return;
  }
  stop_requested_.store(false, std::memory_order_release);
  supervisor_ = std::thread([this] { supervise(); });
}

void WebSocketClient::stop() {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!supervisor_.joinable()) {
    return;
  }
  stop_requested_.store(true, std::memory_order_release);
  {
    const std::lock_guard<std::mutex> wake(wake_mutex_);
  }
  wake_cv_.notify_all();
  supervisor_.join();
  set_state(LinkState::Stopped, "stopped");
}

bool WebSocketClient::send_text(const std::string_view payload) {
  std::shared_ptr<ix::WebSocket> socket;
  if (state() == LinkState::Connected) {
    const std::lock_guard<std::mutex> lock(socket_mutex_);
    socket = socket_;
  }
  if (!socket || socket->getBufferedAmount() > config_.max_buffered_bytes) {
    messages_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const ix::WebSocketSendInfo info = socket->sendText(std::string(payload));
  if (!info.success) {
    messages_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  messages_sent_.fetch_add(1, std::memory_order_relaxed);
  bytes_sent_.fetch_add(payload.size(), std::memory_order_relaxed);
  return true;
}

LinkStats WebSocketClient::stats() const noexcept {
  LinkStats stats;
  stats.messages_sent = messages_sent_.load(std::memory_order_relaxed);
  stats.messages_dropped = messages_dropped_.load(std::memory_order_relaxed);
  stats.messages_received = messages_received_.load(std::memory_order_relaxed);
  stats.bytes_sent = bytes_sent_.load(std::memory_order_relaxed);
  stats.connects = connects_.load(std::memory_order_relaxed);
  stats.disconnects = disconnects_.load(std::memory_order_relaxed);
  stats.handler_errors = handler_errors_.load(std::memory_order_relaxed);
  return stats;
}

void WebSocketClient::set_state(const LinkState state, const std::string_view detail) {
  state_.store(state, std::memory_order_release);
  if (!on_state_) {
    return;
  }
  try {
    on_state_(state, detail);
  } catch (...) {
    handler_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

void WebSocketClient::deliver(const std::string_view text) noexcept {
  try {
    on_message_(text);
  } catch (...) {
    handler_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

void WebSocketClient::configure(ix::WebSocket& socket, const std::shared_ptr<Session>& session) {
  socket.setUrl(config_.url);
  socket.disableAutomaticReconnection();  // the supervisor owns reconnects, backoff and jitter
  socket.setPingInterval(config_.ping_interval_s);
  socket.setHandshakeTimeout(static_cast<int>(std::max<std::int64_t>(1, config_.connect_timeout.count() / 1000)));
  if (config_.compress) {
    socket.enablePerMessageDeflate();
  } else {
    socket.disablePerMessageDeflate();
  }
  if (!config_.bearer_token.empty()) {
    ix::WebSocketHttpHeaders headers;
    headers["Authorization"] = "Bearer " + config_.bearer_token;
    socket.setExtraHeaders(headers);
  }
  if (starts_with(config_.url, "wss://")) {
    ix::SocketTLSOptions tls;
    tls.caFile = "SYSTEM";
    socket.setTLSOptions(tls);
  }

  socket.setOnMessageCallback([this, session](const ix::WebSocketMessagePtr& message) {
    switch (message->type) {
      case ix::WebSocketMessageType::Open: {
        bool still_current = false;
        {
          const std::lock_guard<std::mutex> lock(session->mutex);
          if (!session->ended) {
            session->opened = true;
            session->opened_at = Clock::now();
            still_current = true;
          }
        }
        if (still_current && !stop_requested_.load(std::memory_order_acquire)) {
          connects_.fetch_add(1, std::memory_order_relaxed);
          set_state(LinkState::Connected, "connected");
        }
        session->cv.notify_all();
        break;
      }
      case ix::WebSocketMessageType::Message:
        if (!message->binary) {
          messages_received_.fetch_add(1, std::memory_order_relaxed);
          deliver(message->str);
        }
        break;
      case ix::WebSocketMessageType::Close:
        session->finish("closed (" + std::to_string(message->closeInfo.code) + ") " + message->closeInfo.reason);
        break;
      case ix::WebSocketMessageType::Error:
        session->finish("error: " + message->errorInfo.reason + " (http " + std::to_string(message->errorInfo.http_status) + ")");
        break;
      default:
        break;
    }
  });
}

WebSocketClient::SessionResult WebSocketClient::run_session(const std::shared_ptr<ix::WebSocket>& socket,
                                                            const std::shared_ptr<Session>& session) {
  const Clock::time_point began = Clock::now();
  std::optional<Clock::time_point> congested_since;
  std::string detail;

  while (true) {
    bool opened = false;
    {
      std::unique_lock<std::mutex> lock(session->mutex);
      session->cv.wait_for(lock, kPollInterval, [&] { return session->ended || stop_requested_.load(std::memory_order_acquire); });
      opened = session->opened;
      if (session->ended) {
        detail = session->detail;
        break;
      }
    }
    if (stop_requested_.load(std::memory_order_acquire)) {
      detail = "stopping";
      break;
    }
    const Clock::time_point now = Clock::now();
    if (!opened) {
      if (now - began > config_.connect_timeout) {
        detail = "connect timeout";
        break;
      }
      continue;
    }
    if (socket->getBufferedAmount() > config_.max_buffered_bytes) {
      if (!congested_since) {
        congested_since = now;
      } else if (now - *congested_since > config_.stall_timeout) {
        detail = "send buffer stalled";
        break;
      }
    } else {
      congested_since.reset();
    }
  }

  SessionResult result;
  result.detail = std::move(detail);
  const std::lock_guard<std::mutex> lock(session->mutex);
  result.opened = session->opened;
  result.open_for = session->opened ? Clock::now() - session->opened_at : Clock::duration::zero();
  return result;
}

void WebSocketClient::supervise() {
  unsigned failures = 0;
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  while (!stop_requested_.load(std::memory_order_acquire)) {
    auto session = std::make_shared<Session>();
    auto socket = std::make_shared<ix::WebSocket>();
    configure(*socket, session);
    {
      const std::lock_guard<std::mutex> lock(socket_mutex_);
      socket_ = socket;
    }
    set_state(LinkState::Connecting, config_.url);
    socket->start();

    const SessionResult result = run_session(socket, session);

    set_state(LinkState::Backoff, result.detail);
    {
      const std::lock_guard<std::mutex> lock(socket_mutex_);
      socket_.reset();
    }
    socket->stop();
    if (stop_requested_.load(std::memory_order_acquire)) {
      break;
    }

    if (result.opened) {
      disconnects_.fetch_add(1, std::memory_order_relaxed);
    }
    // A link that opened and then died within seconds is still a failure; without this rule a server
    // that accepts and immediately closes would be retried at the minimum delay forever.
    const bool stable =
        result.opened && std::chrono::duration_cast<std::chrono::milliseconds>(result.open_for) >= config_.stable_after;
    failures = stable ? 0U : failures + 1U;

    const std::chrono::milliseconds delay = backoff_delay(config_, failures - (failures > 0 ? 1U : 0U), unit(rng_));
    std::unique_lock<std::mutex> lock(wake_mutex_);
    wake_cv_.wait_for(lock, delay, [this] { return stop_requested_.load(std::memory_order_acquire); });
  }
}

}  // namespace sax
