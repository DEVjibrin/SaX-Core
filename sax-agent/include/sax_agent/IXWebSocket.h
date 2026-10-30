// In-process stand-in for the subset of ixwebsocket that WebSocketClient uses. It lets the
// reconnect, backoff and congestion logic be tested without a network or the real library.
// The real ix::WebSocket runs its own thread and reports Open, Message, Close and Error events
// through one callback; so does this fake.
#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ix {

enum class WebSocketMessageType { Message, Open, Close, Error, Ping, Pong, Fragment };

struct WebSocketErrorInfo {
  std::uint32_t retries{0};
  double wait_time{0};
  int http_status{0};
  std::string reason;
  bool decompressionError{false};
};

struct WebSocketCloseInfo {
  std::uint16_t code{1000};
  std::string reason;
  bool remote{false};
};

struct WebSocketOpenInfo {
  std::string uri;
};

struct WebSocketMessage {
  WebSocketMessageType type{WebSocketMessageType::Message};
  std::string str;
  std::size_t wireSize{0};
  WebSocketErrorInfo errorInfo;
  WebSocketOpenInfo openInfo;
  WebSocketCloseInfo closeInfo;
  bool binary{false};
};

using WebSocketMessagePtr = std::unique_ptr<WebSocketMessage>;
using WebSocketHttpHeaders = std::map<std::string, std::string>;

struct SocketTLSOptions {
  std::string caFile;
};

struct WebSocketSendInfo {
  bool success{false};
  bool compressionError{false};
  std::size_t payloadSize{0};
  std::size_t wireSize{0};
};

class WebSocket;

/// Process-wide controls for the fake network. Tests call reset() first.
struct FakeNetwork {
  std::mutex mutex;
  std::condition_variable cv;
  bool refuse{false};           // new connections fail with an Error event
  bool close_immediately{false};  // new connections open and are closed by the server at once
  bool drop_requested{false};   // the server closes the active connection
  std::size_t buffered{0};      // value reported by getBufferedAmount()
  int connect_attempts{0};
  std::string last_url;
  WebSocketHttpHeaders last_headers;
  std::vector<std::string> sent;
  WebSocket* active{nullptr};

  static FakeNetwork& instance() {
    static FakeNetwork network;
    return network;
  }

  void reset() {
    const std::lock_guard<std::mutex> lock(mutex);
    refuse = false;
    close_immediately = false;
    drop_requested = false;
    buffered = 0;
    connect_attempts = 0;
    last_url.clear();
    last_headers.clear();
    sent.clear();
  }

  void drop() {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      drop_requested = true;
    }
    cv.notify_all();
  }

  void set_refuse(bool value) {
    const std::lock_guard<std::mutex> lock(mutex);
    refuse = value;
  }

  void set_close_immediately(bool value) {
    const std::lock_guard<std::mutex> lock(mutex);
    close_immediately = value;
  }

  void set_buffered(std::size_t value) {
    const std::lock_guard<std::mutex> lock(mutex);
    buffered = value;
  }

  std::vector<std::string> sent_snapshot() {
    const std::lock_guard<std::mutex> lock(mutex);
    return sent;
  }

  std::string url() {
    const std::lock_guard<std::mutex> lock(mutex);
    return last_url;
  }

  std::string header(const std::string& name) {
    const std::lock_guard<std::mutex> lock(mutex);
    const auto found = last_headers.find(name);
    return found == last_headers.end() ? std::string() : found->second;
  }

  inline void inject(const std::string& text);
  int attempts() {
    const std::lock_guard<std::mutex> lock(mutex);
    return connect_attempts;
  }
};

class WebSocket {
 public:
  using Callback = std::function<void(const WebSocketMessagePtr&)>;

  WebSocket() = default;
  WebSocket(const WebSocket&) = delete;
  WebSocket& operator=(const WebSocket&) = delete;
  ~WebSocket() { stop(); }

  void setUrl(const std::string& url) { url_ = url; }
  void disableAutomaticReconnection() {}
  void setPingInterval(int) {}
  void setHandshakeTimeout(int) {}
  void enablePerMessageDeflate() {}
  void disablePerMessageDeflate() {}
  void setExtraHeaders(const WebSocketHttpHeaders& headers) { headers_ = headers; }
  void setTLSOptions(const SocketTLSOptions&) {}
  void setOnMessageCallback(Callback callback) { callback_ = std::move(callback); }

  void start() {
    if (thread_.joinable()) {
      return;
    }
    thread_ = std::thread([this] { run(); });
  }

  void stop(std::uint16_t = 1000, const std::string& = "") {
    {
      auto& network = FakeNetwork::instance();
      const std::lock_guard<std::mutex> lock(network.mutex);
      stopping_ = true;
    }
    FakeNetwork::instance().cv.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  WebSocketSendInfo sendText(const std::string& text) {
    auto& network = FakeNetwork::instance();
    const std::lock_guard<std::mutex> lock(network.mutex);
    if (!open_) {
      return {};
    }
    network.sent.push_back(text);
    return WebSocketSendInfo{true, false, text.size(), text.size()};
  }

  std::size_t getBufferedAmount() const {
    auto& network = FakeNetwork::instance();
    const std::lock_guard<std::mutex> lock(network.mutex);
    return network.buffered;
  }

  void deliver(const std::string& text) {
    WebSocketMessagePtr message = std::make_unique<WebSocketMessage>();
    message->type = WebSocketMessageType::Message;
    message->str = text;
    if (callback_) {
      callback_(message);
    }
  }

 private:
  void emit(WebSocketMessageType type, const std::string& reason = "", std::uint16_t code = 1000) {
    WebSocketMessagePtr message = std::make_unique<WebSocketMessage>();
    message->type = type;
    message->closeInfo.code = code;
    message->closeInfo.reason = reason;
    message->errorInfo.reason = reason;
    if (callback_) {
      callback_(message);
    }
  }

  void run() {
    auto& network = FakeNetwork::instance();
    bool refuse = false;
    bool close_now = false;
    {
      const std::lock_guard<std::mutex> lock(network.mutex);
      ++network.connect_attempts;
      network.last_url = url_;
      network.last_headers = headers_;
      refuse = network.refuse;
      close_now = network.close_immediately;
    }
    if (refuse) {
      emit(WebSocketMessageType::Error, "connection refused");
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(network.mutex);
      open_ = true;
      network.active = this;
    }
    emit(WebSocketMessageType::Open);

    bool server_closed = close_now;
    {
      std::unique_lock<std::mutex> lock(network.mutex);
      if (!close_now) {
        network.cv.wait(lock, [&] { return stopping_ || network.drop_requested; });
        server_closed = network.drop_requested && !stopping_;
      }
      if (network.drop_requested) {
        network.drop_requested = false;
      }
      open_ = false;
      if (network.active == this) {
        network.active = nullptr;
      }
    }
    emit(WebSocketMessageType::Close, server_closed ? "server closed the connection" : "client closed", server_closed ? 1006 : 1000);
  }

  std::string url_;
  WebSocketHttpHeaders headers_;
  Callback callback_;
  std::thread thread_;
  bool open_{false};
  bool stopping_{false};

  friend struct FakeNetwork;
};

inline void FakeNetwork::inject(const std::string& text) {
  WebSocket* target = nullptr;
  {
    const std::lock_guard<std::mutex> lock(mutex);
    target = active;
  }
  if (target != nullptr) {
    target->deliver(text);
  }
}

}  // namespace ix
