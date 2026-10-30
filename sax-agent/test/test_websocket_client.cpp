#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ixwebsocket/IXWebSocket.h>  // the in-process fake under test/fakes

#include "check.hpp"
#include "sax_agent/websocket_client.hpp"

using namespace std::chrono_literals;
using sax::LinkState;
using sax::WebSocketClient;
using sax::WebSocketClientConfig;

namespace {

ix::FakeNetwork& net = ix::FakeNetwork::instance();

template <typename Predicate>
bool wait_until(Predicate predicate, const std::chrono::milliseconds timeout = 3s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

WebSocketClientConfig fast_config() {
  WebSocketClientConfig config;
  config.url = "ws://bridge.test/ws/agent/robot_01";
  config.bearer_token = "secret";
  config.backoff_min = 20ms;
  config.backoff_max = 200ms;
  config.backoff_jitter = 0.0;
  config.stable_after = 50ms;
  config.stall_timeout = 60s;
  config.connect_timeout = 2s;
  return config;
}

struct Recorder {
  std::mutex mutex;
  std::vector<std::string> messages;
  std::vector<LinkState> states;
  bool throw_on_message{false};

  WebSocketClient make(WebSocketClientConfig config) {
    return WebSocketClient(
        std::move(config),
        [this](const std::string_view text) {
          const std::lock_guard<std::mutex> lock(mutex);
          messages.emplace_back(text);
          if (throw_on_message) {
            throw std::runtime_error("handler bug");
          }
        },
        [this](const LinkState state, std::string_view) {
          const std::lock_guard<std::mutex> lock(mutex);
          states.push_back(state);
        });
  }

  std::size_t message_count() {
    const std::lock_guard<std::mutex> lock(mutex);
    return messages.size();
  }
  bool saw(const LinkState state) {
    const std::lock_guard<std::mutex> lock(mutex);
    for (const LinkState s : states) {
      if (s == state) {
        return true;
      }
    }
    return false;
  }
};

void test_connects_sends_and_receives() {
  net.reset();
  Recorder rec;
  WebSocketClient client = rec.make(fast_config());
  CHECK(!client.send_text("too early"));
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  CHECK(net.url() == "ws://bridge.test/ws/agent/robot_01");
  CHECK(net.header("Authorization") == "Bearer secret");

  CHECK(client.send_text("SX1|robot_01|0|A|0,0,0,0,0,0,1|"));
  const std::vector<std::string> sent = net.sent_snapshot();
  CHECK(sent.size() == 1 && sent[0] == "SX1|robot_01|0|A|0,0,0,0,0,0,1|");
  net.inject("SXE|robot_01|ann");
  CHECK(wait_until([&] { return rec.message_count() == 1; }));
  CHECK(rec.messages[0] == "SXE|robot_01|ann");

  const sax::LinkStats stats = client.stats();
  CHECK(stats.messages_sent == 1 && stats.messages_dropped == 1 && stats.messages_received == 1 && stats.connects == 1);
}

void test_congestion_drops_frames_instead_of_queueing() {
  net.reset();
  Recorder rec;
  WebSocketClient client = rec.make(fast_config());
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.set_buffered(1024 * 1024);
  CHECK(!client.send_text("stale"));
  net.set_buffered(0);
  CHECK(client.send_text("fresh"));
  CHECK(client.stats().messages_dropped == 1 && net.sent_snapshot().size() == 1);
}

void test_reconnects_after_the_server_drops_the_link() {
  net.reset();
  Recorder rec;
  WebSocketClient client = rec.make(fast_config());
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.drop();
  CHECK(wait_until([&] { return rec.saw(LinkState::Backoff); }));
  CHECK(wait_until([&] { return client.stats().connects == 2 && client.state() == LinkState::Connected; }));
  CHECK(client.send_text("after reconnect"));
  CHECK(client.stats().disconnects == 1);
}

void test_backs_off_while_the_server_is_unreachable() {
  net.reset();
  Recorder rec;
  WebSocketClient client = rec.make(fast_config());
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.set_refuse(true);
  net.drop();
  const int before = net.attempts();
  std::this_thread::sleep_for(700ms);
  const int attempts = net.attempts() - before;
  CHECK(attempts >= 3);   // it keeps trying
  CHECK(attempts <= 12);  // but 20/40/80/160/200 ms steps, not a tight loop (that would be 35+)
  net.set_refuse(false);
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
}

void test_short_lived_sessions_keep_backing_off() {
  net.reset();
  net.set_close_immediately(true);
  Recorder rec;
  WebSocketClientConfig config = fast_config();
  config.stable_after = 5s;
  WebSocketClient client = rec.make(config);
  client.start();
  std::this_thread::sleep_for(700ms);
  CHECK(net.attempts() >= 3 && net.attempts() <= 12);
  client.stop();
}

void test_a_stalled_send_buffer_forces_a_reconnect() {
  net.reset();
  Recorder rec;
  WebSocketClientConfig config = fast_config();
  config.stall_timeout = 300ms;
  config.max_buffered_bytes = 1000;
  WebSocketClient client = rec.make(config);
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.set_buffered(10'000);
  CHECK(wait_until([&] { return net.attempts() >= 2; }, 3s));
  net.set_buffered(0);
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
}

void test_stop_is_prompt_idempotent_and_restartable() {
  net.reset();
  Recorder rec;
  WebSocketClient client = rec.make(fast_config());
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  const auto began = std::chrono::steady_clock::now();
  client.stop();
  client.stop();
  CHECK(std::chrono::steady_clock::now() - began < 1s);
  CHECK(client.state() == LinkState::Stopped && !client.send_text("after stop"));

  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.set_refuse(true);
  net.drop();
  CHECK(wait_until([&] { return client.state() == LinkState::Backoff; }));
  const auto stopping = std::chrono::steady_clock::now();
  client.stop();
  CHECK(std::chrono::steady_clock::now() - stopping < 1s);
}

void test_a_throwing_message_handler_does_not_kill_the_link() {
  net.reset();
  Recorder rec;
  rec.throw_on_message = true;
  WebSocketClient client = rec.make(fast_config());
  client.start();
  CHECK(wait_until([&] { return client.state() == LinkState::Connected; }));
  net.inject("one");
  net.inject("two");
  CHECK(wait_until([&] { return rec.message_count() == 2; }));
  CHECK(client.state() == LinkState::Connected && client.stats().handler_errors == 2);
}

void test_backoff_delay_math() {
  WebSocketClientConfig config;
  config.backoff_min = 500ms;
  config.backoff_max = 30'000ms;
  config.backoff_jitter = 0.0;
  const long long expected[] = {500, 1000, 2000, 4000, 8000, 16000, 30000, 30000};
  for (unsigned attempt = 0; attempt < 8; ++attempt) {
    CHECK(WebSocketClient::backoff_delay(config, attempt, 0.5).count() == expected[attempt]);
  }
  CHECK(WebSocketClient::backoff_delay(config, 1000, 0.5).count() == 30000);
  config.backoff_jitter = 0.2;
  CHECK(WebSocketClient::backoff_delay(config, 0, 0.0).count() == 400);
  CHECK(WebSocketClient::backoff_delay(config, 0, 1.0).count() == 600);
  CHECK(WebSocketClient::backoff_delay(config, 9, 1.0).count() == 30000);

  bool threw = false;
  try {
    WebSocketClientConfig bad;
    WebSocketClient client(bad, [](std::string_view) {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

}  // namespace

int main() {
  RUN(test_backoff_delay_math);
  RUN(test_connects_sends_and_receives);
  RUN(test_congestion_drops_frames_instead_of_queueing);
  RUN(test_reconnects_after_the_server_drops_the_link);
  RUN(test_backs_off_while_the_server_is_unreachable);
  RUN(test_short_lived_sessions_keep_backing_off);
  RUN(test_a_stalled_send_buffer_forces_a_reconnect);
  RUN(test_stop_is_prompt_idempotent_and_restartable);
  RUN(test_a_throwing_message_handler_does_not_kill_the_link);
  return 0;
}
