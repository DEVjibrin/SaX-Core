#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "check.hpp"
#include "sax_agent/agent_core.hpp"

using namespace sax;
using Clock = AgentCore::Clock;
using std::chrono::milliseconds;

namespace {

struct Harness {
  std::mutex mutex;
  std::vector<std::string> lines;
  std::vector<std::pair<bool, std::string>> estops;
  std::vector<std::vector<Waypoint>> trajectories;
  bool link_up{true};
  bool estop_callback_throws{false};
  std::unique_ptr<AgentCore> core;

  explicit Harness(AgentCoreConfig config = {}) {
    AgentCallbacks callbacks;
    callbacks.on_estop = [this](const bool engaged, const std::string_view operator_id) {
      const std::lock_guard<std::mutex> lock(mutex);
      estops.emplace_back(engaged, std::string(operator_id));
      if (estop_callback_throws) {
        throw std::runtime_error("relay board offline");
      }
    };
    callbacks.on_trajectory = [this](const std::vector<Waypoint>& waypoints) {
      const std::lock_guard<std::mutex> lock(mutex);
      trajectories.push_back(waypoints);
    };
    core = std::make_unique<AgentCore>(
        std::move(config),
        [this](const std::string_view line) {
          const std::lock_guard<std::mutex> lock(mutex);
          lines.emplace_back(line);
          return link_up;
        },
        callbacks);
  }
};

const std::vector<Vec3> kCluster = {{1.01, 0.01, 0.01}, {1.02, 0.02, 0.02}, {1.03, 0.03, 0.03}};

void feed(Harness& h, const Clock::time_point at) {
  h.core->update_pose(Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}, at);
  h.core->update_cloud(kCluster, at);
}

bool has_state(const std::string& line, const std::string& state) {
  return line.find("|" + state + "|") != std::string::npos;
}

void test_waits_for_the_first_odometry() {
  Harness h;
  h.core->tick();
  CHECK(h.lines.empty());
}

void test_streams_active_frames_with_voxels() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.core->tick(t0 + milliseconds(10));
  h.core->tick(t0 + milliseconds(110));
  CHECK(h.lines.size() == 2);
  CHECK(h.lines[0].starts_with("SX1|robot_01|0|A|0,0,0,0,0,0,1|5.0.0"));
  CHECK(h.lines[1].starts_with("SX1|robot_01|1|A|"));
  CHECK(h.core->stats().frames_sent == 2 && h.core->stats().last_voxel_count == 1);
}

void test_stale_cloud_is_reported_as_empty() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.core->update_pose(Pose{}, t0 + milliseconds(550));
  h.core->tick(t0 + milliseconds(600));
  CHECK(h.lines.size() == 1 && h.lines[0].ends_with("|"));
}

void test_stale_odometry_reports_localization_lost() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.core->tick(t0 + milliseconds(1500));
  CHECK(has_state(h.lines.back(), "S3"));
}

void test_external_incident_sets_the_stuck_reason() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.core->set_incident(2);
  h.core->tick(t0 + milliseconds(50));
  CHECK(has_state(h.lines.back(), "S2"));
  h.core->set_incident(0);
  h.core->tick(t0 + milliseconds(100));
  CHECK(has_state(h.lines.back(), "A"));
}

void test_estop_acts_immediately_and_latches() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.core->tick(t0 + milliseconds(10));
  const std::size_t before = h.lines.size();

  h.core->handle_command("SXE|robot_01|ann");
  CHECK(h.estops.size() == 1 && h.estops[0].first && h.estops[0].second == "ann");
  CHECK(h.core->estopped());
  CHECK(h.lines.size() == before + 1 && has_state(h.lines.back(), "H"));

  h.core->tick(t0 + milliseconds(60));
  CHECK(has_state(h.lines.back(), "H"));

  h.core->handle_command("SXE|robot_01|ann");
  CHECK(h.estops.size() == 2);

  h.core->handle_command("SXP|robot_01|0,0;100,0");
  CHECK(h.trajectories.empty() && h.core->stats().trajectories_rejected == 1);

  h.core->handle_command("SXR|robot_01|ann");
  CHECK(!h.core->estopped() && h.estops.back() == std::make_pair(false, std::string("ann")));
  h.core->tick(t0 + milliseconds(110));
  CHECK(has_state(h.lines.back(), "A"));
  CHECK(!h.core->release_estop("ann"));
}

void test_malformed_estop_still_stops_and_foreign_estop_does_not() {
  Harness h;
  feed(h, Clock::now());
  h.core->handle_command("SXE|robot_02|ann");
  CHECK(!h.core->estopped() && h.estops.empty());
  h.core->handle_command("SXE|robot_01|bad op!");
  CHECK(h.core->estopped() && h.estops.back().second == "unknown");
}

void test_trajectory_is_forwarded_when_not_halted() {
  Harness h;
  feed(h, Clock::now());
  h.core->handle_command("SXP|robot_01|0,0;100,50");
  CHECK(h.trajectories.size() == 1 && h.trajectories[0].size() == 2);
  CHECK(h.trajectories[0][1].x_m == 1.0 && h.core->stats().trajectories_forwarded == 1);
}

void test_garbage_commands_never_throw() {
  Harness h;
  feed(h, Clock::now());
  const std::string nul("SXE\0|robot_01|ann", 17);
  for (const std::string& line : {std::string(), std::string("|||"), std::string("SXP|robot_01|1,2"), std::string(5000, 'q'), nul}) {
    h.core->handle_command(line);
  }
  CHECK(h.core->stats().commands_received == 5 && h.core->stats().commands_rejected >= 4);
  CHECK(!h.core->estopped());
}

void test_estop_survives_a_failing_callback() {
  Harness h;
  feed(h, Clock::now());
  h.estop_callback_throws = true;
  h.core->handle_command("SXE|robot_01|ann");
  CHECK(h.core->estopped());
  CHECK(!h.lines.empty() && has_state(h.lines.back(), "H"));
}

void test_dropped_frames_are_counted() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  h.link_up = false;
  h.core->tick(t0);
  h.core->tick(t0);
  CHECK(h.core->stats().frames_dropped == 2 && h.core->stats().frames_sent == 0);
  h.link_up = true;
  h.core->tick(t0);
  CHECK(h.core->stats().frames_sent == 1);
}

void test_bandwidth_budget_caps_the_frame() {
  AgentCoreConfig config;
  config.max_bandwidth_kbps = 8.0;  // 100 bytes per frame at 10 Hz
  config.voxel.min_points_per_voxel = 1;
  Harness h(config);
  std::vector<Vec3> points;
  for (int n = 0; n < 200; ++n) {
    points.push_back({1.0 + 0.2 * (n % 20), 0.2 * (n / 20), 0.1});
  }
  const auto t0 = Clock::now();
  h.core->update_pose(Pose{}, t0);
  h.core->update_cloud(points, t0);
  h.core->tick(t0);
  CHECK(h.lines[0].size() <= 100);
  CHECK(h.core->stats().last_voxels_dropped > 0 && h.core->stats().last_voxel_count > 0);
}

void test_estop_latch_survives_a_restart() {
  const std::string path = (std::filesystem::temp_directory_path() / "sax_agent_test_latch.estop").string();
  std::filesystem::remove(path);
  AgentCoreConfig config;
  config.latch_path = path;
  {
    Harness h(config);
    feed(h, Clock::now());
    CHECK(!h.core->estopped() && !h.core->estop_restored() && !std::filesystem::exists(path));
    h.core->handle_command("SXE|robot_01|ann");
    CHECK(std::filesystem::exists(path));
  }
  {
    Harness h(config);  // the process was restarted
    CHECK(h.core->estopped() && h.core->estop_restored());
    const auto t0 = Clock::now();
    feed(h, t0);
    h.core->tick(t0 + milliseconds(10));
    CHECK(has_state(h.lines.back(), "H"));
    CHECK(h.core->release_estop("ann") && !std::filesystem::exists(path));
  }
  Harness h(config);
  CHECK(!h.core->estopped());
}

void test_an_unwritable_latch_does_not_prevent_the_stop() {
  AgentCoreConfig config;
  config.latch_path = "/nonexistent-directory/sax_agent.estop";
  Harness h(config);
  feed(h, Clock::now());
  h.core->handle_command("SXE|robot_01|ann");
  CHECK(h.core->estopped() && h.estops.size() == 1);
  CHECK(!h.lines.empty() && has_state(h.lines.back(), "H"));
}

std::uint64_t sequence_of(const std::string& line) {
  const std::string prefix = "SX1|robot_01|";
  return std::stoull(line.substr(prefix.size(), line.find('|', prefix.size()) - prefix.size()));
}

void test_concurrent_ticks_and_commands_keep_sequence_order() {
  Harness h;
  const auto t0 = Clock::now();
  feed(h, t0);
  std::thread ticker([&] {
    for (int n = 0; n < 400; ++n) {
      h.core->tick(t0 + milliseconds(n));
    }
  });
  std::thread commander([&] {
    for (int n = 0; n < 100; ++n) {
      h.core->handle_command("SXE|robot_01|ann");
      h.core->handle_command("SXR|robot_01|ann");
    }
  });
  ticker.join();
  commander.join();
  CHECK(h.lines.size() >= 400);
  for (std::size_t n = 1; n < h.lines.size(); ++n) {
    CHECK(sequence_of(h.lines[n]) == sequence_of(h.lines[n - 1]) + 1);
  }
}

}  // namespace

int main() {
  RUN(test_waits_for_the_first_odometry);
  RUN(test_streams_active_frames_with_voxels);
  RUN(test_stale_cloud_is_reported_as_empty);
  RUN(test_stale_odometry_reports_localization_lost);
  RUN(test_external_incident_sets_the_stuck_reason);
  RUN(test_estop_acts_immediately_and_latches);
  RUN(test_malformed_estop_still_stops_and_foreign_estop_does_not);
  RUN(test_trajectory_is_forwarded_when_not_halted);
  RUN(test_garbage_commands_never_throw);
  RUN(test_estop_survives_a_failing_callback);
  RUN(test_dropped_frames_are_counted);
  RUN(test_bandwidth_budget_caps_the_frame);
  RUN(test_estop_latch_survives_a_restart);
  RUN(test_an_unwritable_latch_does_not_prevent_the_stop);
  RUN(test_concurrent_ticks_and_commands_keep_sequence_order);
  return 0;
}
