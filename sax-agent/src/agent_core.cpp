#include "sax_agent/agent_core.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace sax {
namespace {

template <typename... Fns>
struct Overloaded : Fns... {
  using Fns::operator()...;
};
template <typename... Fns>
Overloaded(Fns...) -> Overloaded<Fns...>;

double seconds_between(const AgentCore::Clock::time_point from, const AgentCore::Clock::time_point to) {
  return std::chrono::duration<double>(to - from).count();
}

}  // namespace

AgentCore::AgentCore(AgentCoreConfig config, SendFn send, AgentCallbacks callbacks)
    : config_(std::move(config)),
      send_(std::move(send)),
      callbacks_(std::move(callbacks)),
      quantizer_(config_.voxel),
      encoder_(config_.robot_id),
      empty_voxels_(std::make_shared<const std::vector<VoxelIndex>>()),
      latest_voxels_(empty_voxels_) {
  if (!(config_.frequency_hz > 0.0)) {
    throw std::invalid_argument("frequency_hz must be positive");
  }
  if (config_.max_bandwidth_kbps > 0.0) {
    frame_budget_bytes_ =
        static_cast<std::size_t>(config_.max_bandwidth_kbps * 1000.0 / 8.0 / config_.frequency_hz);
  }
  if (!config_.latch_path.empty()) {
    std::error_code error;
    if (std::filesystem::exists(config_.latch_path, error)) {
      halted_.store(true, std::memory_order_release);
      estop_restored_ = true;
      log(LogLevel::Warn, "E-stop restored from " + config_.latch_path + "; staying halted until it is released");
    }
  }
}

void AgentCore::update_pose(const Pose& pose, const Clock::time_point now) {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  pose_ = pose;
  pose_at_ = now;
  have_pose_ = true;
}

void AgentCore::update_cloud(const std::span<const Vec3> world_points, const Clock::time_point now) {
  Vec3 origin;
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    origin = pose_.position_m;
  }
  auto voxels = std::make_shared<const std::vector<VoxelIndex>>(quantizer_.voxelize(world_points, origin));
  const std::lock_guard<std::mutex> lock(state_mutex_);
  latest_voxels_ = std::move(voxels);
  cloud_at_ = now;
  have_cloud_ = true;
}

void AgentCore::set_incident(const int reason) noexcept {
  incident_.store(std::clamp(reason, 0, 99), std::memory_order_release);
}

AgentCore::Snapshot AgentCore::snapshot(const Clock::time_point now) const {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  Snapshot snap;
  snap.have_pose = have_pose_;
  snap.pose = pose_;
  snap.pose_age_s = have_pose_ ? seconds_between(pose_at_, now) : 0.0;
  const bool cloud_fresh = have_cloud_ && seconds_between(cloud_at_, now) <= config_.cloud_timeout_s;
  snap.voxels = cloud_fresh ? latest_voxels_ : empty_voxels_;
  return snap;
}

void AgentCore::tick(const Clock::time_point now) {
  const Snapshot snap = snapshot(now);
  if (!snap.have_pose) {
    if (!warned_no_pose_.exchange(true)) {
      log(LogLevel::Warn, "waiting for the first odometry message before streaming");
    }
    return;
  }

  RobotState state = RobotState::Active;
  int reason = 0;
  if (halted_.load(std::memory_order_acquire)) {
    state = RobotState::Halted;
  } else if (const int incident = incident_.load(std::memory_order_acquire); incident > 0) {
    state = RobotState::Stuck;
    reason = incident;
  } else if (snap.pose_age_s > config_.odom_timeout_s) {
    state = RobotState::Stuck;
    reason = kReasonLocalizationLost;
  }
  send_frame(state, reason, snap);
}

void AgentCore::send_frame(const RobotState state, const int reason, const Snapshot& snap) {
  const std::lock_guard<std::mutex> lock(send_mutex_);
  EncodeStats encoded;
  const std::string line =
      encoder_.encode(Frame{state, reason, snap.pose, std::span<const VoxelIndex>(*snap.voxels)}, frame_budget_bytes_, &encoded);
  frames_built_.fetch_add(1, std::memory_order_relaxed);
  last_frame_bytes_.store(line.size(), std::memory_order_relaxed);
  last_voxel_count_.store(encoded.voxels_encoded, std::memory_order_relaxed);
  last_voxels_dropped_.store(encoded.voxels_dropped, std::memory_order_relaxed);

  bool delivered = false;
  try {
    delivered = send_ && send_(line);
  } catch (const std::exception& error) {
    log(LogLevel::Error, std::string("send failed: ") + error.what());
  } catch (...) {
    log(LogLevel::Error, "send failed with an unknown error");
  }
  if (delivered) {
    frames_sent_.fetch_add(1, std::memory_order_relaxed);
    bytes_sent_.fetch_add(line.size(), std::memory_order_relaxed);
  } else {
    frames_dropped_.fetch_add(1, std::memory_order_relaxed);
  }
}

void AgentCore::send_halted_frame() {
  send_frame(RobotState::Halted, 0, snapshot(Clock::now()));
}

void AgentCore::handle_command(const std::string_view line) noexcept {
  try {
    commands_received_.fetch_add(1, std::memory_order_relaxed);
    const ParseResult parsed = parse_command(line, config_.robot_id);
    if (!parsed.command) {
      commands_rejected_.fetch_add(1, std::memory_order_relaxed);
      log(LogLevel::Warn, "rejected command: " + parsed.error);
      return;
    }
    std::visit(Overloaded{
                   [this](const EStopCommand& command) {
                     if (!command.operator_valid) {
                       log(LogLevel::Warn, "E-stop carried an invalid operator id; stopping anyway");
                     }
                     engage_estop(command.operator_id);
                   },
                   [this](const ReleaseCommand& command) { release_estop(command.operator_id); },
                   [this](const TrajectoryCommand& command) { forward_trajectory(command.waypoints); },
               },
               *parsed.command);
  } catch (const std::exception& error) {
    commands_rejected_.fetch_add(1, std::memory_order_relaxed);
    log(LogLevel::Error, std::string("command handling failed: ") + error.what());
  } catch (...) {
    commands_rejected_.fetch_add(1, std::memory_order_relaxed);
    log(LogLevel::Error, "command handling failed with an unknown error");
  }
}

void AgentCore::engage_estop(const std::string& operator_id) {
  estops_received_.fetch_add(1, std::memory_order_relaxed);
  // Latch first, so the 10 Hz loop can never report anything but H from this point on.
  const bool first = !halted_.exchange(true, std::memory_order_acq_rel);
  if (first) {
    log(LogLevel::Warn, "E-STOP engaged by " + operator_id);
  }
  // The bridge re-sends SXE until it sees H, so the stop is re-asserted on every receipt.
  notify_estop(true, operator_id);
  if (first) {
    persist_latch(operator_id);
  }
  send_halted_frame();
}

bool AgentCore::release_estop(const std::string_view operator_id) {
  if (!halted_.exchange(false, std::memory_order_acq_rel)) {
    return false;
  }
  log(LogLevel::Warn, "E-stop released by " + std::string(operator_id));
  clear_latch();  // before notifying: a crash in between leaves the robot stopped, never moving
  notify_estop(false, operator_id);
  return true;
}

void AgentCore::forward_trajectory(const std::vector<Waypoint>& waypoints) {
  if (halted_.load(std::memory_order_acquire)) {
    trajectories_rejected_.fetch_add(1, std::memory_order_relaxed);
    log(LogLevel::Warn, "trajectory rejected while the E-stop is latched");
    return;
  }
  if (!callbacks_.on_trajectory) {
    trajectories_rejected_.fetch_add(1, std::memory_order_relaxed);
    log(LogLevel::Warn, "trajectory rejected: no consumer is registered");
    return;
  }
  callbacks_.on_trajectory(waypoints);
  trajectories_forwarded_.fetch_add(1, std::memory_order_relaxed);
}

void AgentCore::persist_latch(const std::string_view operator_id) const noexcept {
  if (config_.latch_path.empty()) {
    return;
  }
  try {
    std::ofstream file(config_.latch_path, std::ios::trunc);
    file << operator_id << '\n';
    file.flush();
    if (!file) {
      log(LogLevel::Error, "could not write the E-stop latch file " + config_.latch_path);
    }
  } catch (...) {
    log(LogLevel::Error, "could not write the E-stop latch file " + config_.latch_path);
  }
}

void AgentCore::clear_latch() const noexcept {
  if (config_.latch_path.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::remove(config_.latch_path, error);
  if (error) {
    log(LogLevel::Error, "could not remove the E-stop latch file " + config_.latch_path + ": " + error.message());
  }
}

void AgentCore::notify_estop(const bool engaged, const std::string_view operator_id) const noexcept {
  if (!callbacks_.on_estop) {
    return;
  }
  try {
    callbacks_.on_estop(engaged, operator_id);
  } catch (const std::exception& error) {
    log(LogLevel::Error, std::string("E-stop callback failed: ") + error.what());
  } catch (...) {
    log(LogLevel::Error, "E-stop callback failed with an unknown error");
  }
}

void AgentCore::log(const LogLevel level, const std::string_view message) const noexcept {
  if (!callbacks_.on_log) {
    return;
  }
  try {
    callbacks_.on_log(level, message);
  } catch (...) {
    // Logging must never take the safety path down.
  }
}

CoreStats AgentCore::stats() const noexcept {
  CoreStats stats;
  stats.frames_built = frames_built_.load(std::memory_order_relaxed);
  stats.frames_sent = frames_sent_.load(std::memory_order_relaxed);
  stats.frames_dropped = frames_dropped_.load(std::memory_order_relaxed);
  stats.bytes_sent = bytes_sent_.load(std::memory_order_relaxed);
  stats.commands_received = commands_received_.load(std::memory_order_relaxed);
  stats.commands_rejected = commands_rejected_.load(std::memory_order_relaxed);
  stats.estops_received = estops_received_.load(std::memory_order_relaxed);
  stats.trajectories_forwarded = trajectories_forwarded_.load(std::memory_order_relaxed);
  stats.trajectories_rejected = trajectories_rejected_.load(std::memory_order_relaxed);
  stats.last_frame_bytes = last_frame_bytes_.load(std::memory_order_relaxed);
  stats.last_voxel_count = last_voxel_count_.load(std::memory_order_relaxed);
  stats.last_voxels_dropped = last_voxels_dropped_.load(std::memory_order_relaxed);
  stats.estopped = halted_.load(std::memory_order_acquire);
  return stats;
}

}  // namespace sax
