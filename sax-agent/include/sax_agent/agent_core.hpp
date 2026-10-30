// Transport-agnostic agent logic: state machine, voxelization, SX1 framing, and command handling.
//
// Threading: update_pose / update_cloud / set_incident / tick are called from the sensor and timer
// thread(s); handle_command is called from the WebSocket thread. E-stop handling therefore runs on
// the WebSocket thread, immediately, without waiting for the next 10 Hz tick.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sax_agent/command_parser.hpp"
#include "sax_agent/sx1_encoder.hpp"

namespace sax {

enum class LogLevel { Debug, Info, Warn, Error };

struct AgentCoreConfig {
  std::string robot_id{"robot_01"};
  VoxelizerConfig voxel{};
  double frequency_hz{10.0};
  double max_bandwidth_kbps{900.0};  // SX1 payload budget before WebSocket compression
  double cloud_timeout_s{0.5};       // older clouds are reported as an empty point cloud
  double odom_timeout_s{1.0};        // older odometry is reported as stuck/localization lost
  std::string latch_path{};          // when set, a latched E-stop survives restarts (the file exists while latched)
};

struct AgentCallbacks {
  /// Engage (true) or release (false) the robot's safety stop. Invoked synchronously from the thread
  /// that received the command, so keep it short and thread-safe.
  std::function<void(bool engaged, std::string_view operator_id)> on_estop;
  std::function<void(const std::vector<Waypoint>&)> on_trajectory;
  std::function<void(LogLevel, std::string_view)> on_log;
};

struct CoreStats {
  std::uint64_t frames_built{0};
  std::uint64_t frames_sent{0};
  std::uint64_t frames_dropped{0};
  std::uint64_t bytes_sent{0};
  std::uint64_t commands_received{0};
  std::uint64_t commands_rejected{0};
  std::uint64_t estops_received{0};
  std::uint64_t trajectories_forwarded{0};
  std::uint64_t trajectories_rejected{0};
  std::size_t last_frame_bytes{0};
  std::size_t last_voxel_count{0};
  std::size_t last_voxels_dropped{0};
  bool estopped{false};
};

class AgentCore {
 public:
  using Clock = std::chrono::steady_clock;
  /// Returns true when the frame was handed to the network, false when it was dropped.
  using SendFn = std::function<bool(std::string_view)>;

  AgentCore(AgentCoreConfig config, SendFn send, AgentCallbacks callbacks = {});

  void update_pose(const Pose& pose, Clock::time_point now = Clock::now());
  /// Voxelize world-frame points on the calling thread and publish them as the latest cloud.
  void update_cloud(std::span<const Vec3> world_points, Clock::time_point now = Clock::now());
  /// Report an external incident (1 path blocked, 2 wheel slip, 3 localization lost); 0 clears it.
  void set_incident(int reason) noexcept;

  /// Build and send one SX1 frame. Does nothing until the first odometry message has arrived.
  void tick(Clock::time_point now = Clock::now());

  /// Handle one inbound line from the bridge. Never throws.
  void handle_command(std::string_view line) noexcept;

  /// Release a latched E-stop locally (for example from a ROS service). Returns false if none is latched.
  bool release_estop(std::string_view operator_id);

  [[nodiscard]] bool estopped() const noexcept { return halted_.load(std::memory_order_acquire); }
  /// True when the E-stop was latched at construction because a latch file from a previous run existed.
  [[nodiscard]] bool estop_restored() const noexcept { return estop_restored_; }
  [[nodiscard]] CoreStats stats() const noexcept;
  [[nodiscard]] const AgentCoreConfig& config() const noexcept { return config_; }

 private:
  using VoxelList = std::shared_ptr<const std::vector<VoxelIndex>>;

  struct Snapshot {
    bool have_pose{false};
    Pose pose{};
    double pose_age_s{0.0};
    VoxelList voxels;
  };

  [[nodiscard]] Snapshot snapshot(Clock::time_point now) const;
  void send_frame(RobotState state, int reason, const Snapshot& snap);
  void send_halted_frame();
  void engage_estop(const std::string& operator_id);
  void forward_trajectory(const std::vector<Waypoint>& waypoints);
  void notify_estop(bool engaged, std::string_view operator_id) const noexcept;
  void persist_latch(std::string_view operator_id) const noexcept;
  void clear_latch() const noexcept;
  void log(LogLevel level, std::string_view message) const noexcept;

  AgentCoreConfig config_;
  SendFn send_;
  AgentCallbacks callbacks_;
  VoxelQuantizer quantizer_;
  Sx1Encoder encoder_;
  std::size_t frame_budget_bytes_{0};
  VoxelList empty_voxels_;

  mutable std::mutex state_mutex_;
  bool have_pose_{false};
  Pose pose_{};
  Clock::time_point pose_at_{};
  bool have_cloud_{false};
  Clock::time_point cloud_at_{};
  VoxelList latest_voxels_;

  std::mutex send_mutex_;  // keeps sequence numbers in send order across threads
  bool estop_restored_{false};
  std::atomic<bool> halted_{false};
  std::atomic<int> incident_{0};
  std::atomic<bool> warned_no_pose_{false};

  std::atomic<std::uint64_t> frames_built_{0};
  std::atomic<std::uint64_t> frames_sent_{0};
  std::atomic<std::uint64_t> frames_dropped_{0};
  std::atomic<std::uint64_t> bytes_sent_{0};
  std::atomic<std::uint64_t> commands_received_{0};
  std::atomic<std::uint64_t> commands_rejected_{0};
  std::atomic<std::uint64_t> estops_received_{0};
  std::atomic<std::uint64_t> trajectories_forwarded_{0};
  std::atomic<std::uint64_t> trajectories_rejected_{0};
  std::atomic<std::size_t> last_frame_bytes_{0};
  std::atomic<std::size_t> last_voxel_count_{0};
  std::atomic<std::size_t> last_voxels_dropped_{0};
};

}  // namespace sax
