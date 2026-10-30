// SX1 telemetry encoding and 3D voxel quantization.
//
// Wire format, one robot frame per line:
//   SX1|<robot_id>|<seq>|<state>|<x_cm>,<y_cm>,<z_cm>,<qx>,<qy>,<qz>,<qw>|<i>.<j>.<k>;<i>.<j>.<k>;...
//
// * state: A (active), H (halted) or S<reason> (stuck, reason 1-99)
// * position: integer centimetres in the world (odom) frame
// * orientation: unit quaternion, 4 decimals (the bridge normalises on receipt)
// * voxels: integer cells of a 20 cm grid; cell (i,j,k) covers [i*0.2, (i+1)*0.2) on each axis
#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sax {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// Robot pose in metres, expressed in the world frame.
struct Pose {
  Vec3 position_m;
  Quaternion orientation;
};

enum class RobotState : char { Active = 'A', Stuck = 'S', Halted = 'H' };

/// Reason codes carried by the stuck state (S1, S2, S3). They match the sax-view incident reasons.
inline constexpr int kReasonPathBlocked = 1;
inline constexpr int kReasonWheelSlip = 2;
inline constexpr int kReasonLocalizationLost = 3;

/// Rigid transform p' = R * p + t, used to move sensor points into the world frame.
class Transform3 {
 public:
  Transform3() = default;
  static Transform3 from_pose(const Pose& pose);
  static Transform3 from_translation_rotation(const Vec3& translation, const Quaternion& rotation);
  [[nodiscard]] Vec3 apply(const Vec3& point) const noexcept;

 private:
  std::array<double, 9> r_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  Vec3 t_{};
};

/// Integer cell of the occupancy grid. Ordered lexicographically by (i, j, k).
struct VoxelIndex {
  std::int16_t i{0};
  std::int16_t j{0};
  std::int16_t k{0};

  friend constexpr bool operator==(const VoxelIndex&, const VoxelIndex&) = default;
  friend constexpr auto operator<=>(const VoxelIndex&, const VoxelIndex&) = default;
};

struct VoxelizerConfig {
  double voxel_size_m{0.20};
  double min_range_m{0.30};         // drops returns from the robot body
  double max_range_m{8.0};          // drops far, sparse returns
  double min_z_m{-0.30};            // world-frame height window; removes floor and ceiling
  double max_z_m{2.50};
  std::uint16_t min_points_per_voxel{2};  // single-return noise filter
  std::size_t max_voxels{1500};     // hard cap per frame, nearest cells kept first
};

class VoxelQuantizer {
 public:
  /// Throws std::invalid_argument when the configuration cannot produce a valid grid.
  explicit VoxelQuantizer(VoxelizerConfig config = {});

  /// Grid cell containing `point`, or nullopt when it is not finite or falls outside the int16 grid.
  [[nodiscard]] std::optional<VoxelIndex> quantize(const Vec3& point) const noexcept;

  /// Voxelize world-frame points. `origin` (the robot position) drives range gating, and the result
  /// is ordered nearest-first so that any later truncation keeps the closest obstacles.
  [[nodiscard]] std::vector<VoxelIndex> voxelize(std::span<const Vec3> points, const Vec3& origin) const;

  [[nodiscard]] const VoxelizerConfig& config() const noexcept { return config_; }

 private:
  VoxelizerConfig config_;
  double inverse_size_;
};

struct Frame {
  RobotState state{RobotState::Active};
  int reason{0};
  Pose pose{};
  std::span<const VoxelIndex> voxels{};
};

struct EncodeStats {
  std::size_t voxels_encoded{0};
  std::size_t voxels_dropped{0};
  std::size_t bytes{0};
};

class Sx1Encoder {
 public:
  /// Throws std::invalid_argument when `robot_id` is not 1-32 characters of [A-Za-z0-9_-].
  explicit Sx1Encoder(std::string robot_id);

  [[nodiscard]] const std::string& robot_id() const noexcept { return robot_id_; }
  [[nodiscard]] std::uint64_t next_sequence() const noexcept { return sequence_; }

  /// Encode `frame` with the next sequence number. Voxel cells are appended in order until adding
  /// another would exceed `max_bytes` (0 means unlimited). Not thread-safe.
  std::string encode(const Frame& frame, std::size_t max_bytes = 0, EncodeStats* stats = nullptr);

  /// Stateless variant used by tests and tools.
  static std::string encode_line(std::string_view robot_id, std::uint64_t sequence, const Frame& frame,
                                 std::size_t max_bytes = 0, EncodeStats* stats = nullptr);

 private:
  std::string robot_id_;
  std::uint64_t sequence_{0};
};

[[nodiscard]] bool is_valid_robot_id(std::string_view id) noexcept;
[[nodiscard]] bool is_valid_operator_id(std::string_view id) noexcept;

}  // namespace sax
