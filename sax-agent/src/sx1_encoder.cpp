#include "sax_agent/sx1_encoder.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <system_error>

namespace sax {
namespace {

constexpr std::int32_t kGridOffset = 32768;
constexpr double kGridMin = -32768.0;
constexpr double kGridMax = 32767.0;

constexpr std::uint64_t pack(const VoxelIndex v) noexcept {
  return (static_cast<std::uint64_t>(static_cast<std::uint16_t>(v.i + kGridOffset)) << 32U) |
         (static_cast<std::uint64_t>(static_cast<std::uint16_t>(v.j + kGridOffset)) << 16U) |
         static_cast<std::uint64_t>(static_cast<std::uint16_t>(v.k + kGridOffset));
}

constexpr VoxelIndex unpack(const std::uint64_t key) noexcept {
  return VoxelIndex{static_cast<std::int16_t>(static_cast<std::int32_t>((key >> 32U) & 0xFFFFU) - kGridOffset),
                    static_cast<std::int16_t>(static_cast<std::int32_t>((key >> 16U) & 0xFFFFU) - kGridOffset),
                    static_cast<std::int16_t>(static_cast<std::int32_t>(key & 0xFFFFU) - kGridOffset)};
}

constexpr bool is_ascii_alnum(const char c) noexcept {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

Quaternion normalized(const Quaternion& q) noexcept {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return Quaternion{0.0, 0.0, 0.0, 1.0};
  }
  return Quaternion{q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

std::int64_t to_centimetres(const double metres) noexcept {
  const double cm = metres * 100.0;
  if (!std::isfinite(cm) || std::fabs(cm) > 9.0e15) {
    return 0;
  }
  return static_cast<std::int64_t>(std::llround(cm));
}

template <typename Int>
void append_integer(std::string& out, const Int value) {
  char buffer[24];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  out.append(buffer, result.ptr);
}

// Fixed 4 decimals with trailing zeros trimmed: 0.3827, -0.5, 1, 0.
void append_fraction(std::string& out, double value) {
  if (!std::isfinite(value)) {
    value = 0.0;
  }
  char buffer[40];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value, std::chars_format::fixed, 4);
  if (result.ec != std::errc{}) {
    out += '0';
    return;
  }
  std::string_view text(buffer, static_cast<std::size_t>(result.ptr - buffer));
  if (text.find('.') != std::string_view::npos) {
    while (text.back() == '0') {
      text.remove_suffix(1);
    }
    if (text.back() == '.') {
      text.remove_suffix(1);
    }
  }
  if (text == "-0") {
    text = "0";
  }
  out.append(text);
}

}  // namespace

bool is_valid_robot_id(const std::string_view id) noexcept {
  return !id.empty() && id.size() <= 32 &&
         std::all_of(id.begin(), id.end(), [](const char c) { return is_ascii_alnum(c) || c == '_' || c == '-'; });
}

bool is_valid_operator_id(const std::string_view id) noexcept {
  return !id.empty() && id.size() <= 64 && std::all_of(id.begin(), id.end(), [](const char c) {
           return is_ascii_alnum(c) || c == '_' || c == '.' || c == '@' || c == '-';
         });
}

Transform3 Transform3::from_pose(const Pose& pose) {
  return from_translation_rotation(pose.position_m, pose.orientation);
}

Transform3 Transform3::from_translation_rotation(const Vec3& translation, const Quaternion& rotation) {
  const Quaternion q = normalized(rotation);
  Transform3 transform;
  transform.t_ = translation;
  transform.r_ = {1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y - q.z * q.w), 2.0 * (q.x * q.z + q.y * q.w),
                  2.0 * (q.x * q.y + q.z * q.w), 1.0 - 2.0 * (q.x * q.x + q.z * q.z), 2.0 * (q.y * q.z - q.x * q.w),
                  2.0 * (q.x * q.z - q.y * q.w), 2.0 * (q.y * q.z + q.x * q.w), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
  return transform;
}

Vec3 Transform3::apply(const Vec3& p) const noexcept {
  return Vec3{r_[0] * p.x + r_[1] * p.y + r_[2] * p.z + t_.x, r_[3] * p.x + r_[4] * p.y + r_[5] * p.z + t_.y,
              r_[6] * p.x + r_[7] * p.y + r_[8] * p.z + t_.z};
}

VoxelQuantizer::VoxelQuantizer(VoxelizerConfig config) : config_(config), inverse_size_(0.0) {
  if (!(config_.voxel_size_m > 0.0) || !std::isfinite(config_.voxel_size_m)) {
    throw std::invalid_argument("voxel_size_m must be positive");
  }
  if (!(config_.max_range_m > config_.min_range_m) || config_.min_range_m < 0.0) {
    throw std::invalid_argument("max_range_m must exceed min_range_m, which must not be negative");
  }
  if (!(config_.max_z_m > config_.min_z_m)) {
    throw std::invalid_argument("max_z_m must exceed min_z_m");
  }
  config_.min_points_per_voxel = std::max<std::uint16_t>(config_.min_points_per_voxel, 1);
  inverse_size_ = 1.0 / config_.voxel_size_m;
}

std::optional<VoxelIndex> VoxelQuantizer::quantize(const Vec3& point) const noexcept {
  const double fi = std::floor(point.x * inverse_size_);
  const double fj = std::floor(point.y * inverse_size_);
  const double fk = std::floor(point.z * inverse_size_);
  // Written as positive comparisons so that NaN is rejected too.
  if (!(fi >= kGridMin && fi <= kGridMax && fj >= kGridMin && fj <= kGridMax && fk >= kGridMin && fk <= kGridMax)) {
    return std::nullopt;
  }
  return VoxelIndex{static_cast<std::int16_t>(fi), static_cast<std::int16_t>(fj), static_cast<std::int16_t>(fk)};
}

std::vector<VoxelIndex> VoxelQuantizer::voxelize(const std::span<const Vec3> points, const Vec3& origin) const {
  const double min_r2 = config_.min_range_m * config_.min_range_m;
  const double max_r2 = config_.max_range_m * config_.max_range_m;

  std::vector<std::uint64_t> keys;
  keys.reserve(points.size());
  for (const Vec3& p : points) {
    const double dx = p.x - origin.x;
    const double dy = p.y - origin.y;
    const double dz = p.z - origin.z;
    const double r2 = dx * dx + dy * dy + dz * dz;
    if (!(r2 >= min_r2 && r2 <= max_r2)) {
      continue;
    }
    if (!(p.z >= config_.min_z_m && p.z <= config_.max_z_m)) {
      continue;
    }
    if (const auto index = quantize(p)) {
      keys.push_back(pack(*index));
    }
  }
  std::sort(keys.begin(), keys.end());

  struct Cell {
    VoxelIndex index;
    double distance2;
  };
  std::vector<Cell> cells;
  for (std::size_t first = 0; first < keys.size();) {
    std::size_t last = first;
    while (last < keys.size() && keys[last] == keys[first]) {
      ++last;
    }
    if (last - first >= config_.min_points_per_voxel) {
      const VoxelIndex v = unpack(keys[first]);
      const double cx = (static_cast<double>(v.i) + 0.5) * config_.voxel_size_m - origin.x;
      const double cy = (static_cast<double>(v.j) + 0.5) * config_.voxel_size_m - origin.y;
      const double cz = (static_cast<double>(v.k) + 0.5) * config_.voxel_size_m - origin.z;
      cells.push_back(Cell{v, cx * cx + cy * cy + cz * cz});
    }
    first = last;
  }

  std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) {
    return a.distance2 != b.distance2 ? a.distance2 < b.distance2 : a.index < b.index;
  });
  if (cells.size() > config_.max_voxels) {
    cells.erase(cells.begin() + static_cast<std::ptrdiff_t>(config_.max_voxels), cells.end());
  }

  std::vector<VoxelIndex> result;
  result.reserve(cells.size());
  for (const Cell& cell : cells) {
    result.push_back(cell.index);
  }
  return result;
}

Sx1Encoder::Sx1Encoder(std::string robot_id) : robot_id_(std::move(robot_id)) {
  if (!is_valid_robot_id(robot_id_)) {
    throw std::invalid_argument("robot_id must be 1-32 characters of [A-Za-z0-9_-]");
  }
}

std::string Sx1Encoder::encode(const Frame& frame, const std::size_t max_bytes, EncodeStats* stats) {
  return encode_line(robot_id_, sequence_++, frame, max_bytes, stats);
}

std::string Sx1Encoder::encode_line(const std::string_view robot_id, const std::uint64_t sequence, const Frame& frame,
                                    const std::size_t max_bytes, EncodeStats* stats) {
  std::string line;
  line.reserve(max_bytes != 0 ? max_bytes : 96 + frame.voxels.size() * 12);

  line += "SX1|";
  line += robot_id;
  line += '|';
  append_integer(line, sequence);
  line += '|';
  line += static_cast<char>(frame.state);
  if (frame.state == RobotState::Stuck && frame.reason > 0) {
    append_integer(line, std::min(frame.reason, 99));
  }
  line += '|';

  const Quaternion q = normalized(frame.pose.orientation);
  append_integer(line, to_centimetres(frame.pose.position_m.x));
  line += ',';
  append_integer(line, to_centimetres(frame.pose.position_m.y));
  line += ',';
  append_integer(line, to_centimetres(frame.pose.position_m.z));
  for (const double component : {q.x, q.y, q.z, q.w}) {
    line += ',';
    append_fraction(line, component);
  }
  line += '|';

  std::size_t encoded = 0;
  char cell[24];
  for (const VoxelIndex& v : frame.voxels) {
    char* end = cell;
    end = std::to_chars(end, cell + sizeof cell, static_cast<int>(v.i)).ptr;
    *end++ = '.';
    end = std::to_chars(end, cell + sizeof cell, static_cast<int>(v.j)).ptr;
    *end++ = '.';
    end = std::to_chars(end, cell + sizeof cell, static_cast<int>(v.k)).ptr;
    const auto length = static_cast<std::size_t>(end - cell);
    if (max_bytes != 0 && line.size() + length + (encoded != 0 ? 1U : 0U) > max_bytes) {
      break;
    }
    if (encoded != 0) {
      line += ';';
    }
    line.append(cell, length);
    ++encoded;
  }

  if (stats != nullptr) {
    stats->voxels_encoded = encoded;
    stats->voxels_dropped = frame.voxels.size() - encoded;
    stats->bytes = line.size();
  }
  return line;
}

}  // namespace sax
