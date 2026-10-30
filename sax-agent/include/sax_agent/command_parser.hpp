// Parsing of commands sent by sax-bridge to the agent.
//
//   SXE|<robot>|<operator>                    E-stop override (robot may be "*" for every robot)
//   SXP|<robot>|<x_cm>,<y_cm>;<x_cm>,<y_cm>   recovery trajectory
//   SXR|<robot>|<operator>                    release a latched E-stop (optional extension)
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace sax {

struct Waypoint {
  double x_m{0.0};
  double y_m{0.0};
};

struct EStopCommand {
  std::string operator_id;
  bool operator_valid{true};  // false when the operator field was missing or malformed
};

struct ReleaseCommand {
  std::string operator_id;
};

struct TrajectoryCommand {
  std::vector<Waypoint> waypoints;
};

using Command = std::variant<EStopCommand, ReleaseCommand, TrajectoryCommand>;

struct ParseResult {
  std::optional<Command> command;
  std::string error;  // set whenever `command` is empty
};

inline constexpr std::size_t kMaxWaypoints = 256;
inline constexpr long long kMaxCoordinateCm = 1'000'000;  // 10 km

/// Parse one inbound line for the robot named `robot_id`.
///
/// E-stop is fail-safe: any line starting with `SXE` stops this robot unless it explicitly names a
/// different robot, even if the operator field is missing or malformed. Every other command must be
/// well-formed and addressed to this robot exactly.
[[nodiscard]] ParseResult parse_command(std::string_view line, std::string_view robot_id);

}  // namespace sax
