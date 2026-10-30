#include "sax_agent/command_parser.hpp"

#include <charconv>
#include <cstdint>
#include <system_error>

#include "sax_agent/sx1_encoder.hpp"

namespace sax {
namespace {

std::vector<std::string_view> split(const std::string_view text, const char separator) {
  std::vector<std::string_view> parts;
  std::size_t start = 0;
  while (true) {
    const std::size_t end = text.find(separator, start);
    if (end == std::string_view::npos) {
      parts.push_back(text.substr(start));
      return parts;
    }
    parts.push_back(text.substr(start, end - start));
    start = end + 1;
  }
}

std::string_view trim(std::string_view text) {
  constexpr std::string_view kSpace = " \t\r\n";
  const std::size_t first = text.find_first_not_of(kSpace);
  if (first == std::string_view::npos) {
    return {};
  }
  return text.substr(first, text.find_last_not_of(kSpace) - first + 1);
}

bool parse_int(const std::string_view text, long long& value) noexcept {
  if (text.empty()) {
    return false;
  }
  const char* const end = text.data() + text.size();
  const auto [pointer, error] = std::from_chars(text.data(), end, value);
  return error == std::errc{} && pointer == end;
}

ParseResult failure(std::string message) {
  return ParseResult{std::nullopt, std::move(message)};
}

ParseResult parse_estop(const std::vector<std::string_view>& fields, const std::string_view robot_id) {
  const std::string_view target = fields.size() > 1 ? fields[1] : std::string_view{};
  if (!target.empty() && target != "*" && target != robot_id) {
    return failure("E-stop is addressed to another robot");
  }
  EStopCommand command;
  const std::string_view operator_id = fields.size() > 2 ? fields[2] : std::string_view{};
  if (is_valid_operator_id(operator_id)) {
    command.operator_id = std::string(operator_id);
  } else {
    command.operator_id = "unknown";
    command.operator_valid = false;
  }
  ParseResult result;
  result.command = Command{std::move(command)};
  return result;
}

ParseResult parse_release(const std::vector<std::string_view>& fields, const std::string_view robot_id) {
  if (fields.size() < 3 || fields[1] != robot_id) {
    return failure("SXR must be SXR|<this robot>|<operator>");
  }
  if (!is_valid_operator_id(fields[2])) {
    return failure("SXR operator id is malformed");
  }
  ParseResult result;
  result.command = Command{ReleaseCommand{std::string(fields[2])}};
  return result;
}

ParseResult parse_trajectory(const std::vector<std::string_view>& fields, const std::string_view robot_id) {
  if (fields.size() != 3) {
    return failure("SXP must be SXP|<robot>|<x_cm>,<y_cm>;...");
  }
  if (fields[1] != robot_id) {
    return failure("trajectory is addressed to another robot");
  }
  const std::vector<std::string_view> points = split(fields[2], ';');
  if (points.size() < 2 || points.size() > kMaxWaypoints) {
    return failure("trajectory needs between 2 and 256 points");
  }
  TrajectoryCommand command;
  command.waypoints.reserve(points.size());
  for (const std::string_view point : points) {
    const std::vector<std::string_view> axes = split(point, ',');
    long long x_cm = 0;
    long long y_cm = 0;
    if (axes.size() != 2 || !parse_int(axes[0], x_cm) || !parse_int(axes[1], y_cm)) {
      return failure("trajectory points must be integer x_cm,y_cm pairs");
    }
    if (x_cm < -kMaxCoordinateCm || x_cm > kMaxCoordinateCm || y_cm < -kMaxCoordinateCm || y_cm > kMaxCoordinateCm) {
      return failure("trajectory coordinate is outside +/-10 km");
    }
    command.waypoints.push_back(Waypoint{static_cast<double>(x_cm) / 100.0, static_cast<double>(y_cm) / 100.0});
  }
  ParseResult result;
  result.command = Command{std::move(command)};
  return result;
}

}  // namespace

ParseResult parse_command(std::string_view line, const std::string_view robot_id) {
  line = trim(line);
  if (line.empty()) {
    return failure("empty command");
  }
  const std::vector<std::string_view> fields = split(line, '|');
  if (fields[0] == "SXE") {
    return parse_estop(fields, robot_id);
  }
  if (fields[0] == "SXP") {
    return parse_trajectory(fields, robot_id);
  }
  if (fields[0] == "SXR") {
    return parse_release(fields, robot_id);
  }
  return failure("unknown command verb");
}

}  // namespace sax
