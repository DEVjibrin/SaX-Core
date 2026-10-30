#include <string>
#include <variant>

#include "check.hpp"
#include "sax_agent/command_parser.hpp"

using namespace sax;

namespace {

const EStopCommand* as_estop(const ParseResult& result) {
  return result.command ? std::get_if<EStopCommand>(&*result.command) : nullptr;
}

void test_estop_variants() {
  const ParseResult normal = parse_command("SXE|robot_01|jibrin", "robot_01");
  CHECK(as_estop(normal) && as_estop(normal)->operator_id == "jibrin" && as_estop(normal)->operator_valid);

  const ParseResult fleet = parse_command("SXE|*|maya", "robot_01");
  CHECK(as_estop(fleet) && as_estop(fleet)->operator_id == "maya");

  const ParseResult crlf = parse_command("  SXE|robot_01|ann\r\n", "robot_01");
  CHECK(as_estop(crlf) && as_estop(crlf)->operator_id == "ann");
}

void test_estop_is_fail_safe() {
  for (const char* line : {"SXE|robot_01", "SXE|robot_01|bad op!", "SXE|robot_01||", "SXE||ann", "SXE", "SXE|robot_01|ann|extra"}) {
    const ParseResult result = parse_command(line, "robot_01");
    CHECK(as_estop(result) != nullptr);
  }
  const ParseResult unknown_operator = parse_command("SXE|robot_01|bad op!", "robot_01");
  CHECK(as_estop(unknown_operator)->operator_id == "unknown" && !as_estop(unknown_operator)->operator_valid);
}

void test_estop_for_another_robot_is_ignored() {
  const ParseResult result = parse_command("SXE|robot_02|ann", "robot_01");
  CHECK(!result.command && !result.error.empty());
}

void test_trajectory_parsing() {
  const ParseResult result = parse_command("SXP|robot_01|0,0;100,50;250,-75", "robot_01");
  CHECK(result.command);
  const auto* trajectory = std::get_if<TrajectoryCommand>(&*result.command);
  CHECK(trajectory && trajectory->waypoints.size() == 3);
  CHECK(trajectory->waypoints[1].x_m == 1.0 && trajectory->waypoints[1].y_m == 0.5);
  CHECK(trajectory->waypoints[2].x_m == 2.5 && trajectory->waypoints[2].y_m == -0.75);
}

void test_malformed_trajectories_are_rejected() {
  std::string too_many = "SXP|robot_01|0,0";
  for (int n = 0; n < 256; ++n) {
    too_many += ";1,1";
  }
  for (const std::string& line : {std::string("SXP|robot_01"), std::string("SXP|robot_01|"), std::string("SXP|robot_01|1,2"),
                                  std::string("SXP|robot_01|1,2;3"), std::string("SXP|robot_01|1,2;a,b"),
                                  std::string("SXP|robot_01|1.5,2;3,4"), std::string("SXP|robot_01|1,2;3,4;"),
                                  std::string("SXP|robot_01|0,0;2000000,0"), std::string("SXP|robot_02|0,0;1,1"),
                                  std::string("SXP|robot_01|0,0;1,1|extra"), too_many}) {
    const ParseResult result = parse_command(line, "robot_01");
    CHECK(!result.command);
    CHECK(!result.error.empty());
  }
}

void test_release_and_unknown_verbs() {
  const ParseResult release = parse_command("SXR|robot_01|ann", "robot_01");
  CHECK(release.command && std::get_if<ReleaseCommand>(&*release.command));
  CHECK(!parse_command("SXR|robot_01", "robot_01").command);
  CHECK(!parse_command("SXR|robot_02|ann", "robot_01").command);
  CHECK(!parse_command("", "robot_01").command);
  CHECK(!parse_command("   \r\n", "robot_01").command);
  CHECK(!parse_command("SXZ|robot_01|ann", "robot_01").command);
  CHECK(!parse_command(std::string(5000, 'x'), "robot_01").command);
}

}  // namespace

int main() {
  RUN(test_estop_variants);
  RUN(test_estop_is_fail_safe);
  RUN(test_estop_for_another_robot_is_ignored);
  RUN(test_trajectory_parsing);
  RUN(test_malformed_trajectories_are_rejected);
  RUN(test_release_and_unknown_verbs);
  return 0;
}
