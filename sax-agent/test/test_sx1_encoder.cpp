#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "check.hpp"
#include "sax_agent/sx1_encoder.hpp"

using namespace sax;

namespace {

// Shared with scripts/run_agent_cli.py and its tests: both encoders must produce this exact line.
constexpr const char* kGolden = "SX1|03|42|S1|120,-45,0,0,0,0.3827,0.9239|-60.-40.0;-59.-40.1";

void test_quantize_floors_onto_a_20cm_grid() {
  const VoxelQuantizer quantizer;
  const auto a = quantizer.quantize({0.6, 0.0, -0.01});
  CHECK(a && a->i == 3 && a->j == 0 && a->k == -1);
  const auto b = quantizer.quantize({0.19999, 0.2, 0.39999});
  CHECK(b && b->i == 0 && b->j == 1 && b->k == 1);
  CHECK(!quantizer.quantize({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
  CHECK(!quantizer.quantize({1.0e9, 0.0, 0.0}));
  CHECK(!quantizer.quantize({0.0, -7000.0, 0.0}));
}

void test_voxelize_filters_and_orders_nearest_first() {
  const std::vector<Vec3> points = {
      {1.01, 0.01, 0.01}, {1.05, 0.05, 0.05},          // voxel (5,0,0)
      {2.01, 0.00, 0.01}, {2.05, 0.00, 0.05},          // voxel (10,0,0)
      {0.51, 0.00, 0.01}, {0.55, 0.00, 0.05},          // voxel (2,0,0), nearest
      {3.01, 0.01, 0.01},                              // single return: dropped
      {0.10, 0.10, 0.10}, {0.12, 0.12, 0.12},          // inside min range: dropped
      {9.00, 0.00, 0.00}, {9.00, 0.00, 0.00},          // beyond max range: dropped
      {2.01, 0.00, 5.00}, {2.02, 0.00, 5.00},          // above the height window: dropped
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
  };
  const std::vector<VoxelIndex> all = VoxelQuantizer().voxelize(points, {0.0, 0.0, 0.0});
  CHECK(all.size() == 3);
  CHECK(all[0] == (VoxelIndex{2, 0, 0}) && all[1] == (VoxelIndex{5, 0, 0}) && all[2] == (VoxelIndex{10, 0, 0}));

  VoxelizerConfig capped;
  capped.max_voxels = 2;
  const std::vector<VoxelIndex> nearest = VoxelQuantizer(capped).voxelize(points, {0.0, 0.0, 0.0});
  CHECK(nearest.size() == 2 && nearest[1] == (VoxelIndex{5, 0, 0}));
  CHECK(VoxelQuantizer().voxelize({}, {0.0, 0.0, 0.0}).empty());
}

void test_config_validation() {
  VoxelizerConfig bad;
  bad.voxel_size_m = 0.0;
  bool threw = false;
  try {
    const VoxelQuantizer quantizer(bad);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

void test_transform_rotates_then_translates() {
  const double s = std::sqrt(0.5);
  const Transform3 transform = Transform3::from_translation_rotation({1.0, 2.0, 3.0}, {0.0, 0.0, s, s});
  const Vec3 p = transform.apply({1.0, 0.0, 0.0});
  CHECK(std::fabs(p.x - 1.0) < 1e-9 && std::fabs(p.y - 3.0) < 1e-9 && std::fabs(p.z - 3.0) < 1e-9);
  const Vec3 identity = Transform3().apply({4.0, 5.0, 6.0});
  CHECK(identity.x == 4.0 && identity.y == 5.0 && identity.z == 6.0);
}

void test_encode_matches_the_golden_line() {
  const std::vector<VoxelIndex> voxels{{-60, -40, 0}, {-59, -40, 1}};
  const Frame frame{RobotState::Stuck, kReasonPathBlocked, Pose{{1.2, -0.45, 0.0}, {0.0, 0.0, 0.3827, 0.9239}}, voxels};
  EncodeStats stats;
  CHECK_STR(Sx1Encoder::encode_line("03", 42, frame, 0, &stats), kGolden);
  CHECK(stats.voxels_encoded == 2 && stats.voxels_dropped == 0 && stats.bytes == std::string(kGolden).size());
}

void test_encode_formatting_edge_cases() {
  const Frame zero_quaternion{RobotState::Active, 0, Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}, {}};
  CHECK_STR(Sx1Encoder::encode_line("r1", 0, zero_quaternion), "SX1|r1|0|A|0,0,0,0,0,0,1|");

  const Frame tiny{RobotState::Halted, 0, Pose{{-0.001, 0.004, 12.345}, {-0.00001, 0.0, 0.0, 1.0}}, {}};
  CHECK_STR(Sx1Encoder::encode_line("r1", 7, tiny), "SX1|r1|7|H|0,0,1235,0,0,0,1|");

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Frame broken{RobotState::Active, 0, Pose{{nan, std::numeric_limits<double>::infinity(), 0.0}, {nan, 0.0, 0.0, 1.0}}, {}};
  CHECK_STR(Sx1Encoder::encode_line("r1", 1, broken), "SX1|r1|1|A|0,0,0,0,0,0,1|");

  CHECK_STR(Sx1Encoder::encode_line("r1", 0, Frame{RobotState::Stuck, 0, {}, {}}), "SX1|r1|0|S|0,0,0,0,0,0,1|");
  CHECK_STR(Sx1Encoder::encode_line("r1", 0, Frame{RobotState::Stuck, 150, {}, {}}), "SX1|r1|0|S99|0,0,0,0,0,0,1|");
  CHECK_STR(Sx1Encoder::encode_line("r1", 0, Frame{RobotState::Active, 5, {}, {}}), "SX1|r1|0|A|0,0,0,0,0,0,1|");
}

void test_encoder_increments_the_sequence() {
  Sx1Encoder encoder("robot_01");
  CHECK(encoder.next_sequence() == 0);
  CHECK(encoder.encode(Frame{}).starts_with("SX1|robot_01|0|"));
  CHECK(encoder.encode(Frame{}).starts_with("SX1|robot_01|1|"));
  CHECK(encoder.next_sequence() == 2);
}

void test_byte_budget_truncates_whole_cells() {
  std::vector<VoxelIndex> voxels;
  for (std::int16_t n = 0; n < 100; ++n) {
    voxels.push_back({n, static_cast<std::int16_t>(-n), 3});
  }
  Frame frame{RobotState::Active, 0, {}, voxels};
  const std::size_t header = Sx1Encoder::encode_line("r1", 0, Frame{}).size();
  EncodeStats stats;
  const std::string line = Sx1Encoder::encode_line("r1", 0, frame, header + 30, &stats);
  CHECK(line.size() <= header + 30 && stats.bytes == line.size());
  CHECK(stats.voxels_encoded > 0 && stats.voxels_dropped > 0 && stats.voxels_encoded + stats.voxels_dropped == 100);
  CHECK(line.ends_with("3") && line.find(";;") == std::string::npos);
  CHECK(line.find("|0.0.3;1.-1.3;2.-2.3") != std::string::npos);
}

void test_identifier_validation() {
  CHECK(is_valid_robot_id("robot_01") && is_valid_robot_id("A-1"));
  CHECK(!is_valid_robot_id("") && !is_valid_robot_id("bad id") && !is_valid_robot_id("a|b"));
  CHECK(!is_valid_robot_id(std::string(33, 'a')) && is_valid_robot_id(std::string(32, 'a')));
  CHECK(is_valid_operator_id("jibrin") && is_valid_operator_id("ann.lee@corp-1"));
  CHECK(!is_valid_operator_id("") && !is_valid_operator_id("a b") && !is_valid_operator_id(std::string(65, 'a')));
  bool threw = false;
  try {
    Sx1Encoder encoder("bad id");
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

}  // namespace

int main() {
  RUN(test_quantize_floors_onto_a_20cm_grid);
  RUN(test_voxelize_filters_and_orders_nearest_first);
  RUN(test_config_validation);
  RUN(test_transform_rotates_then_translates);
  RUN(test_encode_matches_the_golden_line);
  RUN(test_encode_formatting_edge_cases);
  RUN(test_encoder_increments_the_sequence);
  RUN(test_byte_budget_truncates_whole_cells);
  RUN(test_identifier_validation);
  return 0;
}
