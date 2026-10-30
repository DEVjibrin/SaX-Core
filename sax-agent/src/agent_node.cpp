// ROS 2 node: turns /point_cloud and /odom into SX1 frames for sax-bridge and exposes the safety hooks.
//
//   subscribes  /point_cloud    sensor_msgs/PointCloud2   any frame; transformed into the odometry frame
//               /odom           nav_msgs/Odometry         robot pose in the world frame
//               /sax/incident   std_msgs/UInt8            navigation stack reports stuck reasons (0 clears)
//   publishes   /sax/estop          std_msgs/Bool         latched; true while an E-stop is engaged
//               /sax/recovery_path  nav_msgs/Path         trajectory received from an operator (SXP)
//               <estop_twist_topic> geometry_msgs/Twist   optional zero velocity at 20 Hz while stopped
//   service     ~/release_estop     std_srvs/Trigger      local release of a latched E-stop
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sax_agent/agent_core.hpp"
#include "sax_agent/websocket_client.hpp"

namespace {

std::string build_url(std::string base, const std::string& robot_id) {
  if (base.find("/ws/agent/") != std::string::npos) {
    return base;
  }
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }
  return base + "/ws/agent/" + robot_id;
}

bool finite_pose(const sax::Pose& pose) {
  return std::isfinite(pose.position_m.x + pose.position_m.y + pose.position_m.z + pose.orientation.x +
                       pose.orientation.y + pose.orientation.z + pose.orientation.w);
}

class AgentNode final : public rclcpp::Node {
 public:
  AgentNode() : rclcpp::Node("sax_agent") {
    sax::AgentCoreConfig config;
    config.robot_id = declare_parameter<std::string>("robot_id", "robot_01");
    config.frequency_hz = declare_parameter<double>("frequency_hz", 10.0);
    config.max_bandwidth_kbps = declare_parameter<double>("max_bandwidth_kbps", 900.0);
    config.cloud_timeout_s = declare_parameter<double>("cloud_timeout_s", 0.5);
    config.odom_timeout_s = declare_parameter<double>("odom_timeout_s", 1.0);
    config.voxel.voxel_size_m = declare_parameter<double>("voxel_size_m", 0.20);
    config.voxel.min_range_m = declare_parameter<double>("min_range_m", 0.30);
    config.voxel.max_range_m = declare_parameter<double>("max_range_m", 8.0);
    config.voxel.min_z_m = declare_parameter<double>("min_z_m", -0.30);
    config.voxel.max_z_m = declare_parameter<double>("max_z_m", 2.50);
    config.voxel.min_points_per_voxel = static_cast<std::uint16_t>(
        std::clamp<std::int64_t>(declare_parameter<std::int64_t>("min_points_per_voxel", 2), 1, 1000));
    config.voxel.max_voxels =
        static_cast<std::size_t>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("max_voxels", 1500)));
    point_stride_ = static_cast<std::size_t>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("point_stride", 1)));
    const bool persist_estop = declare_parameter<bool>("persist_estop", true);
    const auto latch_dir = declare_parameter<std::string>("estop_latch_dir", "/var/tmp");
    if (persist_estop) {
      config.latch_path = latch_dir + "/sax_agent_" + config.robot_id + ".estop";
    }

    sax::WebSocketClientConfig link;
    link.url = build_url(declare_parameter<std::string>("bridge_url", "ws://localhost:8000"), config.robot_id);
    link.bearer_token = declare_parameter<std::string>("bridge_token", "");
    link.compress = declare_parameter<bool>("compress", true);
    link.backoff_min = std::chrono::milliseconds(declare_parameter<std::int64_t>("reconnect_min_ms", 500));
    link.backoff_max = std::chrono::milliseconds(declare_parameter<std::int64_t>("reconnect_max_ms", 30'000));

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/point_cloud");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto incident_topic = declare_parameter<std::string>("incident_topic", "/sax/incident");
    const auto estop_topic = declare_parameter<std::string>("estop_topic", "/sax/estop");
    const auto path_topic = declare_parameter<std::string>("recovery_path_topic", "/sax/recovery_path");
    const auto twist_topic = declare_parameter<std::string>("estop_twist_topic", "");

    sax::AgentCallbacks callbacks;
    callbacks.on_estop = [this](const bool engaged, const std::string_view) { publish_estop(engaged); };
    callbacks.on_trajectory = [this](const std::vector<sax::Waypoint>& waypoints) { publish_path(waypoints); };
    callbacks.on_log = [this](const sax::LogLevel level, const std::string_view message) { forward_log(level, message); };

    core_ = std::make_unique<sax::AgentCore>(
        config, [this](const std::string_view line) { return client_ && client_->send_text(line); }, callbacks);

    estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic, rclcpp::QoS(1).reliable().transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1).reliable());
    if (!twist_topic.empty()) {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
    }
    publish_estop(core_->estopped());  // true again after a restart that found a latched E-stop

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = std::make_unique<sax::WebSocketClient>(
        link, [this](const std::string_view message) { core_->handle_command(message); },
        [this](const sax::LinkState state, const std::string_view detail) {
          const auto name = sax::to_string(state);
          RCLCPP_INFO(get_logger(), "bridge link %.*s: %.*s", static_cast<int>(name.size()), name.data(),
                      static_cast<int>(detail.size()), detail.data());
        });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(*message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odom(*message); });
    incident_sub_ = create_subscription<std_msgs::msg::UInt8>(
        incident_topic, 10, [this](const std_msgs::msg::UInt8::ConstSharedPtr message) { core_->set_incident(message->data); });

    release_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/release_estop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = core_->release_estop("local-ros-service");
          response->message = response->success ? "E-stop released" : "no E-stop was latched";
        });

    tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config.frequency_hz), [this] { core_->tick(); });
    status_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { log_status(); });
    if (twist_pub_) {
      twist_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
        if (core_->estopped()) {
          twist_pub_->publish(geometry_msgs::msg::Twist());
        }
      });
    }

    client_->start();
    RCLCPP_INFO(get_logger(), "sax_agent for '%s' streaming to %s at %.1f Hz (budget %.0f kbps)", config.robot_id.c_str(),
                link.url.c_str(), config.frequency_hz, config.max_bandwidth_kbps);
  }

  ~AgentNode() override {
    if (client_) {
      client_->stop();  // no WebSocket callback may touch the node while it is being torn down
    }
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry& message) {
    sax::Pose pose;
    const auto& p = message.pose.pose.position;
    const auto& q = message.pose.pose.orientation;
    pose.position_m = {p.x, p.y, p.z};
    pose.orientation = {q.x, q.y, q.z, q.w};
    if (!finite_pose(pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring odometry with a non-finite pose");
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_frame_ = message.header.frame_id;
      base_frame_ = message.child_frame_id;
      latest_pose_ = pose;
      have_odom_ = true;
    }
    core_->update_pose(pose);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2& message) {
    sax::Pose pose;
    std::string odom_frame;
    std::string base_frame;
    {
      const std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_odom_) {
        return;
      }
      pose = latest_pose_;
      odom_frame = odom_frame_;
      base_frame = base_frame_;
    }

    sax::Transform3 to_world;  // identity: the cloud is already in the world frame
    const std::string& frame = message.header.frame_id;
    if (frame.empty() || frame == base_frame) {
      to_world = sax::Transform3::from_pose(pose);
    } else if (frame != odom_frame) {
      try {
        const auto transform = tf_buffer_->lookupTransform(odom_frame, frame, tf2::TimePointZero);
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        to_world = sax::Transform3::from_translation_rotation({t.x, t.y, t.z}, {r.x, r.y, r.z, r.w});
      } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no transform from '%s' to '%s': %s", frame.c_str(),
                             odom_frame.c_str(), error.what());
        return;
      }
    }

    try {
      const std::size_t count = static_cast<std::size_t>(message.width) * message.height;
      points_.clear();
      points_.reserve(count / point_stride_ + 1);
      sensor_msgs::PointCloud2ConstIterator<float> it_x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(message, "z");
      for (std::size_t n = 0; n < count; ++n, ++it_x, ++it_y, ++it_z) {
        if (n % point_stride_ != 0) {
          continue;
        }
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          points_.push_back(to_world.apply({x, y, z}));
        }
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unusable point cloud (needs float32 x, y, z): %s", error.what());
      return;
    }
    core_->update_cloud(points_);
  }

  void publish_estop(const bool engaged) {
    std_msgs::msg::Bool message;
    message.data = engaged;
    estop_pub_->publish(message);
    if (engaged && twist_pub_) {
      twist_pub_->publish(geometry_msgs::msg::Twist());  // do not wait for the 20 Hz timer
    }
  }

  void publish_path(const std::vector<sax::Waypoint>& waypoints) {
    std::string frame;
    {
      const std::lock_guard<std::mutex> lock(odom_mutex_);
      frame = odom_frame_;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame.empty() ? "odom" : frame;
    path.poses.reserve(waypoints.size());
    double yaw = 0.0;
    for (std::size_t n = 0; n < waypoints.size(); ++n) {
      if (n + 1 < waypoints.size()) {
        yaw = std::atan2(waypoints[n + 1].y_m - waypoints[n].y_m, waypoints[n + 1].x_m - waypoints[n].x_m);
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoints[n].x_m;
      pose.pose.position.y = waypoints[n].y_m;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void forward_log(const sax::LogLevel level, const std::string_view message) {
    const int length = static_cast<int>(message.size());
    switch (level) {
      case sax::LogLevel::Debug:
        RCLCPP_DEBUG(get_logger(), "%.*s", length, message.data());
        break;
      case sax::LogLevel::Info:
        RCLCPP_INFO(get_logger(), "%.*s", length, message.data());
        break;
      case sax::LogLevel::Warn:
        RCLCPP_WARN(get_logger(), "%.*s", length, message.data());
        break;
      case sax::LogLevel::Error:
        RCLCPP_ERROR(get_logger(), "%.*s", length, message.data());
        break;
    }
  }

  void log_status() {
    const sax::CoreStats stats = core_->stats();
    const sax::LinkStats link = client_->stats();
    const auto now_time = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now_time - last_status_at_).count();
    const double kbps =
        seconds > 0.0 ? static_cast<double>(stats.bytes_sent - last_bytes_sent_) * 8.0 / seconds / 1000.0 : 0.0;
    last_status_at_ = now_time;
    last_bytes_sent_ = stats.bytes_sent;
    const auto state = sax::to_string(client_->state());
    RCLCPP_INFO(get_logger(), "link=%.*s %.1f kbps | frames sent=%lu dropped=%lu | voxels=%zu (cut %zu) | reconnects=%lu%s",
                static_cast<int>(state.size()), state.data(), kbps, static_cast<unsigned long>(stats.frames_sent),
                static_cast<unsigned long>(stats.frames_dropped), stats.last_voxel_count, stats.last_voxels_dropped,
                static_cast<unsigned long>(link.connects > 0 ? link.connects - 1 : 0), stats.estopped ? " | E-STOP LATCHED" : "");
  }

  std::unique_ptr<sax::AgentCore> core_;
  std::unique_ptr<sax::WebSocketClient> client_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr incident_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr twist_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex odom_mutex_;
  bool have_odom_{false};
  sax::Pose latest_pose_{};
  std::string odom_frame_;
  std::string base_frame_;

  std::size_t point_stride_{1};
  std::vector<sax::Vec3> points_;

  std::chrono::steady_clock::time_point last_status_at_{std::chrono::steady_clock::now()};
  std::uint64_t last_bytes_sent_{0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<AgentNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sax_agent"), "fatal: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
