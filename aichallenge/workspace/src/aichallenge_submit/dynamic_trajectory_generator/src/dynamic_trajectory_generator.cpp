#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

class DynamicTrajectoryGenerator : public rclcpp::Node {
public:
  DynamicTrajectoryGenerator()
      : Node("dynamic_trajectory_generator"), has_trajectory_(false) {
    this->declare_parameter<std::string>("map_path", "");
    std::string map_path = this->get_parameter("map_path").as_string();

    if (!initLaneletMap(map_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load Lanelet2 map: %s",
                   map_path.c_str());
    }

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&DynamicTrajectoryGenerator::onGoalReceived, this,
                  std::placeholders::_1));

    traj_pub_ =
        this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/scenario_planning/trajectory", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DynamicTrajectoryGenerator::publishTrajectory, this));
  }

private:
  bool initLaneletMap(const std::string &map_path) {
    if (map_path.empty())
      return false;
    lanelet::projection::UtmProjector projector(lanelet::Origin(lanelet::GPSPoint{0.0, 0.0}));
    lanelet_map_ = lanelet::load(map_path, projector);

    lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    routing_graph_ =
        lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules);
    return true;
  }

  void
  onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
    if (!lanelet_map_ || !routing_graph_) {
      RCLCPP_ERROR(this->get_logger(), "Lanelet map is not initialized.");
      return;
    }

    geometry_msgs::msg::TransformStamped robot_pose;
    try {
      robot_pose =
          tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
      return;
    }

    lanelet::BasicPoint2d start_pt(robot_pose.transform.translation.x,
                                   robot_pose.transform.translation.y);
    lanelet::BasicPoint2d goal_pt(goal_msg->pose.position.x,
                                  goal_msg->pose.position.y);

    auto start_lanelets =
        lanelet::geometry::findNearest(lanelet_map_->laneletLayer, start_pt, 1);
    auto goal_lanelets =
        lanelet::geometry::findNearest(lanelet_map_->laneletLayer, goal_pt, 1);

    if (start_lanelets.empty() || goal_lanelets.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Nearest lanelet not found.");
      return;
    }

    auto route = routing_graph_->getRoute(start_lanelets.front().second,
                                          goal_lanelets.front().second);
    if (!route) {
      RCLCPP_ERROR(this->get_logger(), "Route not found.");
      return;
    }

    lanelet::routing::LaneletPath shortest_path = route->shortestPath();
    auto centerline_poses = extractAndResampleCenterline(shortest_path);

    cached_trajectory_.points.clear();
    cached_trajectory_.header.frame_id = "map";

    const double target_speed_mps = 4.16; // 15 km/h
    const size_t total_points = centerline_poses.size();

    for (size_t i = 0; i < total_points; ++i) {
      autoware_auto_planning_msgs::msg::TrajectoryPoint p;
      p.pose = centerline_poses[i];

      if (total_points > 25 && i >= total_points - 25) {
        double ratio = static_cast<double>(total_points - 1 - i) / 25.0;
        p.longitudinal_velocity_mps =
            static_cast<float>(target_speed_mps * ratio);
      } else {
        p.longitudinal_velocity_mps = static_cast<float>(target_speed_mps);
      }
      cached_trajectory_.points.push_back(p);
    }

    has_trajectory_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Dynamic Trajectory Generated with %zu points.",
                cached_trajectory_.points.size());
  }

  std::vector<geometry_msgs::msg::Pose>
  extractAndResampleCenterline(const lanelet::routing::LaneletPath &path) {
    std::vector<lanelet::BasicPoint2d> raw_centerline;
    for (const auto &ll : path) {
      auto cl = ll.centerline2d();
      for (const auto &pt : cl) {
        raw_centerline.push_back(pt);
      }
    }

    std::vector<geometry_msgs::msg::Pose> resampled_poses;
    if (raw_centerline.size() < 2)
      return resampled_poses;

    const double step_size = 0.2; // 0.2m 間隔

    for (size_t i = 0; i < raw_centerline.size() - 1; ++i) {
      double p1_x = raw_centerline[i].x();
      double p1_y = raw_centerline[i].y();
      double p2_x = raw_centerline[i + 1].x();
      double p2_y = raw_centerline[i + 1].y();

      double seg_len = std::hypot(p2_x - p1_x, p2_y - p1_y);
      if (seg_len < 1e-4)
        continue;

      double yaw = std::atan2(p2_y - p1_y, p2_x - p1_x);
      geometry_msgs::msg::Quaternion q;
      q.z = std::sin(yaw / 2.0);
      q.w = std::cos(yaw / 2.0);

      for (double s = 0; s < seg_len; s += step_size) {
        double t = s / seg_len;
        geometry_msgs::msg::Pose p;
        p.position.x = p1_x + t * (p2_x - p1_x);
        p.position.y = p1_y + t * (p2_y - p1_y);
        p.orientation = q;
        resampled_poses.push_back(p);
      }
    }
    return resampled_poses;
  }

  void publishTrajectory() {
    if (!has_trajectory_)
      return;
    cached_trajectory_.header.stamp = this->now();
    traj_pub_->publish(cached_trajectory_);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
      traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;

  autoware_auto_planning_msgs::msg::Trajectory cached_trajectory_;
  bool has_trajectory_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}