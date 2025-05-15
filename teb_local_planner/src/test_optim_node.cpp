#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "teb_local_planner/teb_local_planner_ros.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

using namespace teb_local_planner;

class TestOptimNode : public nav2_util::LifecycleNode
{
public:
  TestOptimNode();
  void init();

private:
  // Core planner components
  TebConfig config;
  PlannerInterfacePtr planner;
  TebVisualizationPtr visual;
  ObstContainer obst_vector;
  ViaPointContainer via_points;
  unsigned int no_fixed_obstacles;

  // Timers
  rclcpp::TimerBase::SharedPtr main_cycle_timer;
  rclcpp::TimerBase::SharedPtr publish_timer;

  // Subscriptions
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_points_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> obst_vel_subs;

  // Dynamic parameter callback
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dynamic_param_callback_handle_;

  // Interactive Marker Server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

  void initializeObstacles();
  void mainCycleCallback();
  void publishCycleCallback();
  void customObstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
  void clickedPointsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void viaPointsCallback(const nav_msgs::msg::Path::SharedPtr msg);
  // void setObstacleVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg, unsigned int id);
  void createInteractiveMarker(const ObstaclePtr& obstacle, unsigned int id);
  void interactiveMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
};

// Constructor — leave empty to avoid shared_from_this in constructor
TestOptimNode::TestOptimNode()
  : nav2_util::LifecycleNode("test_optim_node") {

}

// Separate init logic
void TestOptimNode::init()
{
  config.declareParameters(shared_from_this(), this->get_name());
  config.loadRosParamFromNodeHandle(shared_from_this(), this->get_name());
  dynamic_param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TebConfig::dynamicParametersCallback, std::ref(config), std::placeholders::_1));
  visual = std::make_shared<TebVisualization>(shared_from_this(), config);
  visual->on_configure();
  visual->on_activate();

  // Planner setup
  if (config.hcp.enable_homotopy_class_planning)
  {
    planner = std::make_shared<HomotopyClassPlanner>(
      shared_from_this(), config, &obst_vector, visual, &via_points);
  }
  else
  {
    planner = std::make_shared<TebOptimalPlanner>(
      shared_from_this(), config, &obst_vector, visual, &via_points);
  }

  planner->setVisualization(visual);
  initializeObstacles();

  marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_obstacles", shared_from_this());

  for (unsigned int i = 0; i < obst_vector.size(); ++i) {
    createInteractiveMarker(obst_vector[i], i);
  }

  marker_server_->applyChanges();

  main_cycle_timer = this->create_wall_timer(
    std::chrono::milliseconds(25),
    std::bind(&TestOptimNode::mainCycleCallback, this));

  publish_timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TestOptimNode::publishCycleCallback, this));

  custom_obst_sub = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
    "obstacles", 1,
    std::bind(&TestOptimNode::customObstacleCallback, this, std::placeholders::_1));

  clicked_points_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 5,
    std::bind(&TestOptimNode::clickedPointsCallback, this, std::placeholders::_1));

  via_points_sub = this->create_subscription<nav_msgs::msg::Path>(
    "via_points", 1,
    std::bind(&TestOptimNode::viaPointsCallback, this, std::placeholders::_1));
}

void TestOptimNode::initializeObstacles()
{
  obst_vector.push_back(std::make_shared<PointObstacle>(-3, 1));
  obst_vector.push_back(std::make_shared<PointObstacle>(6, 2));
  obst_vector.push_back(std::make_shared<PointObstacle>(0, 0.1));

  obst_vector[0]->setCentroidVelocity(Eigen::Vector2d(0.1, -0.3));
  obst_vector[1]->setCentroidVelocity(Eigen::Vector2d(-0.3, -0.2));

  no_fixed_obstacles = obst_vector.size();

  // for (unsigned int i = 0; i < obst_vector.size(); ++i)
  // {
  //   std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
  //   auto sub = this->create_subscription<geometry_msgs::msg::Twist>(
  //     topic, 1,
  //     [this, i](const geometry_msgs::msg::Twist::SharedPtr msg) {
  //       setObstacleVelocityCallback(msg, i);
  //     });
  //   obst_vel_subs.push_back(sub);
  // }
}

void TestOptimNode::mainCycleCallback()
{
  planner->plan(PoseSE2(-4, 0, 0), PoseSE2(4, 0, 0));
}

void TestOptimNode::publishCycleCallback()
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void TestOptimNode::customObstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  obst_vector.resize(no_fixed_obstacles);
  for (const auto& obs : msg->obstacles)
  {
    if (obs.polygon.points.empty()) continue;

    if (obs.polygon.points.size() == 1)
    {
      if (obs.radius == 0)
        obst_vector.push_back(std::make_shared<PointObstacle>(obs.polygon.points[0].x, obs.polygon.points[0].y));
      else
        obst_vector.push_back(std::make_shared<CircularObstacle>(obs.polygon.points[0].x, obs.polygon.points[0].y, obs.radius));
    }
    else
    {
      auto poly = new PolygonObstacle;
      for (const auto& pt : obs.polygon.points)
        poly->pushBackVertex(pt.x, pt.y);
      poly->finalizePolygon();
      obst_vector.emplace_back(poly);
    }

    if (!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obs.velocities, obs.orientation);
  }
}

void TestOptimNode::clickedPointsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  via_points.emplace_back(msg->point.x, msg->point.y);
  RCLCPP_INFO(this->get_logger(), "Via-point (%.2f, %.2f) added.", msg->point.x, msg->point.y);

  if (config.optim.weight_viapoint <= 0)
    RCLCPP_WARN(this->get_logger(), "Via-points are deactivated (weight_viapoint <= 0)");
}

void TestOptimNode::viaPointsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  via_points.clear();
  for (const auto& pose : msg->poses)
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
}

// void TestOptimNode::setObstacleVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg, unsigned int id)
// {
//   if (id >= obst_vector.size())
//   {
//     RCLCPP_WARN(this->get_logger(), "Cannot set velocity: unknown obstacle id.");
//     return;
//   }
//   obst_vector[id]->setCentroidVelocity(Eigen::Vector2d(msg->linear.x, msg->linear.y));
// }

void TestOptimNode::createInteractiveMarker(const ObstaclePtr& obstacle, unsigned int id) {
  visualization_msgs::msg::InteractiveMarker i_marker;
  i_marker.header.frame_id = "odom"; // Replace with your frame
  i_marker.header.stamp = this->now();
  i_marker.name = "obstacle_" + std::to_string(id);
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = obstacle->getCentroid().x();
  i_marker.pose.position.y = obstacle->getCentroid().y();
  i_marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);
  i_marker.controls.push_back(box_control);

  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.name = "move_xy";
  move_control.orientation.w = 1.0;
  move_control.orientation.x = 0;
  move_control.orientation.y = 1.0;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  i_marker.controls.push_back(move_control);

  marker_server_->insert(i_marker, std::bind(&TestOptimNode::interactiveMarkerCallback, this, std::placeholders::_1));
}

void TestOptimNode::interactiveMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  unsigned int id = std::stoi(feedback->marker_name.substr(feedback->marker_name.find_last_of('_') + 1));
  if (id >= obst_vector.size()) {
    RCLCPP_WARN(this->get_logger(), "Invalid obstacle ID in interactive marker feedback.");
    return;
  }

  auto point_obstacle = std::dynamic_pointer_cast<PointObstacle>(obst_vector[id]);
  if (point_obstacle) {
    point_obstacle->position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Updated obstacle %d position to (%.2f, %.2f)", id, feedback->pose.position.x, feedback->pose.position.y);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestOptimNode>();
  node->init();  // call init AFTER shared_ptr creation
  rclcpp::spin(node->get_node_base_interface());  // full lifecycle-aware spin
  rclcpp::shutdown();
  return 0;
}
