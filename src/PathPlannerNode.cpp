#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "zed_msgs/msg/cones.hpp"
#include "zed_msgs/msg/cone.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{
  
  public:
    PathPlannerNode()
    : Node("pathplanner_node"), count_(0), point(glm::vec2(1,0))
    {
      odometry_Sub = this->create_subscription<nav_msgs::msg::Odometry>("/ukf_update_pose", 1, std::bind(&PathPlannerNode::odometryCallback, this, std::placeholders::_1));
      landmark_Sub = this->create_subscription<zed_msgs::msg::Cones>("/zed2i/topic_bbox_zed3d", 1, std::bind(&PathPlannerNode::landmarkCallback, this, std::placeholders::_1));
    }

  private:

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      float x = msg->pose.pose.position.x;
      float y = msg->pose.pose.position.y;

      float qx = msg->pose.pose.orientation.x;
      float qy = msg->pose.pose.orientation.y;
      float qz = msg->pose.pose.orientation.z;
      float qw = msg->pose.pose.orientation.w;

      glm::quat quaternion(qw, qx, qy, qz); // GLM usa (w, x, y, z)
      glm::vec3 axis = glm::axis(quaternion);

      float theta = axis.z;

      // Calcola il vettore direzione
      glm::vec2 direction(glm::cos(theta), glm::sin(theta));

      RCLCPP_INFO(this->get_logger(), "pos = [%f, %f]\ndir = [%f, %f]",x, y, direction.x, direction.y);
    }

    void landmarkCallback(const zed_msgs::msg::Cones::SharedPtr msg)
    {
    
      int blue_cones = msg->blue_cones.size();
      int yellow_cones = msg->yellow_cones.size();

      RCLCPP_INFO(this->get_logger(), "Blue Cones Recived: %d\nYellow Cones Recived:%d",blue_cones, yellow_cones);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Sub;
    rclcpp::Subscription<zed_msgs::msg::Cones>::SharedPtr landmark_Sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
