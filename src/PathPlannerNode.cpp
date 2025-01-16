#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "zed_msgs_uc/msg/cones.hpp"
#include "zed_msgs_uc/msg/cone.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "DetectBoundsAlgorithm.h"

#include <vector>

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{

  private:
    bool isValid;
    std::vector<Frame*> frames; 
  
  public:
    PathPlannerNode()
    : Node("pathplanner_node"), isValid(false)
    {
      odometry_Sub = this->create_subscription<nav_msgs::msg::Odometry>("/ukf_update_pose", 1, std::bind(&PathPlannerNode::odometryCallback, this, std::placeholders::_1));
      landmark_Sub = this->create_subscription<zed_msgs_uc::msg::Cones>("/zed2i/topic_bbox_zed3d", 1, std::bind(&PathPlannerNode::landmarkCallback, this, std::placeholders::_1));
    }

  private:

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      this->isValid = true;
      Frame* frame = new Frame();

      float x = msg->pose.pose.position.x;
      float y = msg->pose.pose.position.y;
      frame->veichlePosition = glm::vec2(x, y);

      float qx = msg->pose.pose.orientation.x;
      float qy = msg->pose.pose.orientation.y;
      float qz = msg->pose.pose.orientation.z;
      float qw = msg->pose.pose.orientation.w;

      glm::quat quaternion(qw, qx, qy, qz); // GLM usa (w, x, y, z)
      float theta = 2.0f * std::atan2(quaternion.z, quaternion.w);

      // Calcola il vettore direzione
      glm::vec2 direction(glm::cos(theta), glm::sin(theta));
      frame->veichleDirection = direction;

      frames.push_back(frame);

      RCLCPP_INFO(this->get_logger(), "pos = [%f, %f]\ndir = [%f, %f]",x, y, direction.x, direction.y);
    }

    void landmarkCallback(const zed_msgs_uc::msg::Cones::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "landmarkCallback");
      if(isValid)
      {
        isValid = false;

        Frame* frame = frames[frames.size()-1];

        const int blue_cones_size = msg->blue_cones.size();
        const int yellow_cones_size = msg->yellow_cones.size();
        
        frame->blueCones.resize(blue_cones_size);
        frame->yellowCones.resize(yellow_cones_size);

        for(int i = 0; i < blue_cones_size; i++)
        {
            frame->blueCones[i] = glm::vec2(msg->blue_cones[i].x, msg->blue_cones[i].y);
        }

        for(int i = 0; i < yellow_cones_size; i++)
        {
            frame->yellowCones[i] = glm::vec2(msg->yellow_cones[i].x, msg->yellow_cones[i].y);
        }

        std::vector<glm::vec2> punti_finali_left;
        std::vector<glm::vec2> punti_finali_right;

        begin_frame(frame->blueCones, frame->yellowCones, frame->veichlePosition, frame->veichleDirection, punti_finali_left, punti_finali_right);

        RCLCPP_INFO(this->get_logger(), "%d %d %d %d", blue_cones_size, (int)punti_finali_left.size(), yellow_cones_size, (int)punti_finali_right.size());
      }
      
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_Sub;
    rclcpp::Subscription<zed_msgs_uc::msg::Cones>::SharedPtr landmark_Sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
