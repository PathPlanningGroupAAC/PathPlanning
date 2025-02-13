#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "zed_msgs/msg/cones.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "DetectBoundsAlgorithm.h"

#include <filesystem>
#include <fstream>

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
      points_map = this->create_publisher<std_msgs::msg::Float32MultiArray>("multi_points", 10);

      odometry_Sub = this->create_subscription<nav_msgs::msg::Odometry>("/ukf_update_pose", 1, std::bind(&PathPlannerNode::odometryCallback, this, std::placeholders::_1));
      landmark_Sub = this->create_subscription<zed_msgs::msg::Cones>("/zed2i/topic_bbox_zed3d", 1, std::bind(&PathPlannerNode::landmarkCallback, this, std::placeholders::_1));
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

      // Coversione da quat ad eulero
      tf2::Quaternion quat(qx, qy, qz, qw);
      tf2::Matrix3x3 mat(quat);

      double roll, pitch, yaw;
      ///mat.getRPY(roll, pitch, yaw);
      

      // Calcola il vettore direzione
      glm::vec2 direction(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y);
      frame->veichleDirection = direction;

      frames.push_back(frame);

      RCLCPP_INFO(this->get_logger(), "------------------ FRAME[%d] -------------------\n", frames.size()-1);
      RCLCPP_INFO(this->get_logger(), "pos = [%f, %f]\ndir = [%f, %f]",x, y, direction.x, direction.y);
    }

    void landmarkCallback(const zed_msgs::msg::Cones::SharedPtr msg)
    {
      std::vector<float> x, y;
      RCLCPP_INFO(this->get_logger(), "landmarkCallback");
      
      char separator = std::filesystem::path::preferred_separator;
      std::string absolute_in = std::filesystem::current_path().string() + separator + "m_in.csv";
      std::string absolute_out = std::filesystem::current_path().string() + separator + "m_out.csv";

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
          glm::vec2 cone = glm::vec2(msg->blue_cones[i].x, msg->blue_cones[i].y);
          std::cout << cone.x << "," << cone.y << '\n';   
        }

        std::cout <<  "###" << '\n';

        for(int i = 0; i < yellow_cones_size; i++)
        {
          glm::vec2 cone = glm::vec2(msg->yellow_cones[i].x, msg->yellow_cones[i].y);
          std::cout << cone.x << "," << cone.y << '\n';   
        }

        /*std::fstream f_in(absolute_in, std::fstream::in | std::fstream::out | std::fstream::app);
        for(int i = 0; i < blue_cones_size; i++)
        {
          glm::vec2 cone = frame->veichlePosition +glm::vec2(msg->blue_cones[i].x, msg->blue_cones[i].y);
          f_in << cone.x << "," << cone.y << '\n';   
        }
        
        std::fstream f_out(absolute_out, std::fstream::in | std::fstream::out | std::fstream::app);
        for(int i = 0; i < yellow_cones_size; i++)
        {
          glm::vec2 cone = frame->veichlePosition +glm::vec2(msg->yellow_cones[i].x, msg->yellow_cones[i].y);
          f_out << cone.x << "," << cone.y << '\n';
        }*/

        for(int i = 0; i < blue_cones_size; i++)
        {
            frame->blueCones[i] = glm::vec2(msg->blue_cones[i].x, msg->blue_cones[i].y);
            x.push_back(msg->blue_cones[i].x);
            y.push_back(msg->blue_cones[i].y);
        }

        for(int i = 0; i < yellow_cones_size; i++)
        {
            frame->yellowCones[i] = glm::vec2(msg->yellow_cones[i].x, msg->yellow_cones[i].y);
            x.push_back(msg->yellow_cones[i].x);
            y.push_back(msg->yellow_cones[i].y);
        }

        begin_frame(frame->blueCones, frame->yellowCones, frame->veichlePosition, frame->veichleDirection, frame->punti_finali_left, frame->punti_finali_right);

        RCLCPP_INFO(this->get_logger(), "END FRAME");

        // "Pulizia" del frame (rimozione di punti gia' misurati)
        if(frames.size() >= 2)
        {
            //remove_same_cones(*(frames[frames.size()-2]), *frame);
        }

        // Passaggio alla Delaunay
      }
      
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr points_map;

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
