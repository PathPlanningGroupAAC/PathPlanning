#pragma once

#include <vector>
#include <utility>
#include <glm/glm.hpp>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/waypoint_array_stamped.hpp>
#include <control_msgs/msg/ref_data.hpp>
#include <control_msgs/msg/waypoint.hpp>
#include <zed_msgs/msg/cones.hpp>
#include <zed_msgs/msg/cone.hpp>

#include "DelaunayFromScratch.h"

class DelaunayAlgorithm
{
public:
    
    DelaunayAlgorithm(rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_waypoints_,
        rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_spline_points_,
        rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr& publisher_filtered_cones_);

    void timer_callback(const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right);
    void delaunayCalculation(const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right);
    void spline(const int max_spline_degree_, const std::vector<Vertex>Waypoints);

    void publish_spline_points(const std::vector<glm::vec2>& final_spline);
    void publish_waypoints(const std::vector<Vertex> Waypoints);

    static std::vector<glm::vec2> punti_finali_left;
    static std::vector<glm::vec2> punti_finali_right;

    static std::unordered_set<Vertex> left_p;
    static std::unordered_set<Vertex> right_p;
    
    static std::ofstream File;
    
    static bool initialized;

    double filter_param_distance_;
    double filter_param_y_;
    int max_spline_degree_;
    
    std::vector<double> xBlue, yBlue, xYellow, yYellow, xBigO, yBigO, xLittleO, yLittleO;
    std::vector<Vertex> Waypoints;
    zed_msgs::msg::Cones filtered_cones;

    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr publisher_waypoints_        = NULL;
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr publisher_spline_points_    = NULL;
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr publisher_filtered_cones_                      = NULL;
};