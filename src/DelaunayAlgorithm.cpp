#include "DelaunayAlgorithm.h"
#include "DetectBoundsAlgorithm.h"
#include "DelaunayFromScratch.h"

#include <fstream>
#include <unordered_set>
#include "ChadNet.h"

std::vector<glm::vec2> DelaunayAlgorithm::punti_finali_left;
std::vector<glm::vec2> DelaunayAlgorithm::punti_finali_right;

std::unordered_set<Vertex> DelaunayAlgorithm::left_p;
std::unordered_set<Vertex> DelaunayAlgorithm::right_p;

std::ofstream DelaunayAlgorithm::File;

bool DelaunayAlgorithm::initialized;

DelaunayAlgorithm::DelaunayAlgorithm(rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_waypoints_,
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_spline_points_,
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr& publisher_filtered_cones_)
{
    this->publisher_waypoints_ = publisher_waypoints_;
    this->publisher_spline_points_ = publisher_spline_points_;
    this->publisher_filtered_cones_ = publisher_filtered_cones_;

    if(!ChadConnect("192.168.1.8", CHAD_DEFAULT_PORT))
    {
        std::cout << "Error: Unable to connect to ChadPlotter!\n";
    }

    initialized = false;
}

void DelaunayAlgorithm::timer_callback(const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right)
{

    for (const auto &cone : punti_finali_left)
    {
        //if (std::abs(cone.y) < filter_param_y_ && std::sqrt(cone.x * cone.x + cone.y * cone.y) < filter_param_distance_)
        {

            xBlue.push_back(cone.x);
            yBlue.push_back(cone.y);
        }
    }

    for (const auto &cone : punti_finali_right)
    {
        //if (std::abs(cone.y) < filter_param_y_ && std::sqrt(cone.x * cone.x + cone.y * cone.y) < filter_param_distance_)
        {
            xYellow.push_back(cone.x);
            yYellow.push_back(cone.y);
        }
    }

    // Pubblica i coni filtrati
    /*
    for (size_t i = 0; i < xLittleO.size(); ++i)
    {
        zed_msgs::msg::Cone little_orange_cone;
        
        little_orange_cone.cone_type = 3;
        little_orange_cone.x = xLittleO[i];
        little_orange_cone.y = yLittleO[i];
        little_orange_cone.z = -1.3;
        filtered_cones.little_orange_cones.push_back(little_orange_cone);
    }

    for (size_t i = 0; i < xBigO.size(); ++i)
    {
        zed_msgs::msg::Cone big_orange_cone;
        big_orange_cone.cone_type = 1;
        big_orange_cone.x = xBigO[i];
        big_orange_cone.y = yBigO[i];
        big_orange_cone.z = -1.3;
        filtered_cones.big_orange_cones.push_back(big_orange_cone);
    }
    */

    for (size_t i = 0; i < xBlue.size(); ++i)
    {
        {
            zed_msgs::msg::Cone blue_cone;
            blue_cone.cone_type = 2;
            blue_cone.x = xBlue[i];
            blue_cone.y = yBlue[i];
            blue_cone.z = -1.3;
            filtered_cones.blue_cones.push_back(blue_cone);
        }

        for (size_t i = 0; i < xYellow.size(); ++i)
        {
            zed_msgs::msg::Cone yellow_cone;
            yellow_cone.cone_type = 4;
            yellow_cone.x = xYellow[i];
            yellow_cone.y = yYellow[i];
            yellow_cone.z = -1.3;
            filtered_cones.yellow_cones.push_back(yellow_cone);
        }
    }

    //caso base delaunay ovvero almeno tre coni di diverso colore
    if((xBlue.size() > 0 && xYellow.size() >0) && (xBlue.size() + xYellow.size() > 2))
    {
        if((xBlue.size()>1) || (xYellow.size()>1))
        {
            delaunayCalculation(punti_finali_left, punti_finali_right);
        }
    }

    //richiamo funzione spline
    spline(max_spline_degree_, Waypoints);

    // Pubblica i coni filtrati
    publish_waypoints(Waypoints);
    publisher_filtered_cones_->publish(filtered_cones);

}


void DelaunayAlgorithm::delaunayCalculation(const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right) {
    
    DelaunayAlgorithm::punti_finali_left.clear();
    DelaunayAlgorithm::punti_finali_right.clear();
    DelaunayAlgorithm::left_p.clear();
    DelaunayAlgorithm::right_p.clear();
    DelaunayAlgorithm::punti_finali_left = punti_finali_left;
    DelaunayAlgorithm::punti_finali_right = punti_finali_right;


    std::vector<Vertex> points;
    for(auto& p : punti_finali_left)
    {
        points.push_back({p.x, p.y});
        DelaunayAlgorithm::left_p.insert({p.x, p.y});
    }

    for(auto& p : punti_finali_right)
    {
        points.push_back({p.x, p.y});
        DelaunayAlgorithm::right_p.insert({p.x, p.y});
    }
    
    // Delaunay
    std::vector<Triangle> triangles = delaunay(points);

    for (auto& t : triangles)
    {
        bool existsP1 = false;
        bool existsP2 = false;
        bool existsP3 = false;

        std::unordered_set<Vertex>::const_iterator gotP1Left = left_p.find(t.p1);
        std::unordered_set<Vertex>::const_iterator gotP1Right = right_p.find(t.p1);
        if (!(Exists(gotP1Left, left_p) && Exists(gotP1Right, right_p)))
        {
            existsP1 = true;
        }

        std::unordered_set<Vertex>::const_iterator gotP2Left = left_p.find(t.p2);
        std::unordered_set<Vertex>::const_iterator gotP2Right = right_p.find(t.p2);
        if (!(Exists(gotP2Left, left_p) && Exists(gotP2Right, right_p)))
        {
            existsP2 = true;
        }

        std::unordered_set<Vertex>::const_iterator gotP3Left = left_p.find(t.p3);
        std::unordered_set<Vertex>::const_iterator gotP3Right = right_p.find(t.p3);
        if (!(Exists(gotP3Left, left_p) && Exists(gotP3Right, right_p)))
        {
            existsP3 = true;
        }

        if (existsP1 && existsP2)
        {
            if (DifferentLane(gotP1Right, right_p, gotP2Left, left_p) || DifferentLane(gotP1Left, left_p, gotP2Right, right_p))
            {
                double midX = (t.p1.x + t.p2.x) / 2.0;
                double midY = (t.p1.y + t.p2.y) / 2.0;
                Waypoints.push_back({midX, midY});
            }
        }
        else if (existsP2 && existsP3)
        {
            if (DifferentLane(gotP2Right, right_p, gotP3Left, left_p)|| DifferentLane(gotP2Left, left_p, gotP3Right, right_p))
            {
                double midX = (t.p2.x + t.p3.x) / 2.0;
                double midY = (t.p2.y + t.p3.y) / 2.0;
                Waypoints.push_back({ midX, midY });
            }
        }
        else if (existsP3 && existsP1)
        {
            if (DifferentLane(gotP3Right, right_p, gotP1Left, left_p)|| DifferentLane(gotP3Left, left_p, gotP1Right, right_p))
            {
                double midX = (t.p3.x + t.p1.x) / 2.0;
                double midY = (t.p3.y + t.p1.y) / 2.0;
                Waypoints.push_back({ midX, midY });
            }
        }


        ChadSendLine(t.p1.x, t.p1.y, t.p2.x, t.p2.y);
        ChadSendLine(t.p2.x, t.p2.y, t.p3.x, t.p3.y);
        ChadSendLine(t.p3.x, t.p3.y, t.p1.x, t.p1.y);
    }
    
    if(initialized == false) initialized = true;
}

void DelaunayAlgorithm::spline(const int max_spline_degree_, const  std::vector<Vertex>Waypoints) {
    // DEBUG ONLY grado
    if (max_spline_degree_ != 2)
        throw std::invalid_argument("Grado non valido");

    // Estrai coordinate x e y
    std::vector<double> x, y;
    for (const auto& p : Waypoints) {
        x.push_back(p.x);
        y.push_back(p.y);
    }

    // Calcolo spline quadratiche
    std::vector<double> spline_x, spline_y;
    spapi(max_spline_degree_, x, spline_x);
    spapi(max_spline_degree_, y, spline_y);

    // Valutazione spline per 'max_points' punti.
    std::vector<glm::vec2> final_spline;
    const float max_points = 200; // TODO: In YAML
    const float step = 1/max_points;

    for(float i = 0; i < max_points; i+=step)
    {
        final_spline.push_back(
            glm::vec2(
                fnval(spline_x, i), // x
                fnval(spline_y, i)  // y
            )
        );  
    }

    publish_spline_points(final_spline);
}

void DelaunayAlgorithm::publish_spline_points(const std::vector<glm::vec2>& final_spline)
{
    // Create the message
    auto msg = control_msgs::msg::WaypointArrayStamped();
    
    // Loop over the spline points and fill the message
    for (size_t i = 0; i < final_spline.size(); ++i)
    {
        auto spline_point = control_msgs::msg::Waypoint();
        spline_point.position.x = final_spline[i].x;
        spline_point.position.y = final_spline[i].y;
        msg.waypoints.push_back(spline_point);
    }
    
    // Publish the message
    publisher_spline_points_->publish(msg);
}

void DelaunayAlgorithm::publish_waypoints(const std::vector<Vertex> Waypoints)
{
    auto msg = control_msgs::msg::WaypointArrayStamped();

    for(size_t i = 0; i < Waypoints.size(); ++i)
    {
        auto waypoint = control_msgs::msg::Waypoint();
        waypoint.position.x = Waypoints[i].x;
        waypoint.position.y = Waypoints[i].y;
        ChadSendPoint((float)waypoint.position.x, (float)waypoint.position.y);
        msg.waypoints.push_back(waypoint); 
    }

    publisher_waypoints_->publish(msg);
}