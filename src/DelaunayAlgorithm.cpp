#include "DelaunayAlgorithm.h"
#include "DetectBoundsAlgorithm.h"

#include <fstream>

DelaunayAlgorithm::DelaunayAlgorithm(rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_waypoints_,
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr& publisher_spline_points_,
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr& publisher_filtered_cones_)
{
    this->publisher_waypoints_ = publisher_waypoints_;
    this->publisher_spline_points_ = publisher_spline_points_;
    this->publisher_filtered_cones_ = publisher_filtered_cones_;
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
            delaunayCalculation();
        }
    }

    //richiamo funzione spline
    spline(max_spline_degree_, Waypoints);

    // Pubblica i coni filtrati
    publish_waypoints(Waypoints);
    publisher_filtered_cones_->publish(filtered_cones);

}

void DelaunayAlgorithm::delaunayCalculation() {
    std::vector<CustomPoint2D> points;
    // Itera su ogni "cone" in filtered_cones (di tipo zed_msgs::msg::Cones) per convertire filtered_cones in custompoint2D
    for (const auto& cone : filtered_cones.blue_cones) {  
        CustomPoint2D point;
        point.data[0] = cone.x;  
        point.data[1] = cone.y;  

        points.push_back(point);
    }
    for (const auto& cone : filtered_cones.yellow_cones) {  
        CustomPoint2D point;
        point.data[0] = cone.x;  
        point.data[1] = cone.y;  

        points.push_back(point);
    }
    /*
    for (const auto& cone : filtered_cones.little_orange_cones) {  
        CustomPoint2D point;
        point.data[0] = cone.x;  
        point.data[1] = cone.y;  

        points.push_back(point);
    }
    for (const auto& cone : filtered_cones.big_orange_cones) {  
        CustomPoint2D point;
        point.data[0] = cone.x;  
        point.data[1] = cone.y;  

        points.push_back(point);
    }
    */
    
    
    std::vector<CustomEdge> edges;
    CDT::Triangulation<float> cdt; 

    //converto i punti in vertici per poter rimpire array vertici ed eliminare duplicati
    std::vector<CDT::V2d<float>> pts;
    for (const auto& point : points) {
        pts.push_back({point.data[0], point.data[1]});
        }

    CDT::RemoveDuplicates<float>(pts);


    //triangolo
    cdt.insertVertices(pts);
    
    
    cdt.eraseSuperTriangle();

    CDT::EdgeUSet lati = CDT::extractEdgesFromTriangles(cdt.triangles);
    std::ofstream File;

    File.open("delaunay.txt");
    File<<pts.size()<< " 0\n";

    for (const auto& edge : lati) {
        File << pts[edge.v1()].x << " " << pts[edge.v1()].y << std::endl;
        File << pts[edge.v2()].x << " " << pts[edge.v2()].y << std::endl;
        }

    //calcolo waypoints (punto centrale edges)

    
        //if(lati){RCLCPP_INFO(PathPlannerNode::Instance->get_logger(), "ci sta");}
        
        //controllo che gli edgedes stanno nel vettore xblue o xyellow 
        if((!xBlue.empty() && !yBlue.empty()))
        {
            int foundBV1 = 0;
            int foundBV2 = 0;

            for(auto lato : lati){
                for(size_t i = 0; i < xBlue.size(); ++i)
                {
                    if(cdt.vertices[lato.v1()].x == xBlue[i])
                    {
                        for(size_t j = 0; j < yBlue.size(); ++j)
                        {
                                
                            if(cdt.vertices[lato.v1()].y == yBlue[j])
                            {
                                foundBV1 = 1;
                            }
                        }
                    }
                    lato.v1();

                }
                for(size_t i = 0; i < xBlue.size(); ++i)
                {
                    if(cdt.vertices[lato.v2()].x == xBlue[i])
                    {
                        for(size_t i = 0; i < yBlue.size(); ++i)
                        {
                            if(cdt.vertices[lato.v2()].y == yBlue[i])
                            {
                                foundBV2 = 1;
                            }
                        }
                    }
                    lato.v2();
                }
                //if(!foundBV1 != !foundBV2) //XOR 
                {
                    double midX = (cdt.vertices[lato.v1()].x  + cdt.vertices[lato.v2()].x) / 2.0;     
                    double midY = (cdt.vertices[lato.v1()].y + cdt.vertices[lato.v2()].y) / 2.0;
                    Waypoints.push_back({midX, midY});
                    
                }
            }
        }

}

void DelaunayAlgorithm::spline(const int max_spline_degree_, const  std::vector<CustomPoint2D>Waypoints) {
    // DEBUG ONLY grado
    if (max_spline_degree_ != 2)
        throw std::invalid_argument("Grado non valido");

    // Estrai coordinate x e y
    std::vector<double> x, y;
    for (const auto& p : Waypoints) {
        x.push_back(p.data[0]);
        y.push_back(p.data[1]);
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

void DelaunayAlgorithm::publish_waypoints(const std::vector<CustomPoint2D> Waypoints)
{
    auto msg = control_msgs::msg::WaypointArrayStamped();

    for(size_t i = 0; i < Waypoints.size(); ++i)
    {
        auto waypoint = control_msgs::msg::Waypoint();
        waypoint.position.x = Waypoints[i].data[0];
        waypoint.position.y = Waypoints[i].data[1];
        msg.waypoints.push_back(waypoint); 
    }

    publisher_waypoints_->publish(msg);
}