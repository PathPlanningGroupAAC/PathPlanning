#include <iostream>
#include <iomanip>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

#include <vector>

#define PI 3.14159265359


float calculateAngle(const glm::vec2& point1, const glm::vec2& point2) {
    
    const float dotProduct = glm::dot(point1, point2);
    
    
    const float magnitude1 = glm::length(point1);
    const float magnitude2 = glm::length(point2);

    
    if (magnitude1 == 0.0f || magnitude2 == 0.0f) {
        throw std::invalid_argument("Uno dei vettori ha lunghezza zero.");
    }

    
    float cosTheta = dotProduct / (magnitude1 * magnitude2);
    cosTheta = glm::clamp(cosTheta, -1.0f, 1.0f);

    return glm::acos(cosTheta);
}

/*

    Formato punti:
    std::vector<glm::vec2> left_points;
    std::vector<glm::vec2> right_points;



*/

int main()
{
    glm::vec2 A = glm::vec2(0,1);
    glm::vec2 B = glm::vec2(-1,0);

    float dist = glm::distance(A,B);
    std::cout << "dist = " << dist << '\n';

    float angle = calculateAngle(A,B);
    std::cout << "angle = " << angle << '\n';

    plt::plot({1,3,2,4});
    plt::show();
    return 0;
}