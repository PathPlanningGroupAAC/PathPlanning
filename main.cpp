#include <iostream>

#include "DetectBoundsAlgorithm.h"
#include "DebugUtil.h"

int main(int argc, char* args[])
{
    std::vector<glm::vec2> cones_blue;
    std::vector<glm::vec2> cones_yellow;
    loadTrack(cones_blue, "m_in.csv");
    loadTrack(cones_yellow, "m_out.csv");

    glm::vec2 veichle_position = glm::vec2(22.0, 46.0);
    float angle = 0;
    glm::vec2 vettore_direzione = glm::vec2(cos(angle), sin(angle));
    
    std::vector<glm::vec2> punti_finali_left;
    std::vector<glm::vec2> punti_finali_right;

    begin_frame(cones_blue, cones_yellow, veichle_position, vettore_direzione, punti_finali_left, punti_finali_right);

    debug_plot(cones_blue, cones_yellow, punti_finali_left, punti_finali_right, veichle_position, vettore_direzione);
    return 0;
}