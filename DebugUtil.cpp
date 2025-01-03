#include "DebugUtil.h"

#include <filesystem>

void debug_plot(const std::vector<glm::vec2>& cones_blue, const std::vector<glm::vec2>& cones_yellow, const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right, glm::vec2 veichle_pos, glm::vec2 veichle_dir)
{
    std::vector<float> x, y, u, v;
    x.push_back(veichle_pos.x);
    y.push_back(veichle_pos.y);
    u.push_back(veichle_dir.x);
    v.push_back(veichle_dir.y);
    // Draw Veichle
    plt::quiver(x,y,u,v, {{"color","red"}});

    // Draw Track
    std::vector<float> x_valsb, y_valsb;
    for (glm::vec2 e : cones_blue)
    {
        x_valsb.push_back(e.x);
        y_valsb.push_back(e.y);
    }

    plt::scatter(x_valsb, y_valsb, 10.0, { {"color", "blue"}, {"marker", "o"} });

    std::vector<float> x_valsy, y_valsy;
    for (glm::vec2 e : cones_yellow)
    {
        x_valsy.push_back(e.x);
        y_valsy.push_back(e.y);
    }

    plt::scatter(x_valsy, y_valsy, 10.0, { {"color", "gold"}, {"marker", "o"} });

    // Draw Final
    std::vector<float> plx, ply;
    for (glm::vec2 e : punti_finali_left)
    {
        plx.push_back(e.x);
        ply.push_back(e.y);
    }

    plt::plot(plx, ply, { {"color", "cyan"} });

    std::vector<float> prx, pry;
    for (glm::vec2 e : punti_finali_right)
    {
        prx.push_back(e.x);
        pry.push_back(e.y);
    }

    plt::plot(prx, pry, { {"color", "yellow"} });

    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Track");

    plt::show();
}

// blu in
// giallo out
void loadTrack(std::vector<glm::vec2>& points, const std::string& path)
{
    std::string absolute = std::filesystem::current_path().string() + std::filesystem::path::preferred_separator + path;
    try {
        io::CSVReader<2> in(absolute);

        double x, y;
        while (in.read_row(x, y))
        {
            points.push_back(glm::vec2((float)x, (float)y));
        }
    }
    catch (...)
    {
        std::cout << "File error\n";
    }
}
