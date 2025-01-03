#pragma once

#include <vector>

#include "csv.h"

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#define WITHOUT_NUMPY
#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

void debug_plot(const std::vector<glm::vec2>& cones_blue, const std::vector<glm::vec2>& cones_yellow, const std::vector<glm::vec2>& punti_finali_left, const std::vector<glm::vec2>& punti_finali_right, glm::vec2 veichle_pos, glm::vec2 veichle_dir);

// blu in
// giallo out
void loadTrack(std::vector<glm::vec2>& points, const std::string& path);