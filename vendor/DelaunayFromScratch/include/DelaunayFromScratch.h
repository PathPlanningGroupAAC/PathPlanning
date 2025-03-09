#pragma once


#include <unordered_set>
#include <vector>
#include <utility>
#include <algorithm>
#include <functional>

struct Vertex 
{
    float x = 0, y = 0;

    Vertex() = default;
    Vertex(double x, double y) : x(x), y(y) {}

    bool operator==(const Vertex& other) const {
        return (x == other.x) && (y == other.y);
    }

    std::size_t operator()(const Vertex& v) const {
        std::size_t h1 = std::hash<float>{}(v.x);
        std::size_t h2 = std::hash<float>{}(v.y);
        return h1 ^ (h2 << 1); // XOR + shift per mescolare i bit
    }
};

namespace std {
    template <>
    struct hash<Vertex> {
        std::size_t operator()(const Vertex& v) const {
            std::size_t h1 = std::hash<float>{}(v.x);
            std::size_t h2 = std::hash<float>{}(v.y);
            return h1 ^ (h2 << 1); // XOR + shift per mescolare i bit
        }
    };
}

struct Triangle {
    Vertex p1, p2, p3;

    bool operator==(const Triangle& other) const {
        return (p1 == other.p1 && p2 == other.p2 && p3 == other.p3) ||
            (p1 == other.p2 && p2 == other.p3 && p3 == other.p1) ||
            (p1 == other.p3 && p2 == other.p1 && p3 == other.p2);
    }
};

bool Exists(std::unordered_set<Vertex>::const_iterator& iter, std::unordered_set<Vertex>& set);
bool DifferentLane(std::unordered_set<Vertex>::const_iterator& iter, std::unordered_set<Vertex>& set, 
    std::unordered_set<Vertex>::const_iterator& iter2, std::unordered_set<Vertex>& set2);

bool inCircumcircle(Vertex a, Vertex b, Vertex c, Vertex p);
bool ContainsSuperTrinagleVertex(Triangle t, Vertex p1, Vertex p2, Vertex p3);
std::vector<Triangle> delaunay(std::vector<Vertex>& points, Vertex p1 = { -1000, -1000 }, Vertex p2 = { 1000, -1000 }, Vertex p3 = { 0, 1000 });