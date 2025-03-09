#include "DelaunayFromScratch.h"

#include <vector>
#include <utility>
#include <algorithm>
#include <functional>

bool Exists(std::unordered_set<Vertex>::const_iterator& iter, std::unordered_set<Vertex>& set)
{
    return (iter != set.end());
}

bool DifferentLane(std::unordered_set<Vertex>::const_iterator& iter, std::unordered_set<Vertex>& set, 
    std::unordered_set<Vertex>::const_iterator& iter2, std::unordered_set<Vertex>& set2)
{
    return (Exists(iter, set) && Exists(iter2, set2));
}

bool inCircumcircle(Vertex a, Vertex b, Vertex c, Vertex p)
{
    double ax = a.x - p.x, ay = a.y - p.y;
    double bx = b.x - p.x, by = b.y - p.y;
    double cx = c.x - p.x, cy = c.y - p.y;

    double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
        (bx * bx + by * by) * (ax * cy - cx * ay) +
        (cx * cx + cy * cy) * (ax * by - bx * ay);

    return det > 0;
}

bool ContainsSuperTrinagleVertex(Triangle t, Vertex p1, Vertex p2, Vertex p3)
{
    // p1,p2,p3 are super triangle vertices
    return
        (t.p1.x == p1.x && t.p1.y == p1.y) ||
        (t.p2.x == p1.x && t.p2.y == p1.y) ||
        (t.p3.x == p1.x && t.p3.y == p1.y) ||
        (t.p1.x == p2.x && t.p1.y == p2.y) ||
        (t.p2.x == p2.x && t.p2.y == p2.y) ||
        (t.p3.x == p2.x && t.p3.y == p2.y) ||
        (t.p1.x == p3.x && t.p1.y == p3.y) ||
        (t.p2.x == p3.x && t.p2.y == p3.y) ||
        (t.p3.x == p3.x && t.p3.y == p3.y);
}

std::vector<Triangle> delaunay(std::vector<Vertex>& points, Vertex p1, Vertex p2, Vertex p3 )
{
    std::vector<Triangle> triangles;

    triangles.push_back({ p1 , p2, p3});

    for (const auto& p : points)
    {
        std::vector<Triangle> badTriangles;
        std::vector<std::pair<Vertex, Vertex>> edges;

        for (const auto& t : triangles)
        {
            if (inCircumcircle(t.p1, t.p2, t.p3, p))
            {
                badTriangles.push_back(t);
                edges.push_back({ t.p1, t.p2 });
                edges.push_back({ t.p2, t.p3 });
                edges.push_back({ t.p3, t.p1 });
            }
        }

        for (const auto& t : badTriangles)
        {
            triangles.erase(std::remove(triangles.begin(), triangles.end(), t), triangles.end());
        }

       for (size_t i = 0; i < edges.size(); i++)
        {
            for (size_t j = i + 1; j < edges.size(); j++)
            {
                if ((edges[i].first.x == edges[j].second.x && edges[i].first.y == edges[j].second.y &&
                    edges[i].second.x == edges[j].first.x && edges[i].second.y == edges[j].first.y) ||
                    (edges[i].first.x == edges[j].first.x && edges[i].first.y == edges[j].first.y &&
                        edges[i].second.x == edges[j].second.x && edges[i].second.y == edges[j].second.y))
                {
                    edges.erase(edges.begin() + j);
                    edges.erase(edges.begin() + i);
                    i--;
                    break;
                }
            }
        }

        for (const auto& edge : edges)
        {
            triangles.push_back({ edge.first, edge.second, p });
        }

    }

    triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
        [&](const Triangle& t) {
            return (t.p1.x == p1.x && t.p1.y == p1.y) ||
                (t.p2.x == p1.x && t.p2.y == p1.y) ||
                (t.p3.x == p1.x && t.p3.y == p1.y) ||
                (t.p1.x == p2.x && t.p1.y == p2.y) ||
                (t.p2.x == p2.x && t.p2.y == p2.y) ||
                (t.p3.x == p2.x && t.p3.y == p2.y) ||
                (t.p1.x == p3.x && t.p1.y == p3.y) ||
                (t.p2.x == p3.x && t.p2.y == p3.y) ||
                (t.p3.x == p3.x && t.p3.y == p3.y);
        }), triangles.end());

    return triangles;
}