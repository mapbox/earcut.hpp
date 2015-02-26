#pragma once

#include <vector>
#include <map>
#include <string>
#include <utility>

namespace mapbox {
namespace fixtures {

template <typename T> using Polygon = std::vector<std::vector<T>>;
template <typename T> using Triangles = std::vector<T>;

using IntegerPoint = std::pair<int, int>;
using IntegerTriangles = Triangles<IntegerPoint>;
using IntegerPolygon = Polygon<IntegerPoint>;

using DoublePoint = std::pair<double, double>;
using DoubleTriangles = Triangles<DoublePoint>;
using DoublePolygon = Polygon<DoublePoint>;


extern const IntegerPolygon bad_hole;
extern const IntegerPolygon building;
extern const IntegerPolygon degenerate;
extern const DoublePolygon dude;
extern const IntegerPolygon empty_square;
extern const IntegerPolygon water_huge;
extern const IntegerPolygon water_huge2;
extern const IntegerPolygon water;
extern const IntegerPolygon water2;
extern const IntegerPolygon water3;
extern const IntegerPolygon water3b;
extern const IntegerPolygon water4;

}
}
