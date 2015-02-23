#include <earcut.hpp>
#include "tap.hpp"

#include "fixtures/geometries.hpp"

#include <cstdlib>
#include <cmath>

template <typename Point>
double triangleArea(const Point &a, const Point &b, const Point &c) {
    using namespace mapbox::util;
    return double(std::abs((nth<0, Point>::get(a) - nth<0, Point>::get(c)) *
                               (nth<1, Point>::get(b) - nth<1, Point>::get(a)) -
                           (nth<0, Point>::get(a) - nth<0, Point>::get(b)) *
                               (nth<1, Point>::get(c) - nth<1, Point>::get(a)))) /
           2;
}

template <typename Triangles>
double trianglesArea(const Triangles &triangles) {
    double area = 0;
    for (size_t i = 0; i < triangles.size(); i += 3) {
        area += triangleArea(triangles[i], triangles[i + 1], triangles[i + 2]);
    }
    return area;
}

template <typename Ring>
double ringArea(const Ring &points) {
    using namespace mapbox::util;
    using Point = typename Ring::value_type;
    double sum = 0;
    for (size_t i = 0, len = points.size(), j = len - 1; i < len; j = i++) {
        sum += (nth<0, Point>::get(points[i]) - nth<0, Point>::get(points[j])) *
               (nth<1, Point>::get(points[i]) + nth<1, Point>::get(points[j]));
    }
    return std::abs(sum) / 2;
}

template <typename Polygon>
double polygonArea(const Polygon &rings) {
    double sum = ringArea(rings[0]);
    for (size_t i = 1; i < rings.size(); i++) {
        sum -= ringArea(rings[i]);
    }
    return sum;
}

std::string formatPercent(double num) {
    return std::to_string(std::round(1e8 * num) / 1e6) + "%";
}

template <typename T>
void areaTest(const T &polygons, const std::string &name, double expectedDeviation = 0.000001) {
    Tap::Test t(name);

    const auto it = polygons.find(name);
    if (it == polygons.end()) {
        return t.fail(std::string { "Cannot find polygon with name " } + name);
    }
    const auto &data = it->second;
    mapbox::Earcut<typename std::remove_reference<typename T::mapped_type>::type> earcut;
    earcut(data);

    const auto expectedArea = polygonArea(data);
    const auto area = trianglesArea(earcut.triangles);

    const double deviation = (expectedArea == 0 && area == 0)
                                 ? 0
                                 : std::abs(area - expectedArea) / expectedArea;

    t.ok(deviation < expectedDeviation,
        std::string { "deviation " } + formatPercent(deviation) + " is less than " + formatPercent(expectedDeviation));

    t.end();
}

int main() {
    Tap::Start();

    areaTest(mapbox::fixtures::integerPolygons, "bad_hole", 0.0420);
    areaTest(mapbox::fixtures::integerPolygons, "building");
    areaTest(mapbox::fixtures::integerPolygons, "degenerate");
    areaTest(mapbox::fixtures::doublePolygons, "dude");
    areaTest(mapbox::fixtures::integerPolygons, "empty_square");
    areaTest(mapbox::fixtures::integerPolygons, "water_huge", 0.0015);
    areaTest(mapbox::fixtures::integerPolygons, "water_huge2", 0.0020);
    areaTest(mapbox::fixtures::integerPolygons, "water", 0.0019);
    areaTest(mapbox::fixtures::integerPolygons, "water2");
    areaTest(mapbox::fixtures::integerPolygons, "water3");
    areaTest(mapbox::fixtures::integerPolygons, "water3b");
    areaTest(mapbox::fixtures::integerPolygons, "water4");

    return 0;
}