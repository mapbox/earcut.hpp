#include "comparison/earcut.hpp"
#include "comparison/libtess2.hpp"
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

template <typename Vertices, typename Indices>
double trianglesArea(const Vertices &vertices, const Indices &indices) {
    double area = 0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        area += triangleArea(
            vertices[indices[i]],
            vertices[indices[i + 1]],
            vertices[indices[i + 2]]
        );
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

template <typename Coord, typename Polygon>
void areaTest(const char *name, const Polygon &polygon, double earcutDeviation = 0.000001, double libtess2Deviation = 0.000001) {
    Tap::Test t(name);

    const auto expectedArea = polygonArea(polygon);

    { // Earcut
        EarcutTesselator<Coord, Polygon> tesselator(polygon);
        tesselator.run();

        const auto &vertices = tesselator.vertices();
        const auto &indices = tesselator.indices();

        const auto area = trianglesArea(vertices, indices);
        const double deviation =
            (expectedArea == 0 && area == 0) ? 0 : std::abs(area - expectedArea) / expectedArea;

        t.ok(deviation <= earcutDeviation, std::string{ "earcut deviation " } + formatPercent(deviation) +
                                                " is less than " +
                                                formatPercent(earcutDeviation));
    }

    { // Libtess2
        Libtess2Tesselator<Coord, Polygon> tesselator(polygon);
        tesselator.run();

        const auto &vertices = tesselator.vertices();
        const auto &indices = tesselator.indices();

        const auto area = trianglesArea(vertices, indices);
        const double deviation =
            (expectedArea == 0 && area == 0) ? 0 : std::abs(area - expectedArea) / expectedArea;

        t.ok(deviation <= libtess2Deviation, std::string{ "libtess2 deviation " } + formatPercent(deviation) +
                                                " is less than " +
                                                formatPercent(libtess2Deviation));
    }
    t.end();
}

int main() {
    Tap tap;

    areaTest<int>("bad_hole", mapbox::fixtures::bad_hole, 0.042, 0.0022);
    areaTest<int>("building", mapbox::fixtures::building);
    areaTest<int>("degenerate", mapbox::fixtures::degenerate);
    areaTest<double>("dude", mapbox::fixtures::dude);
    // allow libtess2 failure on this by providing infinity.
    areaTest<int>("empty_square", mapbox::fixtures::empty_square, 0, std::numeric_limits<double>::infinity());
    areaTest<int>("water_huge", mapbox::fixtures::water_huge, 0.0015, 0.0002);
    areaTest<int>("water_huge2", mapbox::fixtures::water_huge2, 0.002, 0.00015);
    areaTest<int>("water", mapbox::fixtures::water, 0.0019, 0.00002);
    areaTest<int>("water2", mapbox::fixtures::water2);
    areaTest<int>("water3", mapbox::fixtures::water3);
    areaTest<int>("water3b", mapbox::fixtures::water3b);
    areaTest<int>("water4", mapbox::fixtures::water4);

    return 0;
}
