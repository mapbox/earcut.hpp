#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>

#include "fixtures/geometries.hpp"

template <typename Point>
double triangleArea(const Point& a, const Point& b, const Point& c) {
    using namespace mapbox::util;
    return double(std::abs(
               (nth<0, Point>::get(a) - nth<0, Point>::get(c)) * (nth<1, Point>::get(b) - nth<1, Point>::get(a)) -
               (nth<0, Point>::get(a) - nth<0, Point>::get(b)) * (nth<1, Point>::get(c) - nth<1, Point>::get(a)))) /
           2;
}

template <typename Vertices, typename Indices>
double trianglesArea(const Vertices& vertices, const Indices& indices) {
    double area = 0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        area += triangleArea(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
    }
    return area;
}

template <typename Ring>
double ringArea(const Ring& points) {
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
double polygonArea(const Polygon& rings) {
    if (rings.empty()) return .0;
    double sum = ringArea(rings[0]);
    for (size_t i = 1; i < rings.size(); i++) {
        sum -= ringArea(rings[i]);
    }
    return std::max(sum, .0);
}

std::string formatPercent(double num) {
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << std::fixed << std::setprecision(6) << num * 100 << "%";
    return ss.str();
}

class EarcutAreaTest : public ::testing::TestWithParam<mapbox::fixtures::FixtureTester*> {};

TEST_P(EarcutAreaTest, EarcutTriangulation) {
    auto fixture = GetParam();

    const auto expectedArea = polygonArea(fixture->polygon());
    const auto expectedTriangles = fixture->expectedTriangles;

    const auto earcut = fixture->earcut();
    const auto earcutTriangles = earcut.indices.size() / 3;

    EXPECT_EQ(earcutTriangles, expectedTriangles)
        << fixture->name << ": " << earcutTriangles << " triangles when expected " << expectedTriangles;

    if (expectedTriangles > 0) {
        const auto area = trianglesArea(earcut.vertices, earcut.indices);
        const double deviation = (expectedArea == area) ? 0
                                 : expectedArea == 0    ? std::numeric_limits<double>::infinity()
                                                        : std::abs(area - expectedArea) / expectedArea;

        EXPECT_LE(deviation, fixture->expectedEarcutDeviation)
            << fixture->name << ": earcut deviation " << formatPercent(deviation) << " is not less than "
            << formatPercent(fixture->expectedEarcutDeviation);
    }
}

TEST_P(EarcutAreaTest, LibtessTriangulation) {
    auto fixture = GetParam();

    const auto expectedArea = polygonArea(fixture->polygon());
    const auto libtess = fixture->libtess();
    const auto area = trianglesArea(libtess.vertices, libtess.indices);
    const double deviation = (expectedArea == area) ? 0
                             : expectedArea == 0    ? std::numeric_limits<double>::infinity()
                                                    : std::abs(area - expectedArea) / expectedArea;

    EXPECT_LE(deviation, fixture->expectedLibtessDeviation)
        << fixture->name << ": libtess2 deviation " << formatPercent(deviation) << " is not less than "
        << formatPercent(fixture->expectedLibtessDeviation);
}

TEST(EarcutBasicTest, EmptyInput) {
    auto polygon = mapbox::fixtures::Polygon<std::pair<int, int>>{};
    EarcutTesselator<int, decltype(polygon)> tesselator(polygon);
    tesselator.run();
    EXPECT_TRUE(tesselator.indices().empty()) << "empty input should produce empty result";
}

INSTANTIATE_TEST_SUITE_P(FixtureTests,
                         EarcutAreaTest,
                         ::testing::ValuesIn(mapbox::fixtures::FixtureTester::collection()),
                         [](const ::testing::TestParamInfo<mapbox::fixtures::FixtureTester*>& info) {
                             std::string name = info.param->name;
                             std::replace(name.begin(), name.end(), '-', '_');
                             std::replace(name.begin(), name.end(), '.', '_');
                             return name;
                         });
