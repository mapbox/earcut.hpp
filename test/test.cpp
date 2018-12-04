#include "tap.hpp"
#include "fixtures/geometries.hpp"

#include <iomanip>
#include <locale>
#include <sstream>

template <typename Point>
double triangleArea(const Point &a, const Point &b, const Point &c) {
    using namespace mapbox::util;
    return double(std::abs((nth<0, Point>::get(a) - nth<0, Point>::get(c)) * (nth<1, Point>::get(b) - nth<1, Point>::get(a)) -
                           (nth<0, Point>::get(a) - nth<0, Point>::get(b)) * (nth<1, Point>::get(c) - nth<1, Point>::get(a)))) / 2;
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

void areaTest(mapbox::fixtures::FixtureTester* fixture) {
    Tap::Test t(fixture->name);

    const auto expectedArea = polygonArea(fixture->polygon());
    const auto expectedTriangles = fixture->expectedTriangles;

    { // Earcut
        const auto earcut = fixture->earcut();

        const auto earcutTriangles = earcut.indices.size() / 3;
        t.ok(earcutTriangles == expectedTriangles, std::to_string(earcutTriangles) + " triangles when expected " +
            std::to_string(expectedTriangles));

        if (expectedTriangles > 0) {
            const auto area = trianglesArea(earcut.vertices, earcut.indices);
            const double deviation = (expectedArea == area) ? 0 :
                    expectedArea == 0 ? std::numeric_limits<double>::infinity() :
                    std::abs(area - expectedArea) / expectedArea;

            bool deviationOk = deviation <= fixture->expectedEarcutDeviation;
            t.ok(deviationOk, std::string{ "earcut deviation " } + formatPercent(deviation) +
                                                    " is " + (deviationOk ? "" : "not ") + "less than " +
                                                    formatPercent(fixture->expectedEarcutDeviation));
        }
    }

    { // Libtess2
        const auto libtess = fixture->libtess();
        const auto area = trianglesArea(libtess.vertices, libtess.indices);
        const double deviation = (expectedArea == area) ? 0 :
                expectedArea == 0 ? std::numeric_limits<double>::infinity() :
                std::abs(area - expectedArea) / expectedArea;

        bool deviationOk = deviation <= fixture->expectedLibtessDeviation;
        t.ok(deviationOk, std::string{ "libtess2 deviation " } + formatPercent(deviation) +
                                             " is " + (deviationOk ? "" : "not ") + "less than " +
                                             formatPercent(fixture->expectedLibtessDeviation));
    }

    t.end();
}

int main() {
    Tap tap;

    {
        Tap::Test t("empty");
        auto polygon = mapbox::fixtures::Polygon<std::pair<int, int>> {};
        EarcutTesselator<int, decltype(polygon)> tesselator(polygon);
        tesselator.run();
        t.ok(tesselator.indices().empty(), "empty input produces empty result");
        t.end();
    }

    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    for (auto fixture : fixtures) {
        areaTest(fixture);
    }

    return 0;
}
