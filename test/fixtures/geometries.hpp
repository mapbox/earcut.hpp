#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../comparison/earcut.hpp"

namespace mapbox {
namespace fixtures {
template <typename T>
using Polygon = std::vector<std::vector<T>>;
template <typename T>
using Triangles = std::vector<T>;
using DoublePoint = std::pair<double, double>;
using DoubleTriangles = Triangles<DoublePoint>;
using DoublePolygon = Polygon<DoublePoint>;

// A single test fixture. Unlike the old codegen, the polygon and its expected
// triangle count / deviation are loaded at runtime from the fetched JS repo's
// test/fixtures/*.json + test/expected.json (see geometries.cpp).
class FixtureTester {
public:
    struct TesselatorResult {
        std::vector<std::array<double, 2>> const& vertices;
        std::vector<uint32_t> const& indices;
    };

    FixtureTester(std::string testname, std::size_t triangles, double deviation, DoublePolygon poly)
        : name(std::move(testname)),
          expectedTriangles(triangles),
          expectedEarcutDeviation(deviation),
          poly_(std::move(poly)),
          earcutTesselator_(poly_) {}

    FixtureTester(const FixtureTester&) = delete;
    FixtureTester& operator=(const FixtureTester&) = delete;

    TesselatorResult earcut() {
        earcutTesselator_.run();
        return {earcutTesselator_.vertices(), earcutTesselator_.indices()};
    }
    DoublePolygon const& polygon() const { return poly_; }

    const std::string name;
    const std::size_t expectedTriangles;
    const double expectedEarcutDeviation;

    // Loads every fixture referenced by expected.json once; returns them sorted by name.
    static std::vector<FixtureTester*>& collection();

private:
    DoublePolygon poly_;
    EarcutTesselator<double, DoublePolygon> earcutTesselator_;
};

} // namespace fixtures
} // namespace mapbox
