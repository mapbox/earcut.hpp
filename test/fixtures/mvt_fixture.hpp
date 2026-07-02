#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "geometries.hpp"

namespace mapbox {
namespace fixtures {

// The MVT tile fixture (bench/tiles-fixture.bin in the JS repo): a realistic set of
// production polygons — mostly small/simple, integer coordinates — decoded from packed
// MVT geometry. Exercises exactly the integer-MVT path GL Native routes to earcut.
using MvtPoint = std::pair<int, int>;
using MvtPolygon = Polygon<MvtPoint>;

struct MvtFeature {
    MvtPolygon polygon;
    std::size_t vertexCount; // total vertices across all rings
    std::size_t holeCount;   // rings - 1
    int zoom;
};

// Decodes tiles-fixture.bin once and returns all polygon features (cached).
const std::vector<MvtFeature>& mvtFixture();

} // namespace fixtures
} // namespace mapbox
