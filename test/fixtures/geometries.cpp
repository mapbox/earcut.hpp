#include "geometries.hpp"

#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#define PICOJSON_USE_INT64
#include <picojson.h>

#ifndef EARCUT_FIXTURE_DIR
#error "EARCUT_FIXTURE_DIR must be defined (path to the fetched JS test/fixtures)"
#endif
#ifndef EARCUT_EXPECTED_JSON
#error "EARCUT_EXPECTED_JSON must be defined (path to the fetched JS test/expected.json)"
#endif

namespace mapbox {
namespace fixtures {
namespace {

picojson::value parseFile(const std::string& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("failed to open fixture file: " + path);
    std::stringstream buffer;
    buffer << stream.rdbuf();
    picojson::value value;
    const std::string err = picojson::parse(value, buffer.str());
    if (!err.empty()) throw std::runtime_error("failed to parse " + path + ": " + err);
    return value;
}

// A fixture JSON is an array of rings; each ring an array of [x, y] pairs.
DoublePolygon parsePolygon(const picojson::value& value) {
    DoublePolygon polygon;
    for (const auto& ringValue : value.get<picojson::array>()) {
        std::vector<DoublePoint> ring;
        for (const auto& pointValue : ringValue.get<picojson::array>()) {
            const auto& p = pointValue.get<picojson::array>();
            ring.emplace_back(p[0].get<double>(), p[1].get<double>());
        }
        polygon.push_back(std::move(ring));
    }
    return polygon;
}

std::map<std::string, double> parseNumberMap(const picojson::value& value) {
    std::map<std::string, double> result;
    if (!value.is<picojson::object>()) return result;
    for (const auto& entry : value.get<picojson::object>()) {
        result.emplace(entry.first, entry.second.get<double>());
    }
    return result;
}

std::vector<std::unique_ptr<FixtureTester>> loadFixtures() {
    const picojson::value expected = parseFile(EARCUT_EXPECTED_JSON);
    const auto triangles = parseNumberMap(expected.get("triangles"));
    const auto errors = parseNumberMap(expected.get("errors"));

    std::vector<std::unique_ptr<FixtureTester>> fixtures;
    for (const auto& entry : triangles) {
        const std::string& name = entry.first;
        const auto expectedTriangles = static_cast<std::size_t>(entry.second);

        const std::string path = std::string(EARCUT_FIXTURE_DIR) + "/" + name + ".json";
        std::ifstream probe(path, std::ios::binary);
        if (!probe) continue; // expected.json may list ids without a fixture file
        probe.close();

        DoublePolygon polygon = parsePolygon(parseFile(path));

        const auto errorIt = errors.find(name);
        // JS bakes a 1e-14 epsilon into the recorded deviation; match it.
        const double deviation = (errorIt != errors.end() ? errorIt->second : 0.0) + 1e-14;

        fixtures.push_back(std::make_unique<FixtureTester>(name, expectedTriangles, deviation, std::move(polygon)));
    }
    return fixtures;
}

} // namespace

std::vector<FixtureTester*>& FixtureTester::collection() {
    static std::vector<std::unique_ptr<FixtureTester>> owned = loadFixtures();
    static std::vector<FixtureTester*> pointers = [] {
        std::vector<FixtureTester*> result;
        for (const auto& fixture : owned) result.push_back(fixture.get());
        std::sort(result.begin(), result.end(), [](FixtureTester* a, FixtureTester* b) { return a->name < b->name; });
        return result;
    }();
    return pointers;
}

} // namespace fixtures
} // namespace mapbox
