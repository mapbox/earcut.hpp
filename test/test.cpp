#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

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

double trianglePerimeter(const std::vector<uint32_t>& triangles,
                         const std::vector<double>& vertices,
                         std::size_t dim = 2) {
    double perimeter = 0;
    for (std::size_t i = 0; i < triangles.size(); i += 3) {
        const double ax = vertices[triangles[i] * dim];
        const double ay = vertices[triangles[i] * dim + 1];
        const double bx = vertices[triangles[i + 1] * dim];
        const double by = vertices[triangles[i + 1] * dim + 1];
        const double cx = vertices[triangles[i + 2] * dim];
        const double cy = vertices[triangles[i + 2] * dim + 1];

        perimeter += std::hypot(ax - bx, ay - by) + std::hypot(bx - cx, by - cy) + std::hypot(cx - ax, cy - ay);
    }
    return perimeter;
}

double flatSignedArea(const std::vector<double>& vertices, std::size_t start, std::size_t end, std::size_t dim) {
    double sum = 0;
    for (std::size_t i = start, j = end - dim; i < end; j = i, i += dim) {
        sum += (vertices[j] - vertices[i]) * (vertices[i + 1] + vertices[j + 1]);
    }
    return sum;
}

double flatDeviation(const std::vector<double>& vertices,
                     const std::vector<std::size_t>& holes,
                     std::size_t dim,
                     const std::vector<uint32_t>& triangles) {
    const std::size_t outerLen = holes.empty() ? vertices.size() : holes[0] * dim;
    double polygonArea = std::abs(flatSignedArea(vertices, 0, outerLen, dim));
    for (std::size_t i = 0; i < holes.size(); i++) {
        const std::size_t start = holes[i] * dim;
        const std::size_t end = i < holes.size() - 1 ? holes[i + 1] * dim : vertices.size();
        polygonArea -= std::abs(flatSignedArea(vertices, start, end, dim));
    }

    double trianglesArea = 0;
    for (std::size_t i = 0; i < triangles.size(); i += 3) {
        const std::size_t a = triangles[i] * dim;
        const std::size_t b = triangles[i + 1] * dim;
        const std::size_t c = triangles[i + 2] * dim;
        trianglesArea += std::abs((vertices[a] - vertices[c]) * (vertices[b + 1] - vertices[a + 1]) -
                                  (vertices[a] - vertices[b]) * (vertices[c + 1] - vertices[a + 1]));
    }

    return polygonArea == 0 && trianglesArea == 0 ? 0 : std::abs((trianglesArea - polygonArea) / polygonArea);
}

class EarcutAreaTest : public ::testing::TestWithParam<mapbox::fixtures::FixtureTester*> {};

TEST_P(EarcutAreaTest, EarcutTriangulation) {
    auto fixture = GetParam();

    const auto expectedArea = polygonArea(fixture->polygon());
    const auto expectedTriangles = fixture->expectedTriangles;

    const auto earcut = fixture->earcut();
    const auto earcutTriangles = earcut.indices.size() / 3;

    // int64 triangulation may differ in triangle count from the double baseline
    if (fixture->name.find("_i64") == std::string::npos) {
        EXPECT_EQ(earcutTriangles, expectedTriangles)
            << fixture->name << ": " << earcutTriangles << " triangles when expected " << expectedTriangles;
    }

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

TEST_P(EarcutAreaTest, EarcutTriangulationWithRotations) {
    auto fixture = GetParam();

    const std::map<std::string, double> errorsWithRotation = {{"water_huge", 0.005},
                                                              {"water_huge2", 0.03},
                                                              {"bad_hole", 0.03},
                                                              {"issue16", 1e-15},
                                                              {"eberly_6", 3e-14},
                                                              {"self_touching", 1e-13}};

    for (const int rotation : {90, 180, 270}) {
        const double theta = rotation * 3.14159265358979323846 / 180.0;
        const int xx = static_cast<int>(std::round(std::cos(theta)));
        const int xy = static_cast<int>(std::round(-std::sin(theta)));
        const int yx = static_cast<int>(std::round(std::sin(theta)));
        const int yy = static_cast<int>(std::round(std::cos(theta)));

        mapbox::fixtures::DoublePolygon rotated;
        rotated.reserve(fixture->polygon().size());
        for (const auto& ring : fixture->polygon()) {
            std::vector<mapbox::fixtures::DoublePoint> rotatedRing;
            rotatedRing.reserve(ring.size());
            for (const auto& point : ring) {
                const auto x = std::get<0>(point);
                const auto y = std::get<1>(point);
                rotatedRing.emplace_back(xx * x + xy * y, yx * x + yy * y);
            }
            rotated.push_back(std::move(rotatedRing));
        }

        EarcutTesselator<double, mapbox::fixtures::DoublePolygon> tesselator(rotated);
        tesselator.run();

        const auto expectedArea = polygonArea(rotated);
        const auto area = trianglesArea(tesselator.vertices(), tesselator.indices());
        const double deviation = (expectedArea == area) ? 0
                                 : expectedArea == 0    ? std::numeric_limits<double>::infinity()
                                                        : std::abs(area - expectedArea) / expectedArea;
        const auto error = errorsWithRotation.find(fixture->name);
        const double expectedDeviation =
            error != errorsWithRotation.end() ? error->second : fixture->expectedEarcutDeviation;

        EXPECT_LE(deviation, expectedDeviation)
            << fixture->name << " rotation " << rotation << ": earcut deviation " << formatPercent(deviation)
            << " is not less than " << formatPercent(expectedDeviation);
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

TEST(EarcutBasicTest, Indices2D) {
    std::vector<std::vector<std::array<int, 2>>> polygon = {{{10, 0}, {0, 50}, {60, 60}, {70, 10}}};
    const auto indices = mapbox::earcut<uint32_t>(polygon);
    const std::vector<uint32_t> expected = {1, 0, 3, 1, 3, 2};
    EXPECT_EQ(indices, expected);
}

TEST(EarcutBasicTest, Indices3DUsesXYOnly) {
    std::vector<std::vector<std::array<int, 3>>> polygon = {{{10, 0, 0}, {0, 50, 0}, {60, 60, 0}, {70, 10, 0}}};
    const auto indices = mapbox::earcut<uint32_t>(polygon);
    const std::vector<uint32_t> expected = {1, 0, 3, 1, 3, 2};
    EXPECT_EQ(indices, expected);
}

TEST(EarcutBasicTest, InfiniteLoopRegression) {
    std::vector<std::vector<std::pair<int, int>>> polygon = {{{1, 2}, {2, 2}, {1, 2}, {1, 1}, {1, 2}},
                                                             {{4, 1}, {5, 1}, {3, 2}, {4, 2}, {4, 1}}};
    EXPECT_NO_FATAL_FAILURE(mapbox::earcut<uint32_t>(polygon));
}

TEST(EarcutBasicTest, BlockIndexCollinear) {
    const int n = 30;
    std::vector<std::pair<int, int>> outer;
    for (int x = 0; x <= n; x++) outer.emplace_back(x, 0);
    for (int y = 1; y <= n; y++) outer.emplace_back(n, y);
    for (int x = n - 1; x >= 0; x--) outer.emplace_back(x, n);
    for (int y = n - 1; y >= 1; y--) outer.emplace_back(0, y);

    auto rect = [](int x0, int y0, int w, int h) {
        return std::vector<std::pair<int, int>>{{x0, y0}, {x0, y0 + h}, {x0 + w, y0 + h}, {x0 + w, y0}};
    };

    std::vector<std::vector<std::pair<int, int>>> polygon = {outer, rect(5, 5, 2, 4), rect(2, 23, 1, 1)};
    EarcutTesselator<double, decltype(polygon)> tesselator(polygon);
    tesselator.run();

    const auto expectedArea = polygonArea(polygon);
    const auto area = trianglesArea(tesselator.vertices(), tesselator.indices());
    const double deviation = (expectedArea == area) ? 0 : std::abs(area - expectedArea) / expectedArea;
    EXPECT_LT(deviation, 1e-9);
}

TEST(EarcutRefineTest, ImprovesBadQuadDiagonal) {
    const std::vector<double> vertices = {0, 0, 3, 0, 10, 1, 0, 2};
    const std::vector<std::vector<std::array<double, 2>>> polygon = {{{0, 0}, {3, 0}, {10, 1}, {0, 2}}};
    std::vector<uint32_t> triangles = {2, 3, 0, 2, 0, 1};
    const double beforePerimeter = trianglePerimeter(triangles, vertices);
    mapbox::refine(triangles, polygon);
    const double afterPerimeter = trianglePerimeter(triangles, vertices);

    const std::vector<uint32_t> expected = {2, 3, 1, 3, 0, 1};
    EXPECT_EQ(triangles, expected);
    EXPECT_LT(afterPerimeter, beforePerimeter * 0.7);
    EXPECT_EQ(flatDeviation(vertices, {}, 2, triangles), 0);
}

TEST(EarcutRefineTest, LeavesGoodQuadDiagonalAlone) {
    const std::vector<double> vertices = {0, 0, 5, 0, 4, 1, 0, 4};
    const std::vector<std::vector<std::array<double, 2>>> polygon = {{{0, 0}, {5, 0}, {4, 1}, {0, 4}}};
    std::vector<uint32_t> triangles = {2, 3, 0, 2, 0, 1};
    mapbox::refine(triangles, polygon);

    const std::vector<uint32_t> expected = {2, 3, 0, 2, 0, 1};
    EXPECT_EQ(triangles, expected);
    EXPECT_EQ(flatDeviation(vertices, {}, 2, triangles), 0);
}

TEST(EarcutRefineTest, PreservesConcavePolygon) {
    const std::vector<double> vertices = {0, 0, 4, 0, 4, 1, 1, 1, 1, 4, 0, 4};
    const std::vector<std::vector<std::array<double, 2>>> polygon = {{{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 4}, {0, 4}}};
    std::vector<uint32_t> triangles = {4, 5, 0, 0, 1, 2, 0, 2, 3, 0, 3, 4};
    const auto length = triangles.size();
    const double beforePerimeter = trianglePerimeter(triangles, vertices);
    mapbox::refine(triangles, polygon);
    const double afterPerimeter = trianglePerimeter(triangles, vertices);

    EXPECT_EQ(triangles.size(), length);
    EXPECT_LT(afterPerimeter, beforePerimeter * 0.9);
    EXPECT_EQ(flatDeviation(vertices, {}, 2, triangles), 0);
}

TEST(EarcutRefineTest, TerminatesOnNearCocircularPoints) {
    const std::vector<double> vertices = {127.65906365022843,
                                          9.336137742499535,
                                          124.21725103117963,
                                          30.888097161477972,
                                          91.35514946628345,
                                          89.65621376119454,
                                          40.10446780041529,
                                          121.5550560957686,
                                          -110.83205604043928,
                                          64.03323632184248,
                                          -127.20394987965459,
                                          -14.253249980770189,
                                          61.074962259031416,
                                          -112.48932831632469,
                                          127.37846573978545,
                                          -12.598669206638515,
                                          127.77010311801033,
                                          -7.668164657400608};
    std::vector<std::vector<std::array<double, 2>>> polygon(1);
    for (std::size_t i = 0; i < vertices.size(); i += 2) {
        polygon[0].push_back({vertices[i], vertices[i + 1]});
    }
    std::vector<uint32_t> triangles = {8, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 1, 1, 3, 5, 5, 7, 1};
    const auto length = triangles.size();
    mapbox::refine(triangles, polygon);

    EXPECT_EQ(triangles.size(), length);
    EXPECT_LT(flatDeviation(vertices, {}, 2, triangles), 1e-15);
}

static std::size_t countIllegalEdges(const std::vector<uint32_t>& triangles,
                                     const std::vector<double>& vertices,
                                     std::size_t dim = 2) {
    const auto n = triangles.size();
    std::vector<int32_t> halfEdges(n, -1);

    {
        std::map<uint64_t, std::size_t> edges;
        for (std::size_t e = 0; e < n; e++) {
            const std::size_t a = triangles[e];
            const std::size_t b = triangles[mapbox::detail::nextHalfEdge(e)];
            const uint64_t key = a < b ? (static_cast<uint64_t>(a) << 32) | b : (static_cast<uint64_t>(b) << 32) | a;
            const auto it = edges.find(key);
            if (it != edges.end()) {
                halfEdges[e] = static_cast<int32_t>(it->second);
                halfEdges[it->second] = static_cast<int32_t>(e);
                edges.erase(it);
            } else {
                edges[key] = e;
            }
        }
    }

    std::size_t illegal = 0;
    for (std::size_t a = 0; a < n; a++) {
        const std::size_t b = static_cast<std::size_t>(halfEdges[a]);
        if (b == static_cast<std::size_t>(-1) || a > b) continue;

        const std::size_t a0 = a - a % 3;
        const std::size_t b0 = b - b % 3;
        const std::size_t ar = a0 + (a + 2) % 3;
        const std::size_t al = a0 + (a + 1) % 3;
        const std::size_t bl = b0 + (b + 2) % 3;
        const std::size_t p0 = triangles[ar];
        const std::size_t pr = triangles[a];
        const std::size_t pl = triangles[al];
        const std::size_t p1 = triangles[bl];

        const double x0 = vertices[p0 * dim];
        const double y0 = vertices[p0 * dim + 1];
        const double xr = vertices[pr * dim];
        const double yr = vertices[pr * dim + 1];
        const double xl = vertices[pl * dim];
        const double yl = vertices[pl * dim + 1];
        const double x1 = vertices[p1 * dim];
        const double y1 = vertices[p1 * dim + 1];

        const bool convex =
            mapbox::detail::orient(x0, y0, xr, yr, x1, y1) > 0 && mapbox::detail::orient(x0, y0, x1, y1, xl, yl) > 0;
        if (convex && !mapbox::detail::inCircle(x0, y0, xr, yr, xl, yl, x1, y1)) illegal++;
    }
    return illegal;
}

TEST(EarcutRefineTest, LegalizesAllConvexInteriorEdgesInEarcutFixture) {
    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    mapbox::fixtures::FixtureTester* earcutFixture = nullptr;
    for (auto f : fixtures) {
        if (f->name == "earcut") {
            earcutFixture = f;
            break;
        }
    }
    ASSERT_NE(earcutFixture, nullptr);

    const auto result = earcutFixture->earcut();
    std::vector<uint32_t> triangles(result.indices.begin(), result.indices.end());

    std::vector<double> flatVertices;
    flatVertices.reserve(result.vertices.size() * 2);
    for (const auto& v : result.vertices) {
        flatVertices.push_back(v[0]);
        flatVertices.push_back(v[1]);
    }

    const auto& poly = earcutFixture->polygon();
    mapbox::refine(triangles, poly);

    EXPECT_EQ(countIllegalEdges(triangles, flatVertices), std::size_t{0});

    std::vector<std::size_t> holes;
    holes.push_back(poly[0].size());
    for (std::size_t i = 1; i < poly.size(); i++) holes.push_back(holes.back() + poly[i].size());
    holes.pop_back();
    EXPECT_EQ(flatDeviation(flatVertices, holes, 2, triangles), 0);
}

// --- MVT fixture reader ---

static uint32_t readVarint(const uint8_t*& buf, const uint8_t* end) {
    uint32_t val = 0;
    int shift = 0;
    while (buf < end) {
        const uint32_t b = *buf++;
        val |= (b & 0x7f) << shift;
        if (!(b & 0x80)) break;
        shift += 7;
    }
    return val;
}

static int32_t zigZagDecode(uint32_t n) {
    return static_cast<int32_t>((n >> 1) ^ -(n & 1));
}

static double ringAreaMvt(const std::vector<std::pair<int, int>>& ring) {
    double sum = 0;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        sum += (ring[j].first - ring[i].first) * double(ring[i].second + ring[j].second);
    }
    return sum / 2;
}

struct MvtPolygon {
    std::vector<double> vertices;
    std::vector<std::size_t> holes;
    std::size_t dimensions = 2;
};

static std::vector<MvtPolygon> readTilesFixture(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Cannot open " + path);
    const auto size = file.tellg();
    file.seekg(0);
    auto buf = std::make_unique<uint8_t[]>(static_cast<std::size_t>(size));
    file.read(reinterpret_cast<char*>(buf.get()), size);
    const uint8_t* pos = buf.get();
    const uint8_t* end = pos + static_cast<std::size_t>(size);

    std::vector<MvtPolygon> polys;

    while (pos < end) {
        readVarint(pos, end); // zoom, unused
        const auto features = readVarint(pos, end);

        for (uint32_t f = 0; f < features; f++) {
            const auto geomCount = readVarint(pos, end);
            std::vector<uint32_t> geom(geomCount);
            for (uint32_t i = 0; i < geomCount; i++) geom[i] = readVarint(pos, end);

            // decode MVT geometry to rings
            int x = 0, y = 0;
            using IntPoint = std::pair<int, int>;
            std::vector<std::vector<IntPoint>> rings;
            std::vector<IntPoint> ring;
            std::size_t g = 0;

            while (g < geom.size()) {
                const uint32_t cmd = geom[g] & 0x7;
                const uint32_t count = geom[g] >> 3;
                g++;

                if (cmd == 1) { // MoveTo
                    for (uint32_t k = 0; k < count; k++) {
                        x += zigZagDecode(geom[g++]);
                        y += zigZagDecode(geom[g++]);
                        if (!ring.empty()) rings.push_back(std::move(ring));
                        ring = {IntPoint{x, y}};
                    }
                } else if (cmd == 2) { // LineTo
                    for (uint32_t k = 0; k < count; k++) {
                        x += zigZagDecode(geom[g++]);
                        y += zigZagDecode(geom[g++]);
                        ring.emplace_back(x, y);
                    }
                } else if (cmd == 7) { // ClosePath
                    if (!ring.empty()) {
                        rings.push_back(std::move(ring));
                        ring.clear();
                    }
                }
            }
            if (!ring.empty()) rings.push_back(std::move(ring));

            // assemble multi-polygon parts into polys with holes
            std::vector<std::vector<IntPoint>> current;
            for (auto& r : rings) {
                if (r.size() < 3) continue;
                const double area = ringAreaMvt(r);
                if (area == 0) continue;
                if (area > 0) {
                    if (!current.empty()) {
                        MvtPolygon poly;
                        std::size_t vertIdx = 0;
                        for (const auto& ring2 : current) {
                            for (const auto& pt : ring2) {
                                poly.vertices.push_back(static_cast<double>(pt.first));
                                poly.vertices.push_back(static_cast<double>(pt.second));
                                vertIdx++;
                            }
                            if (&ring2 != &current.back()) poly.holes.push_back(vertIdx);
                        }
                        polys.push_back(std::move(poly));
                    }
                    current = {std::move(r)};
                } else {
                    current.push_back(std::move(r));
                }
            }
            if (!current.empty()) {
                MvtPolygon poly;
                std::size_t vertIdx = 0;
                for (const auto& ring2 : current) {
                    for (const auto& pt : ring2) {
                        poly.vertices.push_back(static_cast<double>(pt.first));
                        poly.vertices.push_back(static_cast<double>(pt.second));
                        vertIdx++;
                    }
                    if (&ring2 != &current.back()) poly.holes.push_back(vertIdx);
                }
                polys.push_back(std::move(poly));
            }
        }
    }

    return polys;
}

TEST(EarcutRefineTest, MvtFixtureHasZeroDeviationAndRefinedQuality) {
    const auto polys = readTilesFixture("test/tiles-fixture.bin");

    std::size_t nonzero = 0;
    int firstIndex = -1;
    double firstDev = 0;
    int worstIndex = -1;
    double worstDev = 0;
    double sumDev = 0;
    std::size_t refinedNonzero = 0;
    int refinedFirstIndex = -1;
    double refinedFirstDev = 0;
    int refinedWorstIndex = -1;
    double refinedWorstDev = 0;
    double refinedSumDev = 0;
    std::size_t lengthChanged = 0;
    double basePerimeter = 0;
    double refinedPerimeter = 0;

    for (std::size_t i = 0; i < polys.size(); i++) {
        const auto& poly = polys[i];
        using DoubleRing = std::vector<std::array<double, 2>>;
        std::vector<DoubleRing> rings;
        std::size_t offset = 0;
        for (std::size_t h = 0; h <= poly.holes.size(); h++) {
            const std::size_t next = h < poly.holes.size() ? poly.holes[h] : poly.vertices.size() / poly.dimensions;
            DoubleRing ring;
            for (std::size_t j = offset; j < next; j++) {
                ring.push_back({poly.vertices[j * 2], poly.vertices[j * 2 + 1]});
            }
            rings.push_back(std::move(ring));
            offset = next;
        }
        std::vector<uint32_t> triangles = mapbox::earcut<uint32_t>(rings);
        const auto length = triangles.size();
        basePerimeter += trianglePerimeter(triangles, poly.vertices, poly.dimensions);
        const double dev = flatDeviation(poly.vertices, poly.holes, poly.dimensions, triangles);
        if (dev != 0) {
            if (firstIndex < 0) {
                firstIndex = static_cast<int>(i);
                firstDev = dev;
            }
            nonzero++;
            sumDev += dev;
            if (dev > worstDev) {
                worstIndex = static_cast<int>(i);
                worstDev = dev;
            }
        }

        mapbox::refine(triangles, rings);
        refinedPerimeter += trianglePerimeter(triangles, poly.vertices, poly.dimensions);
        if (triangles.size() != length) lengthChanged++;

        const double refinedDev = flatDeviation(poly.vertices, poly.holes, poly.dimensions, triangles);
        if (refinedDev != 0) {
            if (refinedFirstIndex < 0) {
                refinedFirstIndex = static_cast<int>(i);
                refinedFirstDev = refinedDev;
            }
            refinedNonzero++;
            refinedSumDev += refinedDev;
            if (refinedDev > refinedWorstDev) {
                refinedWorstIndex = static_cast<int>(i);
                refinedWorstDev = refinedDev;
            }
        }
    }

    EXPECT_EQ(polys.size(), std::size_t{119680});
    EXPECT_EQ(nonzero, std::size_t{0}) << nonzero << " polygons with nonzero deviation; first " << firstIndex << ": "
                                       << firstDev << ", worst " << worstIndex << ": " << worstDev << ", sum "
                                       << sumDev;

    EXPECT_EQ(lengthChanged, std::size_t{0}) << lengthChanged << " refined triangulations changed triangle count";
    EXPECT_EQ(refinedNonzero, std::size_t{0})
        << refinedNonzero << " refined polygons with nonzero deviation; first " << refinedFirstIndex << ": "
        << refinedFirstDev << ", worst " << refinedWorstIndex << ": " << refinedWorstDev << ", sum " << refinedSumDev;

    EXPECT_LT(refinedPerimeter, basePerimeter * 0.72)
        << "refined perimeter ratio " << (refinedPerimeter / basePerimeter) << " < 0.72";
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
