#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <locale>
#include <map>
#include <mapbox/earcut.hpp>
#include <sstream>

#include "fixtures/geometries.hpp"
#include "fixtures/mvt_fixture.hpp"

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

TEST(EarcutBasicTest, EmptyInput) {
    auto polygon = mapbox::fixtures::Polygon<std::pair<int, int>>{};
    EarcutTesselator<int, decltype(polygon)> tesselator(polygon);
    tesselator.run();
    EXPECT_TRUE(tesselator.indices().empty()) << "empty input should produce empty result";
}

// The old codegen picked short/int/double per fixture to exercise the templates for integer
// point types. The runtime loader uses double throughout, so cover the integer paths explicitly
// here: a square with a square hole triangulates to 8 triangles with zero area deviation.
template <typename T>
void checkSquareWithHole() {
    mapbox::fixtures::Polygon<std::pair<T, T>> polygon{
        {{0, 0}, {100, 0}, {100, 100}, {0, 100}},
        {{25, 25}, {75, 25}, {75, 75}, {25, 75}},
    };
    EarcutTesselator<T, decltype(polygon)> tesselator(polygon);
    tesselator.run();

    const auto& indices = tesselator.indices();
    EXPECT_EQ(indices.size() / 3, 8u);

    const double expectedArea = polygonArea(polygon);
    const double area = trianglesArea(tesselator.vertices(), indices);
    EXPECT_DOUBLE_EQ(area, expectedArea);
}

TEST(EarcutTypedInput, Short) {
    checkSquareWithHole<short>();
}
TEST(EarcutTypedInput, Int) {
    checkSquareWithHole<int>();
}
TEST(EarcutTypedInput, Double) {
    checkSquareWithHole<double>();
}

// A minimal container whose size() returns a *signed* type, mimicking Qt's QList (qsizetype).
// earcut is a template, so signedness warnings (-Wsign-compare / -Wsign-conversion) only fire for
// the container types it is actually instantiated with. Every other test uses std::vector, whose
// size() is unsigned, so this instantiation is what guards the signed-size container path against
// regressions like PR #119 — under the CI's -Werror build, a reintroduced mismatch fails the build.
template <typename T>
class SignedSizeList {
public:
    using value_type = T;
    using size_type = std::ptrdiff_t; // signed, like Qt's qsizetype

    SignedSizeList() = default;
    SignedSizeList(std::initializer_list<T> init) : data_(init) {}

    size_type size() const { return static_cast<size_type>(data_.size()); }
    bool empty() const { return data_.empty(); }
    const T& operator[](std::size_t i) const { return data_[i]; }

private:
    std::vector<T> data_;
};

TEST(EarcutTypedInput, SignedSizeContainer) {
    using Ring = SignedSizeList<std::pair<double, double>>;
    SignedSizeList<Ring> polygon{
        Ring{{0, 0}, {100, 0}, {100, 100}, {0, 100}},
        Ring{{25, 25}, {75, 25}, {75, 75}, {25, 75}},
    };
    const auto indices = mapbox::earcut<uint32_t>(polygon);
    EXPECT_EQ(indices.size() / 3, 8u);
}

// ---- refine() ------------------------------------------------------------------------------

template <typename Vertices, typename Indices>
double trianglePerimeter(const Vertices& verts, const Indices& indices) {
    using namespace mapbox::util;
    using Point = typename Vertices::value_type;
    double perimeter = 0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        const auto& a = verts[indices[i]];
        const auto& b = verts[indices[i + 1]];
        const auto& c = verts[indices[i + 2]];
        const double ax = nth<0, Point>::get(a), ay = nth<1, Point>::get(a);
        const double bx = nth<0, Point>::get(b), by = nth<1, Point>::get(b);
        const double cx = nth<0, Point>::get(c), cy = nth<1, Point>::get(c);
        perimeter += std::hypot(ax - bx, ay - by) + std::hypot(bx - cx, by - cy) + std::hypot(cx - ax, cy - ay);
    }
    return perimeter;
}

static double refOrient(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// Matches detail::Refiner::inCircle (sign-negated, cocircular-tie margin) so the test agrees with
// what refine() considers legal.
static bool refInCircle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) {
    const double dx = ax - px, dy = ay - py, ex = bx - px, ey = by - py, fx = cx - px, fy = cy - py;
    const double ap = dx * dx + dy * dy, bp = ex * ex + ey * ey, cp = fx * fx + fy * fy;
    const double s = ap + bp + cp;
    return dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx) <= 1e-13 * s * s;
}

// Port of the JS test's countIllegalEdges: number of convex interior edges still failing inCircle.
template <typename Vertices, typename Indices>
std::size_t countIllegalEdges(const Vertices& verts, const Indices& indices) {
    using namespace mapbox::util;
    using Point = typename Vertices::value_type;
    const std::size_t n = indices.size();
    auto nextHE = [](std::size_t e) { return e - e % 3 + (e + 1) % 3; };

    std::vector<std::ptrdiff_t> halfEdges(n, -1);
    std::map<std::pair<uint32_t, uint32_t>, std::size_t> edges;
    for (std::size_t e = 0; e < n; e++) {
        const uint32_t a = uint32_t(indices[e]), b = uint32_t(indices[nextHE(e)]);
        const auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
        auto it = edges.find(key);
        if (it != edges.end()) {
            halfEdges[e] = std::ptrdiff_t(it->second);
            halfEdges[it->second] = std::ptrdiff_t(e);
            edges.erase(it);
        } else {
            edges.emplace(key, e);
        }
    }

    auto X = [&](std::size_t p) { return double(nth<0, Point>::get(verts[p])); };
    auto Y = [&](std::size_t p) { return double(nth<1, Point>::get(verts[p])); };

    std::size_t illegal = 0;
    for (std::size_t a = 0; a < n; a++) {
        const std::ptrdiff_t bs = halfEdges[a];
        if (bs == -1 || std::ptrdiff_t(a) > bs) continue;
        const std::size_t b = std::size_t(bs);
        const std::size_t a0 = a - a % 3, b0 = b - b % 3;
        const std::size_t ar = a0 + (a + 2) % 3, al = a0 + (a + 1) % 3, bl = b0 + (b + 2) % 3;
        const std::size_t p0 = indices[ar], pr = indices[a], pl = indices[al], p1 = indices[bl];
        const bool convex = refOrient(X(p0), Y(p0), X(pr), Y(pr), X(p1), Y(p1)) > 0 &&
                            refOrient(X(p0), Y(p0), X(p1), Y(p1), X(pl), Y(pl)) > 0;
        if (convex && !refInCircle(X(p0), Y(p0), X(pr), Y(pr), X(pl), Y(pl), X(p1), Y(p1))) illegal++;
    }
    return illegal;
}

using RefinePoint = std::pair<double, double>;

TEST(EarcutRefine, ImprovesBadQuadDiagonal) {
    std::vector<RefinePoint> verts{{0, 0}, {3, 0}, {10, 1}, {0, 2}};
    std::vector<uint32_t> triangles{2, 3, 0, 2, 0, 1};
    const double before = trianglePerimeter(verts, triangles);
    mapbox::refine(triangles, verts);
    const double after = trianglePerimeter(verts, triangles);

    EXPECT_EQ(triangles, (std::vector<uint32_t>{2, 3, 1, 3, 0, 1}));
    EXPECT_LT(after, before * 0.7);
}

TEST(EarcutRefine, LeavesGoodQuadDiagonalAlone) {
    std::vector<RefinePoint> verts{{0, 0}, {5, 0}, {4, 1}, {0, 4}};
    std::vector<uint32_t> triangles{2, 3, 0, 2, 0, 1};
    mapbox::refine(triangles, verts);
    EXPECT_EQ(triangles, (std::vector<uint32_t>{2, 3, 0, 2, 0, 1}));
}

TEST(EarcutRefine, PreservesConcavePolygon) {
    mapbox::fixtures::Polygon<RefinePoint> poly{{{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 4}, {0, 4}}};
    std::vector<RefinePoint> verts(poly[0].begin(), poly[0].end());
    auto triangles = mapbox::earcut<uint32_t>(poly);
    const auto length = triangles.size();
    const double before = trianglePerimeter(verts, triangles);
    mapbox::refine(triangles, verts);
    const double after = trianglePerimeter(verts, triangles);

    EXPECT_EQ(triangles.size(), length);
    EXPECT_LT(after, before * 0.9);
    EXPECT_DOUBLE_EQ(trianglesArea(verts, triangles), polygonArea(poly));
}

// #205 regression: four near-cocircular points give the non-robust inCircle inconsistent signs for
// an edge and its flip; without the tie margin the Lawson cascade flips forever. Must terminate.
TEST(EarcutRefine, TerminatesOnNearCocircularPoints) {
    mapbox::fixtures::Polygon<RefinePoint> poly{{{127.65906365022843, 9.336137742499535},
                                                 {124.21725103117963, 30.888097161477972},
                                                 {91.35514946628345, 89.65621376119454},
                                                 {40.10446780041529, 121.5550560957686},
                                                 {-110.83205604043928, 64.03323632184248},
                                                 {-127.20394987965459, -14.253249980770189},
                                                 {61.074962259031416, -112.48932831632469},
                                                 {127.37846573978545, -12.598669206638515},
                                                 {127.77010311801033, -7.668164657400608}}};
    std::vector<RefinePoint> verts(poly[0].begin(), poly[0].end());
    auto triangles = mapbox::earcut<uint32_t>(poly);
    const auto length = triangles.size();
    mapbox::refine(triangles, verts);
    EXPECT_EQ(triangles.size(), length);
    const double expectedArea = polygonArea(poly);
    const double dev = std::abs(trianglesArea(verts, triangles) - expectedArea) / expectedArea;
    EXPECT_LT(dev, 1e-15);
}

TEST(EarcutRefine, LegalizesAllConvexInteriorEdgesInFixture) {
    mapbox::fixtures::FixtureTester* fixture = nullptr;
    for (auto* f : mapbox::fixtures::FixtureTester::collection())
        if (f->name == "earcut") {
            fixture = f;
            break;
        }
    ASSERT_NE(fixture, nullptr) << "earcut.json fixture not found";

    const auto& poly = fixture->polygon();
    std::vector<mapbox::fixtures::DoublePoint> verts;
    for (const auto& ring : poly)
        for (const auto& p : ring) verts.push_back(p);

    auto triangles = mapbox::earcut<uint32_t>(poly);
    mapbox::refine(triangles, verts);
    EXPECT_EQ(countIllegalEdges(verts, triangles), 0u);
    EXPECT_DOUBLE_EQ(trianglesArea(verts, triangles), polygonArea(poly));
}

// The strongest correctness gate the project has: every MVT polygon must triangulate with area
// deviation exactly 0. Integer coords make the shoelace areas exact, so anything
// above rounding noise means a dropped/duplicated triangle. Also folds in refined quality:
// after refine(), deviation stays 0, triangle count is unchanged, and the aggregate refined
// perimeter drops below 0.72x the base — the mesh got measurably better without breaking.
TEST(EarcutTiles, ZeroDeviation) {
    const auto& features = mapbox::fixtures::mvtFixture();
    EXPECT_EQ(features.size(), 119680u) << "unexpected MVT polygon count";

    std::vector<mapbox::fixtures::MvtPoint> verts;
    std::size_t failures = 0, refinedFailures = 0, lengthChanged = 0;
    double worst = 0, refinedWorst = 0;
    double basePerimeter = 0, refinedPerimeter = 0;
    for (const auto& f : features) {
        verts.clear();
        for (const auto& ring : f.polygon)
            for (const auto& p : ring) verts.push_back(p);

        auto indices = mapbox::earcut<uint32_t>(f.polygon);
        const double expectedArea = polygonArea(f.polygon);
        if (expectedArea == 0) continue;
        const double area = trianglesArea(verts, indices);
        const double deviation = std::abs(area - expectedArea) / expectedArea;
        if (deviation > 1e-9) {
            ++failures;
            worst = std::max(worst, deviation);
        }

        const auto length = indices.size();
        basePerimeter += trianglePerimeter(verts, indices);
        mapbox::refine(indices, verts);
        refinedPerimeter += trianglePerimeter(verts, indices);
        if (indices.size() != length) ++lengthChanged;

        const double refinedArea = trianglesArea(verts, indices);
        const double refinedDeviation = std::abs(refinedArea - expectedArea) / expectedArea;
        if (refinedDeviation > 1e-9) {
            ++refinedFailures;
            refinedWorst = std::max(refinedWorst, refinedDeviation);
        }
    }
    EXPECT_EQ(failures, 0u) << failures << " polygons exceed zero-deviation (worst " << formatPercent(worst) << ")";
    EXPECT_EQ(lengthChanged, 0u) << lengthChanged << " refined triangulations changed triangle count";
    EXPECT_EQ(refinedFailures, 0u) << refinedFailures << " refined polygons exceed zero-deviation (worst "
                                   << formatPercent(refinedWorst) << ")";
    EXPECT_LT(refinedPerimeter, basePerimeter * 0.72)
        << "refined perimeter ratio " << (refinedPerimeter / basePerimeter) << " not < 0.72";
}

INSTANTIATE_TEST_SUITE_P(FixtureTests,
                         EarcutAreaTest,
                         ::testing::ValuesIn(mapbox::fixtures::FixtureTester::collection()),
                         [](const ::testing::TestParamInfo<mapbox::fixtures::FixtureTester*>& paramInfo) {
                             std::string name = paramInfo.param->name;
                             std::replace(name.begin(), name.end(), '-', '_');
                             std::replace(name.begin(), name.end(), '.', '_');
                             return name;
                         });
