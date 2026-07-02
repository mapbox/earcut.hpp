#include <benchmark/benchmark.h>

#include <mapbox/earcut.hpp>
#include <set>
#include <vector>

#include "comparison/libtess2.hpp"
#include "fixtures/geometries.hpp"
#include "fixtures/mvt_fixture.hpp"

// Get benchmark fixtures - only those in the whitelist for performance reasons.
// These complement BM_EarcutTiles (clean, representative): the water_huge* fixtures are the only
// ones that exercise the self-intersection recovery path (cure/split), which the guaranteed-clean
// MVT set never hits. Tiny/degenerate fixtures are left out — they're noise as a perf signal.
std::vector<mapbox::fixtures::FixtureTester*> getBenchmarkFixtures() {
    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    std::set<std::string> bench_whitelist = {"building",
                                             "dude",
                                             "water_huge",
                                             "water_huge2",
                                             "water",
                                             "water2",
                                             "water3",
                                             "water3b",
                                             "water4"};

    std::vector<mapbox::fixtures::FixtureTester*> result;
    for (auto fixture : fixtures) {
        if (bench_whitelist.find(fixture->name) != bench_whitelist.end()) {
            result.push_back(fixture);
        }
    }
    return result;
}

// Benchmark earcut triangulation
static void BM_EarcutTriangulation(benchmark::State& state) {
    auto fixtures = getBenchmarkFixtures();
    auto fixture = fixtures[static_cast<size_t>(state.range(0))];

    for (auto _ : state) {
        auto result = fixture->earcut();
        benchmark::DoNotOptimize(result);
    }
    state.SetLabel(fixture->name);
}

// Benchmark libtess2 triangulation (comparison point only; built locally so libtess stays a
// benchmark-only dependency — see A4).
static void BM_LibtessTriangulation(benchmark::State& state) {
    auto fixtures = getBenchmarkFixtures();
    auto fixture = fixtures[static_cast<size_t>(state.range(0))];

    Libtess2Tesselator<double, mapbox::fixtures::DoublePolygon> tesselator(fixture->polygon());
    for (auto _ : state) {
        tesselator.run();
        benchmark::DoNotOptimize(tesselator.indices().data());
    }
    state.SetLabel(fixture->name);
}

// --- MVT tiles benchmark (the realistic production workload; see Step 10) ------------------
//
// The earcut-in-production regime is the small/simple bucket (GL Native routes larger/holey
// polygons to a monotone triangulator). Report the bench bucketed so the speedup story stays
// honest: the small bucket matters most for GL Native, the large bucket for the standalone lib.
enum class TileBucket { Small, Large, All };

static bool inBucket(const mapbox::fixtures::MvtFeature& f, TileBucket bucket) {
    // small/simple = fewer than 64 vertices OR fewer than 2 holes; large/holey = the rest
    const bool small = f.vertexCount < 64 || f.holeCount < 2;
    switch (bucket) {
        case TileBucket::Small:
            return small;
        case TileBucket::Large:
            return !small;
        case TileBucket::All:
            return true;
    }
    return true;
}

static void BM_EarcutTiles(benchmark::State& state, TileBucket bucket) {
    const auto& all = mapbox::fixtures::mvtFixture();

    std::vector<const mapbox::fixtures::MvtPolygon*> polys;
    std::size_t verts = 0;
    for (const auto& f : all) {
        if (inBucket(f, bucket)) {
            polys.push_back(&f.polygon);
            verts += f.vertexCount;
        }
    }

    std::size_t tris = 0;
    for (auto _ : state) {
        tris = 0;
        for (const auto* poly : polys) {
            auto indices = mapbox::earcut<uint32_t>(*poly);
            tris += indices.size();
            benchmark::DoNotOptimize(indices.data());
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(polys.size()));
    state.counters["polys"] = static_cast<double>(polys.size());
    state.counters["verts"] = static_cast<double>(verts);
    state.counters["tris"] = static_cast<double>(tris / 3);
}

// Register benchmarks for all fixtures
static void RegisterBenchmarks() {
    auto fixtures = getBenchmarkFixtures();

    for (size_t i = 0; i < fixtures.size(); ++i) {
        benchmark::RegisterBenchmark(("BM_EarcutTriangulation/" + fixtures[i]->name).c_str(), BM_EarcutTriangulation)
            ->Arg(static_cast<int>(i))
            ->Unit(benchmark::kMicrosecond);

        benchmark::RegisterBenchmark(("BM_LibtessTriangulation/" + fixtures[i]->name).c_str(), BM_LibtessTriangulation)
            ->Arg(static_cast<int>(i))
            ->Unit(benchmark::kMicrosecond);
    }

    benchmark::RegisterBenchmark("BM_EarcutTiles/small", BM_EarcutTiles, TileBucket::Small)
        ->Unit(benchmark::kMillisecond);
    benchmark::RegisterBenchmark("BM_EarcutTiles/large", BM_EarcutTiles, TileBucket::Large)
        ->Unit(benchmark::kMillisecond);
    benchmark::RegisterBenchmark("BM_EarcutTiles/all", BM_EarcutTiles, TileBucket::All)->Unit(benchmark::kMillisecond);
}

int main(int argc, char** argv) {
    RegisterBenchmarks();
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
