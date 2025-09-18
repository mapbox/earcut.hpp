#include <benchmark/benchmark.h>
#include "fixtures/geometries.hpp"

#include <set>
#include <vector>

// Get benchmark fixtures - only those in the whitelist for performance reasons
std::vector<mapbox::fixtures::FixtureTester*> getBenchmarkFixtures() {
    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    std::set<std::string> bench_whitelist = {
        "bad_hole", "building", "degenerate", "dude", "empty_square", "water_huge",
        "water_huge2", "water", "water2", "water3", "water3b", "water4"
    };

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

// Benchmark libtess2 triangulation
static void BM_LibtessTriangulation(benchmark::State& state) {
    auto fixtures = getBenchmarkFixtures();
    auto fixture = fixtures[static_cast<size_t>(state.range(0))];

    for (auto _ : state) {
        auto result = fixture->libtess();
        benchmark::DoNotOptimize(result);
    }
    state.SetLabel(fixture->name);
}

// Register benchmarks for all fixtures
static void RegisterBenchmarks() {
    auto fixtures = getBenchmarkFixtures();

    for (size_t i = 0; i < fixtures.size(); ++i) {
        benchmark::RegisterBenchmark(("BM_EarcutTriangulation/" + fixtures[i]->name).c_str(),
                                   BM_EarcutTriangulation)
            ->Arg(static_cast<int>(i))
            ->Unit(benchmark::kMicrosecond);

        benchmark::RegisterBenchmark(("BM_LibtessTriangulation/" + fixtures[i]->name).c_str(),
                                   BM_LibtessTriangulation)
            ->Arg(static_cast<int>(i))
            ->Unit(benchmark::kMicrosecond);
    }
}

int main(int argc, char** argv) {
    RegisterBenchmarks();
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
