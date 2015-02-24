#include <earcut.hpp>

#include "fixtures/geometries.hpp"

#include <chrono>

template <typename Coord, typename Polygon>
void bench(const char *name, const Polygon &polygon) {
    fprintf(stderr, "benchmarking %s...\t", name);

    std::vector<uint64_t> runs;
    uint64_t total = 0;
    uint32_t warmup = 0;

    mapbox::Earcut<Coord> earcut;

    while (total < 2e9 || runs.size() < 500) {
        const auto started = std::chrono::high_resolution_clock::now();
        earcut(polygon);
        const auto finished = std::chrono::high_resolution_clock::now();

        // Don't count the first couple of iterations.
        if (warmup >= 10) {
            const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started).count();
            runs.push_back(duration);
            total += duration;
        } else {
            warmup++;
        }
    }

    fprintf(stderr, "%.0f ops/s\n", double(runs.size()) / (double(total) / 1e9));
}

int main() {

    bench<int>("bad_hole", mapbox::fixtures::bad_hole);
    bench<int>("building", mapbox::fixtures::building);
    bench<int>("degenerate", mapbox::fixtures::degenerate);
    bench<double>("dude", mapbox::fixtures::dude);
    bench<int>("empty_square", mapbox::fixtures::empty_square);
    bench<int>("water_huge", mapbox::fixtures::water_huge);
    bench<int>("water_huge2", mapbox::fixtures::water_huge2);
    bench<int>("water", mapbox::fixtures::water);
    bench<int>("water2", mapbox::fixtures::water2);
    bench<int>("water3", mapbox::fixtures::water3);
    bench<int>("water3b", mapbox::fixtures::water3b);
    bench<int>("water4", mapbox::fixtures::water4);

    return 0;
}