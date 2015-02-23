#include <earcut.hpp>

#include "fixtures/geometries.hpp"

#include <chrono>

template <typename Polygon>
void bench(const char *name, const Polygon &polygon) {
    fprintf(stderr, "benchmarking %s...\t", name);

    std::vector<uint64_t> runs;
    uint64_t total = 0;
    uint32_t warmup = 0;

    mapbox::Earcut<Polygon> earcut;

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

    bench("bad_hole", mapbox::fixtures::bad_hole);
    bench("building", mapbox::fixtures::building);
    bench("degenerate", mapbox::fixtures::degenerate);
    bench("dude", mapbox::fixtures::dude);
    bench("empty_square", mapbox::fixtures::empty_square);
    bench("water_huge", mapbox::fixtures::water_huge);
    bench("water_huge2", mapbox::fixtures::water_huge2);
    bench("water", mapbox::fixtures::water);
    bench("water2", mapbox::fixtures::water2);
    bench("water3", mapbox::fixtures::water3);
    bench("water3b", mapbox::fixtures::water3b);
    bench("water4", mapbox::fixtures::water4);

    return 0;
}