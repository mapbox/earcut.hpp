#include "fixtures/geometries.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <unordered_set>

template<typename Proc>
double bench(Proc&& procedure) {
    std::vector<int64_t> runs;
    int64_t total = 0;
    uint32_t warmup = 0;

    while (total < 2000000000ll || runs.size() < 100) {
        const auto started = std::chrono::high_resolution_clock::now();
        procedure();
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

    return double(runs.size()) / (double(total) / 1e9);
}

void report(mapbox::fixtures::FixtureTester* fixture, const int cols[]) {
    std::cerr << "| " << std::left << std::setw(cols[0]) << fixture->name << " | ";
    auto earcut = bench([&]{ fixture->earcut(); });
    std::cerr << std::right << std::setw(cols[1] - 6) << std::fixed << std::setprecision(0) << earcut << " ops/s | ";
    auto libtess2 = bench([&]{ fixture->libtess(); });
    std::cerr << std::setw(cols[2] - 6) << std::setprecision(0) << libtess2 << " ops/s |" << std::endl;
}

void separator(const int cols[]) {
    const char filling = std::cerr.fill();
    std::cerr << std::setfill('-');
    for (int i = 0; cols[i]; i++) {
        std::cerr << "+" << std::setw(cols[i]+2) << std::cerr.fill();
    }
    std::cerr << std::setfill(filling);
    std::cerr << "+" << std::endl;
}

int main() {
    std::cerr.imbue(std::locale(""));
    const int cols[] = { 14, 18, 18, 0 };

    separator(cols);

    std::cerr << "|" << std::left
        << std::setw(cols[0]+1) << " Polygon" << " |"
        << std::setw(cols[1]+1) << " earcut" << " |"
        << std::setw(cols[2]+1) << " libtess2" << " |"
        << std::endl;

    separator(cols);

    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    std::unordered_set<std::string> bench_whitelist = {
        "bad_hole", "building", "degenerate", "dude", "empty_square", "water_huge",
        "water_huge2", "water", "water2", "water3", "water3b", "water4"
    };
    for (auto fixture : fixtures) {
        if (bench_whitelist.find(fixture->name) != bench_whitelist.end()) {
            report(fixture, cols);
        }
    }

    separator(cols);

    return 0;
}
