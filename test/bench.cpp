#include "fixtures/geometries.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <set>

template<typename Proc>
double bench(Proc&& procedure) {
    int64_t runs = -10;
    int64_t total = 0;

    while (total < 2000000000ll || runs < 100) {
        const auto started = std::chrono::high_resolution_clock::now();
        procedure();
        const auto finished = std::chrono::high_resolution_clock::now();

        // Don't count the first couple of iterations.
        if (++runs > 0) {
            total += std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started).count();
        }
    }

    return double(runs) / (double(total) / 1e9);
}

void report(mapbox::fixtures::FixtureTester* fixture, const int cols[]) {
    std::ios::fmtflags flags(std::cerr.flags());
    const char filling = std::cerr.fill();
    std::cerr << std::setfill(' ');
    std::cerr << "| " << std::left << std::setw(cols[0]) << fixture->name << " | ";
    auto earcut = bench([&]{ fixture->earcut(); });
    std::cerr << std::right << std::setw(cols[1] - 6) << std::fixed << std::setprecision(0) << earcut << " ops/s | ";
    auto libtess2 = bench([&]{ fixture->libtess(); });
    std::cerr << std::setw(cols[2] - 6) << std::setprecision(0) << libtess2 << " ops/s |" << std::endl;
    std::cerr << std::setfill(filling);
    std::cerr.flags(flags);
}

void separator(const int cols[]) {
    std::ios::fmtflags flags(std::cerr.flags());
    const char filling = std::cerr.fill();
    std::cerr << std::setfill('-');
    for (int i = 0; cols[i]; i++) {
        std::cerr << "+" << std::setw(cols[i]+2) << std::cerr.fill();
    }
    std::cerr << std::setfill(filling);
    std::cerr << "+" << std::endl;
    std::cerr.flags(flags);
}

int main() {
    std::cerr.imbue(std::locale(""));
    const int cols[] = { 14, 18, 18, 0 };

    separator(cols);

    std::ios::fmtflags flags(std::cerr.flags());
    std::cerr << "|" << std::left
        << std::setw(cols[0]+1) << " Polygon" << " |"
        << std::setw(cols[1]+1) << " earcut" << " |"
        << std::setw(cols[2]+1) << " libtess2" << " |"
        << std::endl;
    std::cerr.flags(flags);

    separator(cols);

    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    std::set<std::string> bench_whitelist = {
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
