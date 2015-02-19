#include "point.hpp"

#include <earcut.hpp>

#include <iostream>
#include <vector>
#include <tuple>
#include <array>

int main() {
    {
        using Polygon = std::vector<std::vector<Point>>;
        const auto result = mapbox::earcut<Polygon>({
            {
                { 661, 112 },
                { 661, 96 },
                { 666, 96 },
                { 666, 87 },
                { 743, 87 },
                { 771, 87 },
                { 771, 114 },
                { 750, 114 },
                { 750, 113 },
                { 742, 113 },
                { 742, 106 },
                { 710, 106 },
                { 710, 113 },
                { 666, 113 },
                { 666, 112 },
            }
        });

        for (auto &pt : result) {
            std::cout << pt.x << "/" << pt.y << std::endl;
        }
    }

    {
        using Polygon = std::vector<std::vector<std::tuple<int64_t, int64_t>>>;
        const auto result = mapbox::earcut<Polygon>({
            {
                { 661, 112 },
                { 661, 96 },
                { 666, 96 },
                { 666, 87 },
                { 743, 87 },
                { 771, 87 },
                { 771, 114 },
                { 750, 114 },
                { 750, 113 },
                { 742, 113 },
                { 742, 106 },
                { 710, 106 },
                { 710, 113 },
                { 666, 113 },
                { 666, 112 },
            }
        });

        for (auto &pt : result) {
            std::cout << std::get<0>(pt) << "/" << std::get<1>(pt) << std::endl;
        }
    }

    {
        using Polygon = std::vector<std::vector<std::array<int64_t, 2>>>;
        const auto result = mapbox::earcut<Polygon>({
            {
                {{ 661, 112 }},
                {{ 661, 96 }},
                {{ 666, 96 }},
                {{ 666, 87 }},
                {{ 743, 87 }},
                {{ 771, 87 }},
                {{ 771, 114 }},
                {{ 750, 114 }},
                {{ 750, 113 }},
                {{ 742, 113 }},
                {{ 742, 106 }},
                {{ 710, 106 }},
                {{ 710, 113 }},
                {{ 666, 113 }},
                {{ 666, 112 }},
            }
        });

        for (auto &pt : result) {
            std::cout << std::get<0>(pt) << "/" << std::get<1>(pt) << std::endl;
        }
    }

    {
        using Polygon = std::vector<std::vector<std::pair<int64_t, int64_t>>>;
        const auto result = mapbox::earcut<Polygon>({
            {
                { 661, 112 },
                { 661, 96 },
                { 666, 96 },
                { 666, 87 },
                { 743, 87 },
                { 771, 87 },
                { 771, 114 },
                { 750, 114 },
                { 750, 113 },
                { 742, 113 },
                { 742, 106 },
                { 710, 106 },
                { 710, 113 },
                { 666, 113 },
                { 666, 112 },
            }
        });

        for (auto &pt : result) {
            std::cout << std::get<0>(pt) << "/" << std::get<1>(pt) << std::endl;
        }
    }
}
