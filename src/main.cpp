#include <earcut.hpp>

#include <iostream>

int main() {
    const auto result = earcut({ { { 661, 112 },
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
                                   { 666, 112 } } });

    for (auto &pt : result) {
        std::cout << std::get<0>(pt) << "/" << std::get<1>(pt) << std::endl;
    }
}
