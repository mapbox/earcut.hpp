#include <earcut_detail.hpp>

#include "point.hpp"

#include <tuple>
#include <array>
#include <utility> // for std::pair

namespace mapbox {
namespace util {

template<> struct nth<0, Point> {
    inline static decltype(Point::x) get(const Point &t) { return t.x; };
};

template<> struct nth<1, Point> {
    inline static decltype(Point::y) get(const Point &t) { return t.y; };
};

}
}

// Explicit instantiation
template std::vector<Point> mapbox::earcut(const std::vector<std::vector<Point>> &);
template class mapbox::Earcut<std::vector<std::vector<Point>>>;

template std::vector<std::tuple<int64_t, int64_t>> mapbox::earcut(const std::vector<std::vector<std::tuple<int64_t, int64_t>>> &);
template class mapbox::Earcut<std::vector<std::vector<std::tuple<int64_t, int64_t>>>>;

template std::vector<std::array<int64_t, 2>> mapbox::earcut(const std::vector<std::vector<std::array<int64_t, 2>>> &);
template class mapbox::Earcut<std::vector<std::vector<std::array<int64_t, 2>>>>;

template std::vector<std::pair<int64_t, int64_t>> mapbox::earcut(const std::vector<std::vector<std::pair<int64_t, int64_t>>> &);
template class mapbox::Earcut<std::vector<std::vector<std::pair<int64_t, int64_t>>>>;
