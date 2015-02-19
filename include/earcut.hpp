#pragma once

#include <utility>

namespace mapbox {

namespace util {

template <std::size_t I, typename T> struct nth {
    inline static typename std::tuple_element<I, T>::type
    get(const T &t) { return std::get<I>(t); };
};

}

template <typename Polygon, typename Triangles = typename Polygon::value_type>
auto earcut(const Polygon &polygon) -> Triangles;

}
