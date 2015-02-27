#pragma once
#include <earcut.hpp>

#include <memory>
#include <vector>

template <typename Coord, typename Polygon>
class EarcutTesselator {
public:
    EarcutTesselator(const Polygon &polygon_)
        : polygon(polygon_)
    {
    }

    void run() {
        earcut(polygon);
    }

    auto indices() const -> const typename mapbox::Earcut<Coord>::Indices & {
        return earcut.indices;
    }

    auto vertices() const -> const typename mapbox::Earcut<Coord>::Vertices & {
        return earcut.vertices;
    }

private:
    mapbox::Earcut<Coord> earcut;
    const Polygon &polygon;
};
