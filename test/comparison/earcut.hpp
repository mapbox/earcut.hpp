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

private:
    mapbox::Earcut<Coord> earcut;
    const Polygon &polygon;
};
