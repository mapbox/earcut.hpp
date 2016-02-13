## Earcut

A C++ port of [earcut.js](https://github.com/mapbox/earcut), a fast, header-only polygon triangulation library.

The library implements a modified ear slicing algorithm, optimized by [z-order curve](http://en.wikipedia.org/wiki/Z-order_curve) hashing and extended to handle holes, twisted polygons, degeneracies and self-intersections in a way that doesn't _guarantee_ correctness of triangulation, but attempts to always produce acceptable results for practical data like geographical shapes.

It's based on ideas from [FIST: Fast Industrial-Strength Triangulation of Polygons](http://www.cosy.sbg.ac.at/~held/projects/triang/triang.html) by Martin Held and [Triangulation by Ear Clipping](http://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf) by David Eberly.


## Usage

```cpp
// The number type to use for tessellation
using Coord = double;

// The index type. Defaults to uint32_t, but you can also pass uint16_t if you know that your
// data won't have more than 65536 vertices.
using N = uint32_t;

// Create array
using Point = std::array<Coord, 2>;
std::vector<std::vector<Point>> polygon;
// ... fill polygon structure with actual data

// Run tessellation
// Returns array of indices that refer to the vertices of the input polygon. 
// Three subsequent indices form a triangle.
std::vector<N> indices = mapbox::earcut<N>(polygon);
```

It is also possible to use your custom point type as input. There are default accessors defined for `std::tuple`, `std::pair`, and `std::array`. For a custom type (like Clipper's `IntPoint` type), do this:

```cpp
struct IntPoint {
    int64_t X, Y;
};

namespace mapbox {
namespace util {

template <>
struct nth<0, IntPoint> {
    inline static int64_t get(const IntPoint &t) {
        return t.X;
    };
};
template <>
struct nth<1, IntPoint> {
    inline static int64_t get(const IntPoint &t) {
        return t.Y;
    };
};

} // namespace util
} // namespace mapbox
```

Note that the `earcut.vertices` will always have the type `std::array<Coord, 2>`, and not the source vertex type.

## Status

This is currently based on [earcut 2.0.8](https://github.com/mapbox/earcut#208-jan-19-2016).
