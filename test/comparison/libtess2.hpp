#pragma once
#include "libtess2/tesselator.h"

#include <memory>
#include <vector>
#include <array>

template <typename Coord, typename Polygon>
class Libtess2Tesselator {
    using Vertex = std::array<Coord, 2>;
    using Triangles = std::vector<Vertex>;

public:
    Libtess2Tesselator(const Polygon &polygon)
        : tess(std::unique_ptr<TESStesselator, tessDeleter>(tessNewTess(nullptr)))
    {
        // Convert the polygon to Libtess2 format.
        for (const auto &ring : polygon) {
            std::vector<TESSreal> tessRing;
            for (const auto &pt : ring) {
                tessRing.push_back(pt.first);
                tessRing.push_back(pt.second);
            }
            tessPolygon.push_back(tessRing);
        }
    }

    void run() {
        // Add polygon data
        for (const auto &tessRing : tessPolygon) {
            tessAddContour(tess.get(), vertexSize, tessRing.data(), stride, (int)tessRing.size() / vertexSize);
        }

        int status = tessTesselate(tess.get(), TESS_WINDING_POSITIVE, TESS_POLYGONS, verticesPerTriangle, vertexSize, 0);
        if (!status) {
            throw std::runtime_error("tesselation failed");
        }
    }

    auto triangles() -> const Triangles & {
        triangleData.clear();

        const auto vertices = tessGetVertices(tess.get());
        const auto elements = tessGetElements(tess.get());
        const auto elementCount = tessGetElementCount(tess.get());

        for (int i = 0; i < elementCount; i++) {
            const TESSindex *group = &elements[i * verticesPerTriangle];
            if (group[0] != TESS_UNDEF && group[1] != TESS_UNDEF && group[2] != TESS_UNDEF) {
                const auto a = group[0] * vertexSize;
                const auto b = group[1] * vertexSize;
                const auto c = group[2] * vertexSize;

                triangleData.emplace_back(Vertex{{ Coord(vertices[a]), Coord(vertices[a + 1]) }});
                triangleData.emplace_back(Vertex{{ Coord(vertices[b]), Coord(vertices[b + 1]) }});
                triangleData.emplace_back(Vertex{{ Coord(vertices[c]), Coord(vertices[c + 1]) }});
            }
        }

        return triangleData;
    }

private:
    static const int vertexSize = 2;
    static const int stride = sizeof(TESSreal) * vertexSize;
    static const int verticesPerTriangle = 3;

    struct tessDeleter {
        void operator()(TESStesselator *t) const { tessDeleteTess(t); }
    };

    std::vector<std::vector<TESSreal>> tessPolygon;
    const std::unique_ptr<TESStesselator, tessDeleter> tess;

    Triangles triangleData;
};
