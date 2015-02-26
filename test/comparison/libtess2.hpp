#pragma once
#include "libtess2/tesselator.h"

#include <memory>
#include <vector>

template <typename Coord, typename Polygon>
class Libtess2Tesselator {
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
        } else {
            tessGetVertices(tess.get());
            tessGetVertexCount(tess.get());
            tessGetVertexIndices(tess.get());
            tessGetElements(tess.get());
            tessGetElementCount(tess.get());
        }
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
};
