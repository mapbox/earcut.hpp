#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace mapbox {

namespace util {

template <std::size_t I, typename T>
struct nth {
    inline static typename std::tuple_element<I, T>::type get(const T& t) { return std::get<I>(t); };
};

} // namespace util

namespace detail {

template <typename N = uint32_t, typename Coord = double>
class Earcut {
public:
    std::vector<N> indices;
    std::size_t vertices = 0;

    template <typename Polygon>
    void operator()(const Polygon& points);

private:
    using WideCoord = typename std::conditional<std::is_integral<Coord>::value, int64_t,
                                             long double>::type;

    // A vertex in a circular doubly linked list representing a polygon ring.
    // prev/next are always linked after construction; prevZ/nextZ are the
    // z-order list links and are null at the ends.
    struct Node {
        Node(N index, Coord x_, Coord y_) : x(x_), y(y_), i(index & ~(N(1) << (sizeof(N) * 8 - 1))), steiner(0) {}
        Node(const Node&) = delete;
        Node& operator=(const Node&) = delete;
        Node(Node&&) = delete;
        Node& operator=(Node&&) = delete;

        const Coord x;
        const Coord y;

        // previous and next vertex nodes in a polygon ring
        Node* prev = nullptr;
        Node* next = nullptr;

        // z-order curve value; doubles as the owning block index during eliminateHoles
        int32_t z = 0;

        // original index in polygon
        const N i : (sizeof(N) * 8 - 1);

        // indicates whether this is a steiner point; single-vertex holes are preserved through filterPoints
        N steiner : 1;

        // previous and next nodes in z-order
        Node* prevZ = nullptr;
        Node* nextZ = nullptr;
    };

    // Cache-optimized Triangle structure for repeated geometric tests
    struct Triangle {
        const Coord ax, ay;
        const Coord bx, by;
        const Coord cx, cy;

        Triangle(const Node* a, const Node* b, const Node* c)
            : ax(a->x), ay(a->y), bx(b->x), by(b->y), cx(c->x), cy(c->y) {}

        inline auto area() const -> decltype((by - ay) * (cx - bx) - (bx - ax) * (cy - by)) {
            return (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
        }

        inline bool containsPoint(Coord px, Coord py) const {
            return (cx - px) * (ay - py) >= (ax - px) * (cy - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
                   (bx - px) * (cy - py) >= (cx - px) * (by - py);
        }
    };

    template <typename Ring>
    Node* linkedList(const Ring& points, const bool clockwise);
    Node* filterPoints(Node* start, Node* end = nullptr);
    void earcutLinked(Node* ear, int pass = 0);
    bool isEar(Node* ear);
    bool isEarHashed(Node* ear);
    Node* cureLocalIntersections(Node* start);
    void splitEarcut(Node* start);
    template <typename Polygon>
    Node* eliminateHoles(const Polygon& points, Node* outerNode);
    Node* eliminateHole(Node* hole, Node* outerNode);
    Node* findHoleBridge(Node* hole, Node* outerNode);
    bool sectorContainsSector(const Node* m, const Node* p);
    void buildBlockIndex(std::size_t maxNodes, std::size_t numHoles);
    void indexSegment(Node* head, Node* stop);
    void growBlock(Node* head, Node* tail);
    Node* liveBlockStop(std::size_t block);
    Node* liveBlockHead(std::size_t block);
    void indexCurve(Node* start);
    Node* sortLinked(Node* list);
    int32_t zOrder(Coord x_, Coord y_);
    Node* getLeftmost(Node* start);
    template <typename Ax, typename Ay, typename Bx, typename By, typename Cx, typename Cy, typename Px, typename Py>
    bool pointInTriangle(Ax ax, Ay ay, Bx bx, By by, Cx cx, Cy cy, Px px, Py py) const;
    bool pointInBridgeTriangle(Coord hx, Coord hy, WideCoord qxNum, WideCoord qxDen, Coord mx, Coord my, const Node* p) const;
    bool isValidDiagonal(Node* a, Node* b);
    auto area(const Node* p, const Node* q, const Node* r) const -> decltype((q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y));
    bool equals(const Node* p1, const Node* p2);
    bool intersects(const Node* p1, const Node* q1, const Node* p2, const Node* q2, bool includeBoundary = true);
    bool onSegment(const Node* p, const Node* q, const Node* r);
    template <typename T>
    int sign(T val);
    bool intersectsPolygon(const Node* a, const Node* b);
    bool locallyInside(const Node* a, const Node* b);
    bool middleInside(const Node* a, const Node* b);
    Node* splitPolygon(Node* a, Node* b);
    template <typename Point>
    Node* insertNode(std::size_t i, const Point& p, Node* last);
    void removeNode(Node* p);

    bool hashing;
    Coord minX, maxX;
    Coord minY, maxY;
    WideCoord zScale = 0;

    template <typename T, typename Alloc = std::allocator<T>>
    class ObjectPool {
    public:
        ObjectPool() { allocateNewBlock(256); }
        ObjectPool(std::size_t blockSize_) : baseBlockSize(blockSize_) {
            allocateNewBlock(std::max<std::size_t>(blockSize_, 256));
        }
        ~ObjectPool() { clear(); }
        template <typename... Args>
        T* construct(Args&&... args) {
            // If current block is full, move to next block or allocate new one
            if (currentIndex >= baseBlockSize) {
                currentBlockIndex++;
                if (currentBlockIndex < memoryBlocks.size()) {
                    // Reuse existing block
                    currentIndex = 0;
                } else {
                    // Allocate a new one
                    allocateNewBlock(baseBlockSize);
                }
            }

            T* object = memoryBlocks[currentBlockIndex].get() + currentIndex;
            alloc_traits::construct(alloc, object, std::forward<Args>(args)...);
            totalObjects++;
            currentIndex++;
            return object;
        }
        void reset() { clear(); }
        void clear() {
            // Destroy all objects, but keep blocks allocated for reuse
            std::size_t objectsDestroyed = 0;
            for (std::size_t blockIdx = 0; blockIdx < memoryBlocks.size() && objectsDestroyed < totalObjects;
                 ++blockIdx) {
                // check if we are in the last block
                std::size_t objectsInThisBlock = std::min(baseBlockSize, totalObjects - objectsDestroyed);
                for (std::size_t i = 0; i < objectsInThisBlock; ++i) {
                    T* object = memoryBlocks[blockIdx].get() + i;
                    alloc_traits::destroy(alloc, object);
                }
                objectsDestroyed += objectsInThisBlock;
            }
            // Reset to start from first block again
            currentBlockIndex = 0;
            currentIndex = 0;
            totalObjects = 0;
        }

    private:
        Alloc alloc;
        typedef typename std::allocator_traits<Alloc> alloc_traits;

        // Custom deleter that uses the allocator
        struct AllocDeleter {
            Alloc alloc;
            std::size_t capacity;
            void operator()(T* ptr) { alloc_traits::deallocate(alloc, ptr, capacity); }
        };

        std::vector<std::unique_ptr<T[], AllocDeleter>> memoryBlocks;
        std::vector<std::size_t> blockCapacities;
        std::size_t currentBlockIndex = 0;
        std::size_t currentIndex = 0;
        std::size_t totalObjects = 0;
        std::size_t baseBlockSize = 256;

        void allocateNewBlock(std::size_t capacity) {
            T* rawMemory = alloc_traits::allocate(alloc, capacity);
            auto newBlock = std::unique_ptr<T[], AllocDeleter>(rawMemory, AllocDeleter{alloc, capacity});
            memoryBlocks.push_back(std::move(newBlock));
            blockCapacities.push_back(capacity);
            currentBlockIndex = memoryBlocks.size() - 1;
            currentIndex = 0;
        }
    };

    std::unique_ptr<ObjectPool<Node>> nodes;
    std::vector<Node*> holeQueue;
    std::vector<std::array<Coord, 4>> blockBBox;
    std::vector<Node*> blockHead;
    std::vector<Node*> blockStop;
    std::size_t numBlocks = 0;
    // true only while eliminateHoles merges holes, so removeNode keeps the block index live (growBlock)
    bool indexActive = false;
    // set by filterPoints whenever it removes at least one node; read by earcutLinked's stall handler
    bool filteredOut = false;
};

template <typename N, typename Coord>
template <typename Polygon>
void Earcut<N, Coord>::operator()(const Polygon& points) {
    // reset
    indices.clear();
    vertices = 0;

    if (points.empty()) return;

    Coord x;
    Coord y;
    int threshold = 80;
    std::size_t len = 0;

    for (size_t i = 0; threshold >= 0 && i < points.size(); i++) {
        threshold -= static_cast<int>(points[i].size());
        len += points[i].size();
    }

    // estimate size of nodes and indices
    if (!nodes) {
        std::size_t estimatedNodes = len * 3 / 2;
        nodes = std::make_unique<ObjectPool<Node>>(std::max<std::size_t>(estimatedNodes, 256));
    }
    indices.reserve(len + points[0].size());

    Node* outerNode = linkedList(points[0], true);
    if (!outerNode || outerNode->prev == outerNode->next) return;

    if (points.size() > 1) outerNode = eliminateHoles(points, outerNode);

    // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
    hashing = threshold < 0;
    if (hashing) {
        Node* p = outerNode->next;
        minX = maxX = outerNode->x;
        minY = maxY = outerNode->y;
        do {
            x = p->x;
            y = p->y;
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
            p = p->next;
        } while (p != outerNode);

        // minX, minY and zScale are later used to transform coords into integers for z-order calculation
        const WideCoord width = static_cast<WideCoord>(maxX) - static_cast<WideCoord>(minX);
        const WideCoord height = static_cast<WideCoord>(maxY) - static_cast<WideCoord>(minY);
        zScale = std::max(width, height);
    }

    earcutLinked(outerNode);

    nodes->clear();
    holeQueue.clear();
}

// create a circular doubly linked list from polygon points in the specified winding order
template <typename N, typename Coord>
template <typename Ring>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::linkedList(const Ring& points, const bool clockwise) {
    using Point = typename Ring::value_type;
    auto sum = decltype((Coord{} - Coord{}) * (Coord{} + Coord{})){0};
    const std::size_t len = points.size();
    std::size_t i, j;
    Node* last = nullptr;

    // calculate original winding order of a polygon ring
    for (i = 0, j = len > 0 ? len - 1 : 0; i < len; j = i++) {
        const auto& p1 = points[i];
        const auto& p2 = points[j];
        const auto p20 = util::nth<0, Point>::get(p2);
        const auto p10 = util::nth<0, Point>::get(p1);
        const auto p11 = util::nth<1, Point>::get(p1);
        const auto p21 = util::nth<1, Point>::get(p2);
        sum += (p20 - p10) * (p11 + p21);
    }

    // link points into circular doubly-linked list in the specified winding order
    if (clockwise == (sum > 0)) {
        for (i = 0; i < len; i++) last = insertNode(vertices + i, points[i], last);
    } else {
        for (i = len; i-- > 0;) last = insertNode(vertices + i, points[i], last);
    }

    if (last && equals(last, last->next)) {
        removeNode(last);
        last = last->next;
    }

    vertices += len;

    return last;
}

// Remove collinear or coincident points; removability depends only on a node's immediate
// neighbors, so we sweep forward and re-check the predecessor after each removal. With no end
// we sweep the whole ring, lapping until nothing is removable (the fixpoint the clipper needs).
// With an explicit end we heal only the dirty window around a bridge/diagonal cut, stopping at
// end rather than lapping -- O(window) instead of O(ring).
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::filterPoints(Node* start, Node* end) {
    if (!end) end = start;

    bool full = end == start;

    Node* p = start;
    bool again;
    do {
        again = false;

        if (p != p->next && !p->steiner && (equals(p, p->next) || area(p->prev, p, p->next) == 0)) {
            if (full || p == end) end = p->prev;
            filteredOut = true;
            removeNode(p);
            p = p->prev;
            again = true;
        } else if (full || p != end) {
            p = p->next;
            again = !full;
        }
    } while (again || p != end);

    return end;
}

// main ear slicing loop which triangulates a polygon (given as a linked list)
template <typename N, typename Coord>
void Earcut<N, Coord>::earcutLinked(Node* ear, int pass) {
    if (!ear) return;

    // interlink polygon nodes in z-order
    if (!pass && hashing) indexCurve(ear);

    Node* stop = ear;
    Node* prev;
    Node* next;
    bool cured = pass > 1;

    // iterate through ears, slicing them one by one
    while (ear->prev != ear->next) {
        prev = ear->prev;
        next = ear->next;

        if (hashing ? isEarHashed(ear) : isEar(ear)) {
            // cut off the triangle
            indices.emplace_back(prev->i);
            indices.emplace_back(ear->i);
            indices.emplace_back(next->i);

            removeNode(ear);
            ear = next;
            stop = next;

            continue;
        }

        ear = next;

        // if we looped through the whole remaining polygon and can't find any more ears
        if (ear == stop) {
            // try filtering collinear/coincident points and slicing again; repeat as long as
            // filtering actually removes nodes, since each removal can expose new ears
            filteredOut = false;
            ear = filterPoints(ear);
            if (filteredOut) {
                stop = ear;
                continue;
            }

            // filtering is exhausted: cure small local self-intersections once, then retry
            if (!cured) {
                ear = cureLocalIntersections(ear);
                stop = ear;
                cured = true;
                continue;
            }

            // as a last resort, try splitting the remaining polygon into two
            splitEarcut(ear);
            break;
        }
    }
}

// check whether a polygon node forms a valid ear with adjacent nodes
template <typename N, typename Coord>
bool Earcut<N, Coord>::isEar(Node* ear) {
    const Node* a = ear->prev;
    const Node* b = ear;
    const Node* c = ear->next;

    // reflex check (area(a, b, c) >= 0) is hoisted into the earcutLinked caller in JS;
    // this C++ helper keeps the same predicate locally.
    const Triangle tri(a, b, c);
    if (tri.area() >= 0) return false; // reflex, can't be an ear

    const Coord minTX = std::min(tri.ax, std::min(tri.bx, tri.cx));
    const Coord minTY = std::min(tri.ay, std::min(tri.by, tri.cy));
    const Coord maxTX = std::max(tri.ax, std::max(tri.bx, tri.cx));
    const Coord maxTY = std::max(tri.ay, std::max(tri.by, tri.cy));

    // now make sure we don't have other points inside the potential ear
    Node* p = c->next;

    while (p != a) {
        if (p->x >= minTX && p->x <= maxTX && p->y >= minTY && p->y <= maxTY && !(tri.ax == p->x && tri.ay == p->y) &&
            tri.containsPoint(p->x, p->y) && area(p->prev, p, p->next) >= 0)
            return false;
        p = p->next;
    }

    return true;
}

template <typename N, typename Coord>
bool Earcut<N, Coord>::isEarHashed(Node* ear) {
    const Node* a = ear->prev;
    const Node* b = ear;
    const Node* c = ear->next;

    // reflex check is hoisted into the earcutLinked caller in JS; this C++ helper keeps
    // the same predicate locally.
    const Triangle tri(a, b, c);
    if (tri.area() >= 0) return false; // reflex, can't be an ear

    // triangle bbox; min & max are calculated like this for speed
    const Coord minTX = std::min(tri.ax, std::min(tri.bx, tri.cx));
    const Coord minTY = std::min(tri.ay, std::min(tri.by, tri.cy));
    const Coord maxTX = std::max(tri.ax, std::max(tri.bx, tri.cx));
    const Coord maxTY = std::max(tri.ay, std::max(tri.by, tri.cy));

    // z-order range for the current triangle bbox;
    const int32_t minZ = zOrder(minTX, minTY);
    const int32_t maxZ = zOrder(maxTX, maxTY);

    // look for points inside the triangle in decreasing z-order
    Node* p = ear->prevZ;

    while (p && p->z >= minZ) {
        if (p->x >= minTX && p->x <= maxTX && p->y >= minTY && p->y <= maxTY && p != c &&
            !(tri.ax == p->x && tri.ay == p->y) && tri.containsPoint(p->x, p->y) && area(p->prev, p, p->next) >= 0)
            return false;
        p = p->prevZ;
    }

    // look for points inside the triangle in increasing z-order
    p = ear->nextZ;

    while (p && p->z <= maxZ) {
        if (p->x >= minTX && p->x <= maxTX && p->y >= minTY && p->y <= maxTY && p != c &&
            !(tri.ax == p->x && tri.ay == p->y) && tri.containsPoint(p->x, p->y) && area(p->prev, p, p->next) >= 0)
            return false;
        p = p->nextZ;
    }

    return true;
}

// go through all polygon nodes and cure small local self-intersections
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::cureLocalIntersections(Node* start) {
    Node* p = start;
    bool cured = false;
    do {
        Node* a = p->prev;
        Node* b = p->next->next;

        // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2])
        if (intersects(a, p, p->next, b, false) && locallyInside(a, b) && locallyInside(b, a)) {
            indices.emplace_back(a->i);
            indices.emplace_back(p->i);
            indices.emplace_back(b->i);

            // remove two nodes involved
            removeNode(p);
            removeNode(p->next);

            p = start = b;
            cured = true;
        }
        p = p->next;
    } while (p != start);

    return cured ? filterPoints(p) : p;
}

// try splitting polygon into two and triangulate them independently
template <typename N, typename Coord>
void Earcut<N, Coord>::splitEarcut(Node* start) {
    // look for a valid diagonal that divides the polygon into two
    Node* a = start;
    do {
        Node* b = a->next->next;
        while (b != a->prev) {
            if (a->i != b->i && isValidDiagonal(a, b)) {
                // split the polygon in two by the diagonal
                Node* c = splitPolygon(a, b);

                // filter colinear points around the cuts
                a = filterPoints(a, a->next);
                c = filterPoints(c, c->next);

                // run earcut on each half
                earcutLinked(a);
                earcutLinked(c);
                return;
            }
            b = b->next;
        }
        a = a->next;
    } while (a != start);
}

// link every hole into the outer loop, producing a single-ring polygon without holes
template <typename N, typename Coord>
template <typename Polygon>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::eliminateHoles(const Polygon& points, Node* outerNode) {
    const size_t len = points.size();

    holeQueue.clear();
    for (size_t i = 1; i < len; i++) {
        Node* list = linkedList(points[i], false);
        if (list) {
            if (list == list->next) list->steiner = true;
            holeQueue.push_back(getLeftmost(list));
        }
    }
    std::sort(holeQueue.begin(), holeQueue.end(), [](const Node* a, const Node* b) {
        // when the left-most point of 2 holes meet at a vertex, sort the holes counterclockwise so that when we find
        // the bridge to the outer shell is always the point that they meet at.
        auto slopeLess = [](const Node* p, const Node* q) {
            const auto dy_p = p->next->y - p->y;
            const auto dx_p = p->next->x - p->x;
            const auto dy_q = q->next->y - q->y;
            const auto dx_q = q->next->x - q->x;
            // both edges go upward from leftmost (dy >= 0); compare dy/dx without division
            if (dx_p == 0 && dx_q == 0) return false;
            if (dx_p == 0) return false;       // +inf not < finite
            if (dx_q == 0) return true;        // finite < +inf
            if ((dx_p > 0) == (dx_q > 0)) return dy_p * dx_q < dy_q * dx_p;
            return dy_p * dx_q > dy_q * dx_p;
        };
        return a->x < b->x ||
               (a->x == b->x && (a->y < b->y || (a->y == b->y && slopeLess(a, b))));
    });

    // block-bbox index for findHoleBridge, grown append-only as holes merge. Seed it
    // with the outer ring, then append each merged hole.
    buildBlockIndex(vertices, holeQueue.size());
    indexSegment(outerNode, outerNode);

    // process holes from left to right; indexActive lets removeNode keep block bboxes live as
    // filterPoints heals edges during merges (see growBlock)
    indexActive = true;
    for (size_t i = 0; i < holeQueue.size(); i++) {
        outerNode = eliminateHole(holeQueue[i], outerNode);
    }
    indexActive = false;

    // collapse collinear/coincident points across the whole merged ring once before clipping
    return filterPoints(outerNode);
}

// find a bridge between vertices that connects hole with an outer ring and link it
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::eliminateHole(Node* hole, Node* outerNode) {
    Node* bridge = findHoleBridge(hole, outerNode);
    if (!bridge) {
        return outerNode;
    }

    Node* bridgeReverse = splitPolygon(bridge, hole);

    // index the merged-in segment before filtering: in ring order the splice runs
    // bridge -> hole -> bridgeReverse -> bridge2 -> (bridge's old next), covering the
    // hole's edges and both new slit edges. filterPoints below only drops collinear /
    // coincident points, so these bboxes stay valid (conservative) supersets.
    Node* bridge2 = bridgeReverse->next;
    indexSegment(bridge, bridge2->next);

    // heal collinear/coincident points around the two new slit edges
    filterPoints(bridgeReverse, bridgeReverse->next);
    return filterPoints(bridge, bridge->next);
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::findHoleBridge(Node* hole, Node* outerNode) {
    Node* p = outerNode;
    const Coord hx = hole->x;
    const Coord hy = hole->y;
    Node* m = nullptr;
    WideCoord qxNum = 0;
    WideCoord qxDen = 1;

    if (equals(hole, p)) return p;

    // find a segment intersected by a ray from the hole's leftmost point to the left;
    // segment's endpoint with lesser x will be potential connection point
    // unless they intersect at a vertex, then choose the vertex
    for (std::size_t block = 0; block < numBlocks; block++) {
        const auto& bbox = blockBBox[block];
        // scan blocks; skip any whose bbox can't hold a crossing that beats qx and lies left
        // of hx (the prune Morton order can't express -- explicit per-axis [minY,maxY]/[minX,maxX])
        if (hy < bbox[1] || hy > bbox[3] || bbox[0] > hx || (m && bbox[2] <= m->x)) continue;

        // ensure the walk's exclusive bound is live so we don't overrun into other blocks
        Node* stop = liveBlockStop(block);
        p = liveBlockHead(block);
        do {
            if (p->prev->next == p) {
                if (equals(hole, p->next)) return p->next;
                if (hy <= p->y && hy >= p->next->y && p->next->y != p->y) {
                    const WideCoord den = static_cast<WideCoord>(p->y) - static_cast<WideCoord>(p->next->y);
                    const WideCoord num = static_cast<WideCoord>(p->x) * den +
                                       (static_cast<WideCoord>(p->y) - static_cast<WideCoord>(hy)) *
                                           (static_cast<WideCoord>(p->next->x) - static_cast<WideCoord>(p->x));
                    const WideCoord hxScaled = static_cast<WideCoord>(hx) * den;
                    const bool leftOfHole = num <= hxScaled;
                    const bool rightOfBest = !m || num * qxDen > qxNum * den;
                    if (leftOfHole && rightOfBest) {
                        qxNum = num;
                        qxDen = den;
                        m = p->x < p->next->x ? p : p->next;
                        if (num == hxScaled) return m;
                    }
                }
            }
            p = p->next;
        } while (p != stop);
    }

    if (!m) return 0;

    // look for points inside the triangle of hole point, segment intersection and endpoint;
    // if there are no points found, we have a valid connection;
    // otherwise choose the point of the minimum angle with the ray as connection point

    WideCoord tanMinNum = 0;
    WideCoord tanMinDen = 1;
    bool hasTanMin = false;

    const Coord mx = m->x;
    const Coord my = m->y;
    // the triangle's y span; x span is [mx, hx]
    Coord tminY = std::min(hy, my);
    Coord tmaxY = std::max(hy, my);

    for (std::size_t block = 0; block < numBlocks; block++) {
        const auto& bbox = blockBBox[block];
        // scan the same blocks; skip any whose bbox can't overlap the triangle's [mx,hx] x [tminY,tmaxY] box
        if (bbox[2] < mx || bbox[0] > hx || bbox[3] < tminY || bbox[1] > tmaxY) continue;

        Node* stop = liveBlockStop(block);
        p = liveBlockHead(block);
        do {
            if (p->prev->next == p && hx >= p->x && p->x >= mx && hx != p->x &&
                pointInBridgeTriangle(hx, hy, qxNum, qxDen, mx, my, p)) {
                const WideCoord tanCurNum = static_cast<WideCoord>(hy > p->y ? hy - p->y : p->y - hy);
                const WideCoord tanCurDen = static_cast<WideCoord>(hx) - static_cast<WideCoord>(p->x);
                const bool tanLess = !hasTanMin || tanCurNum * tanMinDen < tanMinNum * tanCurDen;
                const bool tanEqual = hasTanMin && tanCurNum * tanMinDen == tanMinNum * tanCurDen;

                // if hole point sits on p's horizontal edge (T-junction touch): the bridge runs
                // along that edge; locallyInside rejects it as collinear, but it's valid
                if ((locallyInside(p, hole) || (p->y == hy && p->next->y == hy && p->next->x > hx)) &&
                    (tanLess || (tanEqual && (p->x > m->x || (p->x == m->x && sectorContainsSector(m, p)))))) {
                    m = p;
                    tanMinNum = tanCurNum;
                    tanMinDen = tanCurDen;
                    hasTanMin = true;
                }
            }

            p = p->next;
        } while (p != stop);
    }

    return m;
}

// whether sector in vertex m contains sector in vertex p in the same coordinates
template <typename N, typename Coord>
bool Earcut<N, Coord>::sectorContainsSector(const Node* m, const Node* p) {
    return area(m->prev, m, p->prev) < 0 && area(p->next, m, m->next) < 0;
}

// Block-bbox index for findHoleBridge (issue #183): one [minX,minY,maxX,maxY] bbox per K
// consecutive ring edges, so the leftward-ray scan can skip whole blocks in O(1) instead
// of walking the entire merged ring. Grown append-only: the outer ring seeds it, then each
// merged hole appends a segment (head node, stop node, K-blocks over head..stop); independent
// segments, not a ring tiling, since splices land mid-ring. Buffers are sized once from the
// input upper bound and reused across calls.
//
// filterPoints only drops collinear/coincident points, so a stale bbox stays a conservative
// superset of its live edges (never a false skip); the scan skips dead nodes (p->prev->next != p)
// and lazily advances a dead stop. Blocks are scanned in append (not ring) order, so the chosen
// bridge can differ from the un-indexed code -- a different but equally valid result.
template <typename N, typename Coord>
void Earcut<N, Coord>::buildBlockIndex(std::size_t maxNodes, std::size_t numHoles) {
    const std::size_t k = 16;
    const std::size_t maxBlocks = (maxNodes + 2 * numHoles + k - 1) / k + numHoles + 2;
    blockBBox.resize(maxBlocks);
    blockHead.resize(maxBlocks);
    blockStop.resize(maxBlocks);
    numBlocks = 0;
}

template <typename N, typename Coord>
void Earcut<N, Coord>::indexSegment(Node* head, Node* stop) {
    const std::size_t k = 16;
    Node* p = head;
    do {
        const std::size_t block = numBlocks++;
        blockHead[block] = p;

        Coord minBX = std::numeric_limits<Coord>::max();
        Coord minBY = std::numeric_limits<Coord>::max();
        Coord maxBX = std::numeric_limits<Coord>::lowest();
        Coord maxBY = std::numeric_limits<Coord>::lowest();

        std::size_t count = 0;
        do {
            Node* c = p->next; // edge p->c; bbox must bound both endpoints
            p->z = static_cast<int32_t>(block);
            minBX = std::min(minBX, std::min(p->x, c->x));
            minBY = std::min(minBY, std::min(p->y, c->y));
            maxBX = std::max(maxBX, std::max(p->x, c->x));
            maxBY = std::max(maxBY, std::max(p->y, c->y));
            p = c;
        } while (++count < k && p != stop);

        blockStop[block] = p;
        blockBBox[block] = {{minBX, minBY, maxBX, maxBY}};
    } while (p != stop);
}

// when filterPoints heals an edge head->tail (removing the collinear node between them), the
// healed edge can extend past head's frozen block bbox if its old far endpoint lived in another
// block; grow head's block bbox to cover tail so the leftward-ray prune can't false-skip it.
template <typename N, typename Coord>
void Earcut<N, Coord>::growBlock(Node* head, Node* tail) {
    auto& bbox = blockBBox[static_cast<std::size_t>(head->z)];
    bbox[0] = std::min(bbox[0], tail->x);
    bbox[1] = std::min(bbox[1], tail->y);
    bbox[2] = std::max(bbox[2], tail->x);
    bbox[3] = std::max(bbox[3], tail->y);
}

// advance the block's exclusive walk bound past nodes removed by filterPoints
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::liveBlockStop(std::size_t block) {
    Node* stop = blockStop[block];
    while (stop->prev->next != stop) stop = stop->next;
    blockStop[block] = stop;
    return stop;
}

// the block's head node can be removed by filterPoints during merges; advance it to the next
// live node so the walk doesn't start on (and immediately terminate at) a dead node. For the
// single full-ring seed block (head == stop) the same forward advance keeps them equal, so the
// do-while still laps the whole ring instead of collapsing to an empty walk.
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::liveBlockHead(std::size_t block) {
    Node* head = blockHead[block];
    while (head->prev->next != head) head = head->next;
    blockHead[block] = head;
    return head;
}

// interlink polygon nodes in z-order
template <typename N, typename Coord>
void Earcut<N, Coord>::indexCurve(Node* start) {
    assert(start);
    Node* p = start;

    do {
        // always (re)compute: z may still hold a block index left over from eliminateHoles
        p->z = zOrder(p->x, p->y);
        p->prevZ = p->prev;
        p->nextZ = p->next;
        p = p->next;
    } while (p != start);

    p->prevZ->nextZ = nullptr;
    p->prevZ = nullptr;

    p = sortLinked(p);
}

// Simon Tatham's linked list merge sort algorithm
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::sortLinked(Node* list) {
    assert(list);
    Node* p;
    Node* q;
    Node* e;
    Node* tail;
    int i, numMerges, pSize, qSize;
    int inSize = 1;

    for (;;) {
        p = list;
        list = nullptr;
        tail = nullptr;
        numMerges = 0;

        while (p) {
            numMerges++;
            q = p;
            pSize = 0;
            for (i = 0; i < inSize; i++) {
                pSize++;
                q = q->nextZ;
                if (!q) break;
            }

            qSize = inSize;

            while (pSize > 0 || (qSize > 0 && q)) {
                if (pSize == 0) {
                    e = q;
                    q = q->nextZ;
                    qSize--;
                } else if (qSize == 0 || !q) {
                    e = p;
                    p = p->nextZ;
                    pSize--;
                } else if (p->z <= q->z) {
                    e = p;
                    p = p->nextZ;
                    pSize--;
                } else {
                    e = q;
                    q = q->nextZ;
                    qSize--;
                }

                if (tail)
                    tail->nextZ = e;
                else
                    list = e;

                e->prevZ = tail;
                tail = e;
            }

            p = q;
        }

        tail->nextZ = nullptr;

        if (numMerges <= 1) return list;

        inSize *= 2;
    }
}

// z-order of a Vertex given coords and size of the data bounding box
template <typename N, typename Coord>
int32_t Earcut<N, Coord>::zOrder(Coord x_, Coord y_) {
    // coords are transformed into non-negative 15-bit integer range
    const WideCoord xDiff = static_cast<WideCoord>(x_) - static_cast<WideCoord>(minX);
    const WideCoord yDiff = static_cast<WideCoord>(y_) - static_cast<WideCoord>(minY);
    int32_t x = zScale != WideCoord(0) ? static_cast<int32_t>((xDiff * WideCoord(32767)) / zScale) : 0;
    int32_t y = zScale != WideCoord(0) ? static_cast<int32_t>((yDiff * WideCoord(32767)) / zScale) : 0;

    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;

    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;

    return x | (y << 1);
}

// find the leftmost node of a polygon ring
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::getLeftmost(Node* start) {
    Node* p = start;
    Node* leftmost = start;
    do {
        if (p->x < leftmost->x || (p->x == leftmost->x && p->y < leftmost->y)) leftmost = p;
        p = p->next;
    } while (p != start);

    return leftmost;
}

// check if a point lies within a convex triangle
template <typename N, typename Coord>
template <typename Ax, typename Ay, typename Bx, typename By, typename Cx, typename Cy, typename Px, typename Py>
bool Earcut<N, Coord>::pointInTriangle(Ax ax, Ay ay, Bx bx, By by, Cx cx, Cy cy, Px px, Py py) const {
    return (cx - px) * (ay - py) >= (ax - px) * (cy - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
           (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

template <typename N, typename Coord>
bool Earcut<N, Coord>::pointInBridgeTriangle(Coord hx, Coord hy, WideCoord qxNum, WideCoord qxDen, Coord mx, Coord my, const Node* p) const {
    const WideCoord ax = (hy < my ? static_cast<WideCoord>(hx) * qxDen : qxNum);
    const WideCoord bx = static_cast<WideCoord>(mx) * qxDen;
    const WideCoord cx = (hy < my ? qxNum : static_cast<WideCoord>(hx) * qxDen);
    const WideCoord px = static_cast<WideCoord>(p->x) * qxDen;
    const WideCoord ay = static_cast<WideCoord>(hy);
    const WideCoord by = static_cast<WideCoord>(my);
    const WideCoord py = static_cast<WideCoord>(p->y);

    return (cx - px) * (ay - py) >= (ax - px) * (ay - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
           (bx - px) * (ay - py) >= (cx - px) * (by - py);
}

// check if a diagonal between two polygon nodes is valid (lies in polygon interior)
template <typename N, typename Coord>
bool Earcut<N, Coord>::isValidDiagonal(Node* a, Node* b) {
    const bool zeroLength = equals(a, b) && area(a->prev, a, a->next) > 0 && area(b->prev, b, b->next) > 0;
    return a->next->i != b->i &&
           (zeroLength || (locallyInside(a, b) && locallyInside(b, a) &&
                           (area(a->prev, a, b->prev) != 0 || area(a, b->prev, b) != 0))) &&
           !intersectsPolygon(a, b) && (zeroLength || middleInside(a, b));
}

// signed area of a triangle
template <typename N, typename Coord>
auto Earcut<N, Coord>::area(const Node* p, const Node* q, const Node* r) const
    -> decltype((q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y)) {
    return (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
}

// check if two points are equal
template <typename N, typename Coord>
bool Earcut<N, Coord>::equals(const Node* p1, const Node* p2) {
    return p1->x == p2->x && p1->y == p2->y;
}

// check if two segments intersect; by default includes collinear boundary touches
template <typename N, typename Coord>
bool Earcut<N, Coord>::intersects(const Node* p1, const Node* q1, const Node* p2, const Node* q2, bool includeBoundary) {
    const auto o1 = area(p1, q1, p2);
    const auto o2 = area(p1, q1, q2);
    const auto o3 = area(p2, q2, p1);
    const auto o4 = area(p2, q2, q1);

    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return true;

    if (!includeBoundary) return false;

    if (o1 == 0 && onSegment(p1, p2, q1)) return true; // p1, q1 and p2 are collinear and p2 lies on p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; // p1, q1 and q2 are collinear and q2 lies on p1q1
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; // p2, q2 and p1 are collinear and p1 lies on p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; // p2, q2 and q1 are collinear and q1 lies on p2q2

    return false;
}

// for collinear points p, q, r, check if point q lies on segment pr
template <typename N, typename Coord>
bool Earcut<N, Coord>::onSegment(const Node* p, const Node* q, const Node* r) {
        return q->x <= std::max(p->x, r->x) && q->x >= std::min(p->x, r->x) && q->y <= std::max(p->y, r->y) &&
            q->y >= std::min(p->y, r->y);
}

template <typename N, typename Coord>
template <typename T>
int Earcut<N, Coord>::sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// check if a polygon diagonal intersects any polygon segments
template <typename N, typename Coord>
bool Earcut<N, Coord>::intersectsPolygon(const Node* a, const Node* b) {
    // diagonal bbox; an edge whose bbox can't overlap it can't intersect it, so
    // skip the orientation test for those (the common case -- the diagonal is short)
    const Coord minPX = std::min(a->x, b->x);
    const Coord maxPX = std::max(a->x, b->x);
    const Coord minPY = std::min(a->y, b->y);
    const Coord maxPY = std::max(a->y, b->y);

    const Node* p = a;
    do {
        const Node* n = p->next;
        if ((p->x > maxPX && n->x > maxPX) || (p->x < minPX && n->x < minPX) || (p->y > maxPY && n->y > maxPY) ||
            (p->y < minPY && n->y < minPY)) {
            p = n;
            continue;
        }
        if (p->i != a->i && n->i != a->i && p->i != b->i && n->i != b->i && intersects(p, n, a, b)) return true;
        p = n;
    } while (p != a);

    return false;
}

// check if a polygon diagonal is locally inside the polygon
template <typename N, typename Coord>
bool Earcut<N, Coord>::locallyInside(const Node* a, const Node* b) {
    return area(a->prev, a, a->next) < 0 ? area(a, b, a->next) >= 0 && area(a, a->prev, b) >= 0
                                         : area(a, b, a->prev) < 0 || area(a, a->next, b) < 0;
}

// check if the middle Vertex of a polygon diagonal is inside the polygon
template <typename N, typename Coord>
bool Earcut<N, Coord>::middleInside(const Node* a, const Node* b) {
    const Node* p = a;
    bool inside = false;
    const WideCoord px2 = static_cast<WideCoord>(a->x) + static_cast<WideCoord>(b->x);
    const WideCoord py2 = static_cast<WideCoord>(a->y) + static_cast<WideCoord>(b->y);
    do {
        const bool aboveP = static_cast<WideCoord>(p->y) * WideCoord(2) > py2;
        const bool aboveNext = static_cast<WideCoord>(p->next->y) * WideCoord(2) > py2;
        if (aboveP != aboveNext && p->next->y != p->y) {
            const WideCoord dy = static_cast<WideCoord>(p->next->y) - static_cast<WideCoord>(p->y);
            const WideCoord lhs = (px2 - static_cast<WideCoord>(p->x) * WideCoord(2)) * dy;
            const WideCoord rhs = (static_cast<WideCoord>(p->next->x) - static_cast<WideCoord>(p->x)) *
                               (py2 - static_cast<WideCoord>(p->y) * WideCoord(2));
            if ((dy > 0 && lhs < rhs) || (dy < 0 && lhs > rhs)) inside = !inside;
        }
        p = p->next;
    } while (p != a);

    return inside;
}

// link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits
// polygon into two; if one belongs to the outer ring and another to a hole, it merges it into a
// single ring
template <typename N, typename Coord>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::splitPolygon(Node* a, Node* b) {
    Node* a2 = nodes->construct(a->i, a->x, a->y);
    Node* b2 = nodes->construct(b->i, b->x, b->y);
    Node* an = a->next;
    Node* bp = b->prev;

    a->next = b;
    b->prev = a;

    a2->next = an;
    an->prev = a2;

    b2->next = a2;
    a2->prev = b2;

    bp->next = b2;
    b2->prev = bp;

    return b2;
}

// create a node and util::optionally link it with previous one (in a circular doubly linked list)
template <typename N, typename Coord>
template <typename Point>
typename Earcut<N, Coord>::Node* Earcut<N, Coord>::insertNode(std::size_t i, const Point& pt, Node* last) {
    Node* p = nodes->construct(static_cast<N>(i), util::nth<0, Point>::get(pt), util::nth<1, Point>::get(pt));

    if (!last) {
        p->prev = p;
        p->next = p;

    } else {
        assert(last);
        p->next = last->next;
        p->prev = last;
        last->next->prev = p;
        last->next = p;
    }
    return p;
}

template <typename N, typename Coord>
void Earcut<N, Coord>::removeNode(Node* p) {
    p->next->prev = p->prev;
    p->prev->next = p->next;

    if (p->prevZ) p->prevZ->nextZ = p->nextZ;
    if (p->nextZ) p->nextZ->prevZ = p->prevZ;

    // keep the hole-bridge index's block bboxes covering the healed prev->next edge
    if (indexActive) growBlock(p->prev, p->next);
}
} // namespace detail

template <typename N = uint32_t, typename Polygon, typename Coord = void>
std::vector<N> earcut(const Polygon& poly) {
    using Point = typename Polygon::value_type::value_type;
    using CoordType = typename std::conditional<
        std::is_same<Coord, void>::value,
        typename std::decay<decltype(util::nth<0, Point>::get(std::declval<Point>()))>::type,
        Coord>::type;
    mapbox::detail::Earcut<N, CoordType> earcut;
    earcut(poly);
    return std::move(earcut.indices);
}

namespace detail {

// signed area of a triangle
template <typename T>
inline auto orient(T ax, T ay, T bx, T by, T cx, T cy) -> decltype((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

template <typename Point>
inline auto orient(const Point& a, const Point& b, const Point& c)
    -> decltype((util::nth<0, Point>::get(b) - util::nth<0, Point>::get(a)) *
                    (util::nth<1, Point>::get(c) - util::nth<1, Point>::get(a)) -
                (util::nth<1, Point>::get(b) - util::nth<1, Point>::get(a)) *
                    (util::nth<0, Point>::get(c) - util::nth<0, Point>::get(a))) {
    return (util::nth<0, Point>::get(b) - util::nth<0, Point>::get(a)) *
               (util::nth<1, Point>::get(c) - util::nth<1, Point>::get(a)) -
           (util::nth<1, Point>::get(b) - util::nth<1, Point>::get(a)) *
               (util::nth<0, Point>::get(c) - util::nth<0, Point>::get(a));
}

template <typename T>
inline typename std::enable_if<std::is_integral<T>::value, bool>::type inCircleResult(const T& det, const T&) {
    return det <= T(0);
}

template <typename T>
inline typename std::enable_if<!std::is_integral<T>::value, bool>::type inCircleResult(const T& det, const T& s) {
    return det <= static_cast<T>(1e-13) * s * s;
}

// Whether p is inside or exactly on the circumcircle of triangle (a, b, c). Sign is negated vs the
// usual predicate to match earcut's CCW winding -- the standard sign would build the anti-Delaunay
// mesh. Cocircular quads are legal ties, so refine only flips when this returns false.
template <typename T>
inline bool inCircle(T ax, T ay, T bx, T by, T cx, T cy, T px, T py) {
    const auto dx = ax - px;
    const auto dy = ay - py;
    const auto ex = bx - px;
    const auto ey = by - py;
    const auto fx = cx - px;
    const auto fy = cy - py;
    const auto ap = dx * dx + dy * dy;
    const auto bp = ex * ex + ey * ey;
    const auto cp = fx * fx + fy * fy;
    // A near-cocircular quad is a legal Delaunay tie, but roundoff can flag both an edge and its
    // flip as illegal, cascading into an endless flip loop (#205) -- so treat a determinant within
    // a small margin of zero as a tie. The determinant's worst-case roundoff error is provably
    // below 9e-16 (ap + bp + cp)^2 (Shewchuk-style bound), so the margin guarantees every executed
    // flip is illegal in exact arithmetic, and Lawson flipping always terminates.
    const auto s = ap + bp + cp;
    const auto det = dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx);
    return inCircleResult(det, s);
}

template <typename Point>
inline bool inCircle(const Point& a, const Point& b, const Point& c, const Point& p) {
    return inCircle(util::nth<0, Point>::get(a), util::nth<1, Point>::get(a), util::nth<0, Point>::get(b),
                    util::nth<1, Point>::get(b), util::nth<0, Point>::get(c), util::nth<1, Point>::get(c),
                    util::nth<0, Point>::get(p), util::nth<1, Point>::get(p));
}

// next half-edge within the same triangle
inline std::size_t nextHalfEdge(std::size_t edge) {
    return edge - edge % 3 + (edge + 1) % 3;
}

template <typename Polygon>
inline const typename Polygon::value_type::value_type& polygonPoint(const Polygon& polygon, std::size_t index) {
    for (const auto& ring : polygon) {
        if (index < ring.size()) {
            return ring[index];
        }
        index -= ring.size();
    }
    assert(false);
    return polygon[0][0];
}

} // namespace detail

template <typename N, typename Polygon>

// Refine a triangulation toward the constrained Delaunay triangulation by legalizing every
// interior edge in place with Lawson flips -- maximizing the minimum angle and removing most
// slivers. An optional post-pass for earcut output, or any manifold triangle-index array
// indexing into coords. Adapted from delaunator's edge legalization.
//
// Uses non-robust predicates: float input is fine, and the worst case is a not-quite-Delaunay
// edge, never an invalid mesh.
void refine(std::vector<N>& triangles, const Polygon& poly) {
    const std::size_t n = triangles.size();
    if (n < 6) return;

    std::vector<int32_t> halfEdges(n, -1);
    std::vector<int32_t> hashTable;
    std::vector<uint32_t> hashStamp;
    std::vector<uint8_t> edgeStamp(n, 0);
    std::vector<int32_t> edgeStack;
    edgeStack.reserve(n);

    // Build half-edge twins with an undirected-edge hash; consumed slots mark linked pairs. As each
    // pair is linked we seed the stack with one representative (s, the earlier-inserted edge) -- this
    // fuses the initial "push every interior edge" pass into the build, saving a full O(n) scan.
    // edgeStamp is all-zero here (balanced push/pop leaves it clean) and each pair links once, so
    // the seed write needs no dedup guard.
    std::size_t size = 1;
    while (size < n * 4) size <<= 1;
    hashTable.assign(size, 0);
    hashStamp.assign(size, 0);
    const uint32_t generation = 1;
    const std::size_t hashMask = size - 1;

    for (std::size_t edge = 0; edge < n; edge++) {
        const N a = triangles[edge];
        const N b = triangles[detail::nextHalfEdge(edge)];
        const N lo = a < b ? a : b;
        const N hi = a < b ? b : a;
        std::size_t hash =
            (static_cast<std::size_t>(lo) * 0x9e3779b1u ^ static_cast<std::size_t>(hi) * 0x85ebca6bu) & hashMask;

        while (hashStamp[hash] == generation) {
            const int32_t sibling = hashTable[hash];
            // -1 marks a consumed slot (a pair already linked) -- skip past it
            if (sibling != -1) {
                const N siblingA = triangles[static_cast<std::size_t>(sibling)];
                const N siblingB = triangles[detail::nextHalfEdge(static_cast<std::size_t>(sibling))];
                if ((siblingA == lo && siblingB == hi) || (siblingA == hi && siblingB == lo)) {
                    halfEdges[edge] = sibling;
                    halfEdges[static_cast<std::size_t>(sibling)] = static_cast<int32_t>(edge);
                    hashTable[hash] = -1;
                    edgeStamp[static_cast<std::size_t>(sibling)] = 1;
                    edgeStack.push_back(sibling);
                    break;
                }
            }
            hash = (hash + 1) & hashMask;
        }
        if (hashStamp[hash] != generation) {
            // first occurrence: insert
            hashTable[hash] = static_cast<int32_t>(edge);
            hashStamp[hash] = generation;
        }
    }

    while (!edgeStack.empty()) {
        const int32_t a = edgeStack.back();
        edgeStack.pop_back();
        edgeStamp[static_cast<std::size_t>(a)] = 0;
        const int32_t b = halfEdges[static_cast<std::size_t>(a)];
        if (b == -1) continue;

        const std::size_t aEdge = static_cast<std::size_t>(a);
        const std::size_t bEdge = static_cast<std::size_t>(b);
        const std::size_t a0 = aEdge - aEdge % 3;
        const std::size_t b0 = bEdge - bEdge % 3;
        const std::size_t ar = a0 + (aEdge + 2) % 3;
        const std::size_t al = a0 + (aEdge + 1) % 3;
        const std::size_t bl = b0 + (bEdge + 2) % 3;
        const std::size_t br = b0 + (bEdge + 1) % 3;
        const N p0 = triangles[ar];
        const N pr = triangles[aEdge];
        const N pl = triangles[al];
        const N p1 = triangles[bl];

        const auto& p0Point = detail::polygonPoint(poly, static_cast<std::size_t>(p0));
        const auto& prPoint = detail::polygonPoint(poly, static_cast<std::size_t>(pr));
        const auto& plPoint = detail::polygonPoint(poly, static_cast<std::size_t>(pl));
        const auto& p1Point = detail::polygonPoint(poly, static_cast<std::size_t>(p1));

        // Test inCircle first: most interior edges are already Delaunay (inCircle true -> no flip),
        // so this short-circuits before the two convexity orients on the common path. The quad must
        // also be convex (both new triangles CCW) -- flipping a reflex quad would push a triangle
        // outside the polygon. Boundary/hole edges need no guard -- they self-protect via he == -1.
        if (!detail::inCircle(p0Point, prPoint, plPoint, p1Point) && detail::orient(p0Point, prPoint, p1Point) > 0 &&
            detail::orient(p0Point, p1Point, plPoint) > 0) {
            triangles[aEdge] = p1;
            triangles[bEdge] = p0;

            const int32_t hbl = halfEdges[bl];
            const int32_t har = halfEdges[ar];
            halfEdges[aEdge] = hbl;
            if (hbl != -1) halfEdges[static_cast<std::size_t>(hbl)] = a;
            halfEdges[bEdge] = har;
            if (har != -1) halfEdges[static_cast<std::size_t>(har)] = b;
            halfEdges[ar] = static_cast<int32_t>(bl);
            halfEdges[bl] = static_cast<int32_t>(ar);

            // re-check the quad's four outer edges; skip boundary edges (he == -1) and any
            // already queued (edgeStamp), which also keeps the stack bounded by n.
            if (hbl != -1 && edgeStamp[aEdge] == 0) {
                edgeStamp[aEdge] = 1;
                edgeStack.push_back(a);
            }
            if (har != -1 && edgeStamp[bEdge] == 0) {
                edgeStamp[bEdge] = 1;
                edgeStack.push_back(b);
            }
            if (halfEdges[al] != -1 && edgeStamp[al] == 0) {
                edgeStamp[al] = 1;
                edgeStack.push_back(static_cast<int32_t>(al));
            }
            if (halfEdges[br] != -1 && edgeStamp[br] == 0) {
                edgeStamp[br] = 1;
                edgeStack.push_back(static_cast<int32_t>(br));
            }
        }
    }
}
} // namespace mapbox
