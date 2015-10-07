#pragma once

#include <vector>
#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>
#include <memory>

// http://stackoverflow.com/questions/17000542/
#define BOOST_POOL_NO_MT       // disable multi-threading
#define BOOST_THREAD_MUTEX_HPP // define the #include-guard to disable the header
#include <boost/pool/object_pool.hpp>

namespace mapbox {

namespace util {

template <std::size_t I, typename T> struct nth {
    inline static typename std::tuple_element<I, T>::type
    get(const T& t) { return std::get<I>(t); };
};

}

template <typename Coord, typename N = uint32_t>
class Earcut {
public:
    using Vertex = std::array<Coord, 2>;

    using Indices = std::vector<N>;
    Indices indices;

    N vertices = 0;

    template <typename Polygon>
    void operator()(const Polygon& points);

private:
    struct Node {
        Node(N index, const Vertex& vertex) : i(index), v(vertex) {}
        Node(const Node&) = delete;
        Node& operator=(const Node&) = delete;
        Node(Node&&) = delete;
        Node& operator=(Node&&) = delete;

        const N i;
        const Vertex v;

        // previous and next vertice nodes in a polygon ring
        Node* prev = nullptr;
        Node* next = nullptr;

        // z-order curve value
        int32_t z = 0;

        // previous and next nodes in z-order
        Node* prevZ = nullptr;
        Node* nextZ = nullptr;
    };

    template <typename Ring> Node* linkedList(const Ring& points, const bool clockwise);
    Node* filterPoints(Node* start, Node* end = nullptr);
    void earcutLinked(Node* ear, int pass = 0);
    bool isEar(Node* ear);
    Node* cureLocalIntersections(Node* start);
    void splitEarcut(Node* start);
    template <typename Polygon> Node* eliminateHoles(const Polygon& points, Node* outerNode);
    void eliminateHole(Node* holeNode, Node* outerNode);
    Node* findHoleBridge(Node* holeNode, Node* outerNode);
    void indexCurve(Node* start);
    Node* sortLinked(Node* list);
    int32_t zOrder(const double x_, const double y_);
    Node* getLeftmost(Node* start);
    bool isValidDiagonal(Node* a, Node* b);
    int8_t orient(const Vertex& p, const Vertex& q, const Vertex& r) const;
    bool equals(const Vertex& p1, const Vertex& p2);
    bool intersects(const Vertex& p1, const Vertex& q1, const Vertex& p2, const Vertex& q2);
    bool intersectsPolygon(Node* start, Node* a, Node* b);
    bool locallyInside(Node* a, Node* b);
    bool middleInside(Node* start, const Vertex& a, const Vertex& b);
    Node* splitPolygon(Node* a, Node* b);
    template <typename Point> Node* insertNode(N i, const Point& p, Node* last);

    bool hashing;
    Coord minX, maxX;
    Coord minY, maxY;
    double size;

    std::unique_ptr<boost::object_pool<Node>> nodes;
};

template <typename Coord, typename N> template <typename Polygon>
void Earcut<Coord, N>::operator()(const Polygon& points) {
    // reset
    indices.clear();
    vertices = 0;
    nodes = std::make_unique<boost::object_pool<Node>>();

    auto outerNode = filterPoints(linkedList(points[0], true));
    if (!outerNode) return;

    Coord x;
    Coord y;
    size = 0;
    int threshold = 80;

    for (size_t i = 0; threshold >= 0 && i < points.size(); i++) {
        threshold -= points[i].size();
    }

    // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
    hashing = threshold < 0;
    if (hashing) {
        Node* node = outerNode->next;
        minX = maxX = node->v[0];
        minY = maxY = node->v[1];
        do {
            x = node->v[0];
            y = node->v[1];
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            node = node->next;
        } while (node != outerNode);

        // minX, minY and size are later used to transform coords into integers for z-order calculation
        size = std::max(maxX - minX, maxY - minY);
    }

    if (points.size() > 1) outerNode = eliminateHoles(points, outerNode);

    earcutLinked(outerNode);
}

// create a circular doubly linked list from polygon points in the specified winding order
template <typename Coord, typename N> template <typename Ring>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::linkedList(const Ring& points,
                                       const bool clockwise) {
    using Point = typename Ring::value_type;
    double sum = 0;
    const int len = points.size();
    int i, j;
    Point p1, p2;
    Node* last = nullptr;

    // calculate original winding order of a polygon ring
    for (i = 0, j = len - 1; i < len; j = i++) {
        p1 = points[i];
        p2 = points[j];
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
        for (i = len - 1; i >= 0; i--) last = insertNode(vertices + i, points[i], last);
    }

    vertices += len;

    return last;
}

// eliminate colinear or duplicate points
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::filterPoints(Node* start, Node* end) {
    if (!end) end = start;

    auto node = start;
    bool again;
    do {
        again = false;

        if (equals(node->v, node->next->v) || orient(node->prev->v, node->v, node->next->v) == 0) {

            // remove node
            node->prev->next = node->next;
            node->next->prev = node->prev;

            if (node->prevZ) node->prevZ->nextZ = node->nextZ;
            if (node->nextZ) node->nextZ->prevZ = node->prevZ;

            node = end = node->prev;

            if (node == node->next) return nullptr;
            again = true;

        } else {
            node = node->next;
        }
    } while (again || node != end);

    return end;
}

// main ear slicing loop which triangulates a polygon (given as a linked list)
template <typename Coord, typename N>
void Earcut<Coord, N>::earcutLinked(Node* ear, int pass) {
    if (!ear) return;

    // interlink polygon nodes in z-order
    if (!pass && hashing) indexCurve(ear);

    Node* stop = ear;
    Node* prev;
    Node* next;

    int iterations = 0;

    // iterate through ears, slicing them one by one
    while (ear->prev != ear->next) {
        iterations++;
        prev = ear->prev;
        next = ear->next;

        const auto isEarVal = isEar(ear);
        if (isEarVal) {
            // cut off the triangle
            indices.emplace_back(prev->i);
            indices.emplace_back(ear->i);
            indices.emplace_back(next->i);

            // remove ear node
            next->prev = prev;
            prev->next = next;

            if (ear->prevZ) ear->prevZ->nextZ = ear->nextZ;
            if (ear->nextZ) ear->nextZ->prevZ = ear->prevZ;

            // skipping the next vertice leads to less sliver triangles
            ear = next->next;
            stop = next->next;

            continue;
        }

        ear = next;

        // if we looped through the whole remaining polygon and can't find any more ears
        if (ear == stop) {
            // try filtering points and slicing again
            if (!pass) earcutLinked(filterPoints(ear), 1);

            // if this didn't work, try curing all small self-intersections locally
            else if (pass == 1) {
                ear = cureLocalIntersections(ear);
                earcutLinked(ear, 2);

            // as a last resort, try splitting the remaining polygon into two
            } else if (pass == 2) splitEarcut(ear);

            break;
        }
    }
}

// check whether a polygon node forms a valid ear with adjacent nodes
template <typename Coord, typename N>
bool Earcut<Coord, N>::isEar(Node* ear) {
    const auto a = ear->prev;
    const auto b = ear;
    const auto c = ear->next;

    const auto ax = a->v[0], bx = b->v[0], cx = c->v[0];
    const auto ay = a->v[1], by = b->v[1], cy = c->v[1];

    const auto abd = ax * by - ay * bx;
    const auto acd = ax * cy - ay * cx;
    const auto cbd = cx * by - cy * bx;
    const auto A = abd - acd - cbd;

    if (A <= 0) return false; // reflex, can't be an ear

    // now make sure we don't have other points inside the potential ear;
    // the code below is a bit verbose and repetitive but this is done for performance
    const auto cay = cy - ay;
    const auto acx = ax - cx;
    const auto aby = ay - by;
    const auto bax = bx - ax;

    N i;
    Vertex v;
    typename Vertex::value_type px;
    typename Vertex::value_type py;
    double s, t, k;
    Node* node;

    // if we use z-order curve hashing, iterate through the curve
    if (hashing) {

        // triangle bbox; min & max are calculated like this for speed
        auto const minTX = ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx);
        auto const minTY = ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy);
        auto const maxTX = ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx);
        auto const maxTY = ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy);

        // z-order range for the current triangle bbox;
        auto const minZ = zOrder(minTX, minTY);
        auto const maxZ = zOrder(maxTX, maxTY);

        // first look for points inside the triangle in increasing z-order
        node = ear->nextZ;

        while (node && node->z <= maxZ) {
            i = node->i;
            v = node->v;
            node = node->nextZ;
            if (i == a->i || i == c->i) continue;

            px = v[0];
            py = v[1];

            s = cay * px + acx * py - acd;
            if (s >= 0) {
                t = aby * px + bax * py + abd;
                if (t >= 0) {
                    k = A - s - t;
                    if ((k >= 0) && ((s && t) || (s && k) || (t && k))) return false;
                }
            }
        }

        // then look for points in decreasing z-order
        node = ear->prevZ;

        while (node && node->z >= minZ) {
            i = node->i;
            v = node->v;
            node = node->prevZ;
            if (i == a->i || i == c->i) continue;

            px = v[0];
            py = v[1];

            s = cay * px + acx * py - acd;
            if (s >= 0) {
                t = aby * px + bax * py + abd;
                if (t >= 0) {
                    k = A - s - t;
                    if ((k >= 0) && ((s && t) || (s && k) || (t && k))) return false;
                }
            }
        }

    // if we don't use z-order curve hash, simply iterate through all other points
    } else {
        node = ear->next->next;

        while (node != ear->prev) {
            i = node->i;
            v = node->v;
            node = node->next;

            px = v[0];
            py = v[1];

            s = cay * px + acx * py - acd;
            if (s >= 0) {
                t = aby * px + bax * py + abd;
                if (t >= 0) {
                    k = A - s - t;
                    if ((k >= 0) && ((s && t) || (s && k) || (t && k))) return false;
                }
            }
        }
    }

    return true;
}

// go through all polygon nodes and cure small local self-intersections
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::cureLocalIntersections(Node* start) {
    Node* node = start;
    do {
        Node* a = node->prev;
        Node* b = node->next->next;

        // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2])
        if (intersects(a->v, node->v, node->next->v, b->v) && locallyInside(a, b) && locallyInside(b, a)) {
            indices.emplace_back(a->i);
            indices.emplace_back(node->i);
            indices.emplace_back(b->i);

            // remove two nodes involved
            a->next = b;
            b->prev = a;

            Node* az = node->prevZ;
            Node* bz = node->nextZ ? node->nextZ->nextZ : 0;

            if (az) az->nextZ = bz;
            if (bz) bz->prevZ = az;

            node = start = b;
        }
        node = node->next;
    } while (node != start);

    return node;
}

// try splitting polygon into two and triangulate them independently
template <typename Coord, typename N>
void Earcut<Coord, N>::splitEarcut(Node* start) {
    // look for a valid diagonal that divides the polygon into two
    Node* a = start;
    do {
        Node* b = a->next->next;
        while (b != a->prev) {
            if (isValidDiagonal(a, b)) {
                // split the polygon in two by the diagonal
                auto c = splitPolygon(a, b);

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
template <typename Coord, typename N> template <typename Polygon>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::eliminateHoles(const Polygon& points, Node* outerNode) {const auto len = points.size();

    std::vector<Node*> queue;
    for (size_t i = 1; i < len; i++) {
        auto list = filterPoints(linkedList(points[i], false));
        if (list) {
            queue.push_back(getLeftmost(list));
        }
    }
    std::sort(queue.begin(), queue.end(), [this](const Node* a, const Node* b) {
        return a->v[0] < b->v[0];
    });

    // process holes from left to right
    for (size_t i = 0; i < queue.size(); i++) {
        eliminateHole(queue[i], outerNode);
        outerNode = filterPoints(outerNode, outerNode->next);
    }

    return outerNode;
}

// find a bridge between vertices that connects hole with an outer ring and and link it
template <typename Coord, typename N>
void Earcut<Coord, N>::eliminateHole(Node* holeNode, Node* outerNode) {
    outerNode = findHoleBridge(holeNode, outerNode);
    if (outerNode) {
        Node* b = splitPolygon(outerNode, holeNode);
        filterPoints(b, b->next);
    }
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::findHoleBridge(Node* const holeNode, Node* const outerNode) {
    Node* node = outerNode;
    auto px = holeNode->v[0];
    auto py = holeNode->v[1];
    auto qMax = -std::numeric_limits<double>::infinity();
    Node* mNode = nullptr;

    // find a segment intersected by a ray from the hole's leftmost Vertex to the left;
    // segment's endpoint with lesser x will be potential connection Vertex
    do {
        auto a = node;
        auto b = node->next;

        if (py <= a->v[1] && py >= b->v[1]) {
          auto qx = double(a->v[0]) +
                    double(py - a->v[1]) *
                        double(b->v[0] - a->v[0]) /
                        double(b->v[1] - a->v[1]);
          if (qx <= px && qx > qMax) {
            qMax = qx;
            mNode = a->v[0] < b->v[0] ? node : node->next;
          }
        }
        node = node->next;
    } while (node != outerNode);

    if (!mNode) return 0;

    // look for points strictly inside the triangle of hole Vertex, segment intersection and endpoint;
    // if there are no points found, we have a valid connection;
    // otherwise choose the Vertex of the minimum angle with the ray as connection Vertex

    const double bx = mNode->v[0];
    const double by = mNode->v[1];
    const double pbd = px * by - py * bx;
    const double pcd = px * py - py * qMax;
    const double cpy = py - py;
    const double pcx = px - qMax;
    const double pby = py - by;
    const double bpx = bx - px;
    const double A = pbd - pcd - (qMax * by - py * bx);
    const int sign = A <= 0 ? -1 : 1;
    const Node* stop = mNode;
    double tanMin = std::numeric_limits<double>::infinity();
    double s = 0;
    double t = 0;
    double tanCur = 0;

    double mx;
    double my;
    double amx;

    node = mNode->next;

    while (node != stop) {

        mx = node->v[0];
        my = node->v[1];
        amx = px - mx;

        if (amx >= 0 && mx >= bx) {
            s = (cpy * mx + pcx * my - pcd) * sign;
            if (s >= 0) {
                t = (pby * mx + bpx * my + pbd) * sign;

                if (t >= 0 && A * sign - s - t >= 0) {
                    tanCur = double(std::abs(double(py - my))) / amx; // tangential
                    if (tanCur < tanMin && locallyInside(node, holeNode)) {
                        mNode = node;
                        tanMin = tanCur;
                    }
                }
            }
        }

        node = node->next;
    }

    return mNode;
}

// interlink polygon nodes in z-order
template <typename Coord, typename N>
void Earcut<Coord, N>::indexCurve(Node* start) {
    Node* node = start;

    do {
        node->z = node->z ? node->z : zOrder(node->v[0], node->v[1]);
        node->prevZ = node->prev;
        node->nextZ = node->next;
        node = node->next;
    } while (node != start);

    node->prevZ->nextZ = nullptr;
    node->prevZ = nullptr;

    sortLinked(node);
}

// Simon Tatham's linked list merge sort algorithm
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::sortLinked(Node* list) {
    Node* p;
    Node* q;
    Node* e;
    Node* tail;
    int i, numMerges, pSize, qSize;
    int inSize = 1;

    while (true) {
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

                if (tail) tail->nextZ = e;
                else list = e;

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
template <typename Coord, typename N>
int32_t Earcut<Coord, N>::zOrder(const double x_, const double y_) {
    // coords are transformed into (0..1000) integer range
    int32_t x = 1000 * double(x_ - double(minX)) / size;
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;

    int32_t y = 1000 * double(y_ - double(minY)) / size;
    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;

    return x | (y << 1);
}

// find the leftmost node of a polygon ring
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::getLeftmost(Node* start) {
    Node* node = start;
    Node* leftmost = start;
    do {
        if (node->v[0] < leftmost->v[0]) leftmost = node;
        node = node->next;
    } while (node != start);

    return leftmost;
}

// check if a diagonal between two polygon nodes is valid (lies in polygon interior)
template <typename Coord, typename N>
bool Earcut<Coord, N>::isValidDiagonal(Node* a, Node* b) {
    return !intersectsPolygon(a, a, b) &&
           locallyInside(a, b) && locallyInside(b, a) &&
           middleInside(a, a->v, b->v);
}

// winding order of triangle formed by 3 given points
template <typename Coord, typename N>
int8_t Earcut<Coord, N>::orient(const Vertex& p, const Vertex& q, const Vertex& r) const {
    const auto o = double(q[1] - p[1]) * double(r[0] - q[0]) - double(q[0] - p[0]) * double(r[1] - q[1]);
    return o > 0 ? 1 :
           o < 0 ? -1 : 0;
}

// check if two points are equal
template <typename Coord, typename N>
bool Earcut<Coord, N>::equals(const Vertex& p1, const Vertex& p2) {
    return p1[0] == p2[0] && p1[1] == p2[1];
}

// check if two segments intersect
template <typename Coord, typename N>
bool Earcut<Coord, N>::intersects(const Vertex& p1, const Vertex& q1, const Vertex& p2, const Vertex& q2) {
    return orient(p1, q1, p2) != orient(p1, q1, q2) &&
           orient(p2, q2, p1) != orient(p2, q2, q1);
}

// check if a polygon diagonal intersects any polygon segments
template <typename Coord, typename N>
bool Earcut<Coord, N>::intersectsPolygon(Node* start, Node* a, Node* b) {
    Node* node = start;
    do {
        if (node->i != a->i &&
            node->next->i != a->i &&
            node->i != b->i &&
            node->next->i != b->i &&
            intersects(node->v, node->next->v, a->v, b->v)) return true;

        node = node->next;
    } while (node != start);

    return false;
}

// check if a polygon diagonal is locally inside the polygon
template <typename Coord, typename N>
bool Earcut<Coord, N>::locallyInside(Node* a, Node* b) {
    return orient(a->prev->v, a->v, a->next->v) == -1 ?
        orient(a->v, b->v, a->next->v) != -1 && orient(a->v, a->prev->v, b->v) != -1 :
        orient(a->v, b->v, a->prev->v) == -1 || orient(a->v, a->next->v, b->v) == -1;
}

// check if the middle Vertex of a polygon diagonal is inside the polygon
template <typename Coord, typename N>
bool Earcut<Coord, N>::middleInside(Node* start, const Vertex& a, const Vertex& b) {
    Node* node = start;
    bool inside = false;
    auto px = double(a[0] + b[0]) / 2;
    auto py = double(a[1] + b[1]) / 2;
    do {
        auto p1 = node->v;
        auto p2 = node->next->v;

        if (((p1[1] > py) != (p2[1] > py)) &&
            (px < double(p2[0] - p1[0]) * double(py - p1[1]) / double(p2[1] - p1[1]) + p1[0]))
                inside = !inside;

        node = node->next;
    } while (node != start);

    return inside;
}

// link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits
// polygon into two; if one belongs to the outer ring and another to a hole, it merges it into a
// single ring
template <typename Coord, typename N>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::splitPolygon(Node* a, Node* b) {
    const auto a2 = new (nodes->malloc()) Node(a->i, a->v);
    const auto b2 = new (nodes->malloc()) Node(b->i, b->v);
    const auto an = a->next;
    const auto bp = b->prev;

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
template <typename Coord, typename N> template <typename Point>
typename Earcut<Coord, N>::Node*
Earcut<Coord, N>::insertNode(N i, const Point& p, Node* last) {
    Vertex v({{ Coord(util::nth<0, Point>::get(p)),
                Coord(util::nth<1, Point>::get(p)) }});

    auto node = new (nodes->malloc()) Node(i, v);

    if (!last) {
        node->prev = node;
        node->next = node;

    } else {
        assert(last);
        node->next = last->next;
        node->prev = last;
        last->next->prev = node;
        last->next = node;
    }
    return node;
}

}
