#pragma once

#include <vector>
#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>

namespace mapbox {

namespace util {

template <std::size_t I, typename T> struct nth {
    inline static typename std::tuple_element<I, T>::type
    get(const T &t) { return std::get<I>(t); };
};

}

template <typename Coord, typename N = uint32_t>
class Earcut {
public:
    using Vertex = std::array<Coord, 2>;

    using Indices = std::vector<N>;
    Indices indices;

    using Vertices = std::vector<Vertex>;
    Vertices vertices;

    template <typename Polygon>
    void operator()(const Polygon &points);

private:
    struct Node {
        Node() = default;
        Node(const Node &) = delete;
        Node &operator=(const Node &) = delete;
        Node(Node &&) = default;
        Node &operator=(Node &&) = default;

        // previous and next vertice nodes in a polygon ring
        N prev = 0, next = 0;

        // z-order curve value
        int32_t z = 0;

        // previous and next nodes in z-order
        N prevZ = 0, nextZ = 0;
    };


private:
    void discardUnusedVertices();
    template <typename Ring> N linkedList(const Ring &points, const bool clockwise);
    N filterPoints(N start, N end = 0);
    void earcutLinked(N ear, int pass = 0);
    bool isEar(N ear);
    N cureLocalIntersections(N start);
    void splitEarcut(N start);
    template <typename Polygon> N eliminateHoles(const Polygon &points, N outerNode);
    void eliminateHole(N holeNode, N outerNode);
    N findHoleBridge(N holeNode, N outerNode);
    void indexCurve(N start);
    N sortLinked(N list);
    int32_t zOrder(const double x_, const double y_);
    N getLeftmost(N start);
    bool isValidDiagonal(N a, N b);
    int8_t orient(const Vertex &p, const Vertex &q, const Vertex &r) const;
    bool equals(const Vertex &p1, const Vertex &p2);
    bool intersects(const Vertex &p1, const Vertex &q1, const Vertex &p2, const Vertex &q2);
    bool intersectsPolygon(const N start, const Vertex &a, const Vertex &b);
    bool locallyInside(N a, N b);
    bool middleInside(N start, const Vertex &a, const Vertex &b);
    N splitPolygon(N a, N b);
    template <typename Point> N insertNode(const Point &p, N last);
    template <typename Point> N createNode(const Point &p);

    bool hashing;
    Coord minX, maxX;
    Coord minY, maxY;
    double size;

    inline const Vertex &v(N i) const { return vertices[i]; }

    std::vector<Node> nodes;
    inline Node &n(N i) { return nodes[i]; }

    std::vector<N> used;
};

template <typename Coord, typename N> template <typename Polygon>
void Earcut<Coord, N>::operator()(const Polygon &points) {
    // reset
    indices.clear();
    vertices.clear();
    nodes.clear();
    used.clear();

    // null element
    vertices.emplace_back();
    nodes.emplace_back();
    used.push_back(0);

    auto outerNode = filterPoints(linkedList(points[0], true));
    if (!outerNode) return;

    N node;
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
        node = n(outerNode).next;
        minX = maxX = v(node)[0];
        minY = maxY = v(node)[1];
        do {
            x = v(node)[0];
            y = v(node)[1];
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            node = n(node).next;
        } while (node != outerNode);

        // minX, minY and size are later used to transform coords into integers for z-order calculation
        size = std::max(maxX - minX, maxY - minY);
    }

    if (points.size() > 1) outerNode = eliminateHoles(points, outerNode);

    earcutLinked(outerNode);

    discardUnusedVertices();
}

// removes unused vertices from the final set of vertices
template <typename Coord, typename N>
void Earcut<Coord, N>::discardUnusedVertices() {
    size_t dst = 0;
    for (size_t src = 0; src < vertices.size(); ++src) {
        if (used[src] > 0) {
            // This vertex is used. Move to the next free spot.
            vertices[dst] = std::move(vertices[src]);
            used[src] = dst++;
        } else {
            used[src] = dst;
        }
    }

    // remove trailing elements
    vertices.resize(dst);

    // change the triangle indices to the new compressed scheme
    std::transform(indices.begin(), indices.end(), indices.begin(), [&](N n) {
        return used[n];
    });
}

// create a circular doubly linked list from polygon points in the specified winding order
template <typename Coord, typename N> template <typename Ring>
N Earcut<Coord, N>::linkedList(const Ring &points,
                                       const bool clockwise) {
    using Point = typename Ring::value_type;
    double sum = 0;
    const int len = points.size();
    int i, j;
    Point p1, p2;
    N last = 0;

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
        for (i = 0; i < len; i++) last = insertNode(points[i], last);
    } else {
        for (i = len - 1; i >= 0; i--) last = insertNode(points[i], last);
    }

    return last;
}

// eliminate colinear or duplicate points
template <typename Coord, typename N>
N Earcut<Coord, N>::filterPoints(const N start, N end) {
    if (!end) end = start;

    auto node = start;
    bool again;
    do {
        again = false;

        if (equals(v(node), v(n(node).next)) || orient(v(n(node).prev), v(node), v(n(node).next)) == 0) {

            // remove node
            n(n(node).prev).next = n(node).next;
            n(n(node).next).prev = n(node).prev;

            if (n(node).prevZ) n(n(node).prevZ).nextZ = n(node).nextZ;
            if (n(node).nextZ) n(n(node).nextZ).prevZ = n(node).prevZ;

            node = end = n(node).prev;

            if (node == n(node).next) return 0;
            again = true;

        } else {
            node = n(node).next;
        }
    } while (again || node != end);

    return end;
}

// main ear slicing loop which triangulates a polygon (given as a linked list)
template <typename Coord, typename N>
void Earcut<Coord, N>::earcutLinked(N ear, int pass) {
    if (!ear) return;

    // interlink polygon nodes in z-order
    if (!pass && hashing) indexCurve(ear);

    N stop = ear;
    N prev, next;

    int iterations = 0;

    // iterate through ears, slicing them one by one
    while (n(ear).prev != n(ear).next) {
        iterations++;
        prev = n(ear).prev;
        next = n(ear).next;

        const auto isEarVal = isEar(ear);
        if (isEarVal) {
            // cut off the triangle
            indices.emplace_back(prev);
            indices.emplace_back(ear);
            indices.emplace_back(next);
            used[prev] = used[ear] = used[next] = 1;

            // remove ear node
            n(next).prev = prev;
            n(prev).next = next;

            if (n(ear).prevZ) n(n(ear).prevZ).nextZ = n(ear).nextZ;
            if (n(ear).nextZ) n(n(ear).nextZ).prevZ = n(ear).prevZ;

            // skipping the next vertice leads to less sliver triangles
            ear = n(next).next;
            stop = n(next).next;

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
bool Earcut<Coord, N>::isEar(N ear) {
    const auto &a = v(n(ear).prev);
    const auto &b = v(ear);
    const auto &c = v(n(ear).next);

    const auto ax = a[0], bx = b[0], cx = c[0];
    const auto ay = a[1], by = b[1], cy = c[1];

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

    Vertex p;
    typename Vertex::value_type px;
    typename Vertex::value_type py;
    Coord s, t, k;
    N node;

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
        node = n(ear).nextZ;

        while (node && n(node).z <= maxZ) {
            p = v(node);
            node = n(node).nextZ;
            if (p == a || p == c) continue;

            px = p[0];
            py = p[1];

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
        node = n(ear).prevZ;

        while (node && n(node).z >= minZ) {
            p = v(node);
            node = n(node).prevZ;
            if (p == a || p == c) continue;

            px = p[0];
            py = p[1];

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
        node = n(n(ear).next).next;

        while (node != n(ear).prev) {
            p = v(node);

            node = n(node).next;
            px = p[0];
            py = p[1];

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
N Earcut<Coord, N>::cureLocalIntersections(N start) {
    N node = start;
    do {
        N a = n(node).prev;
        N b = n(n(node).next).next;

        // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2])
        if (intersects(v(a), v(node), v(n(node).next), v(b)) && locallyInside(a, b) && locallyInside(b, a)) {
            indices.emplace_back(a);
            indices.emplace_back(node);
            indices.emplace_back(b);
            used[a] = used[node] = used[b] = 1;

            // remove two nodes involved
            n(a).next = b;
            n(b).prev = a;

            N az = n(node).prevZ;
            N bz = n(node).nextZ ? n(n(node).nextZ).nextZ : 0;

            if (az) n(az).nextZ = bz;
            if (bz) n(bz).prevZ = az;

            node = start = b;
        }
        node = n(node).next;
    } while (node != start);

    return node;
}

// try splitting polygon into two and triangulate them independently
template <typename Coord, typename N>
void Earcut<Coord, N>::splitEarcut(N start) {
    // look for a valid diagonal that divides the polygon into two
    N a = start;
    do {
        N b = n(n(a).next).next;
        while (b != n(a).prev) {
            if (isValidDiagonal(a, b)) {
                // split the polygon in two by the diagonal
                auto c = splitPolygon(a, b);

                // filter colinear points around the cuts
                a = filterPoints(a, n(a).next);
                c = filterPoints(c, n(c).next);

                // run earcut on each half
                earcutLinked(a);
                earcutLinked(c);
                return;
            }
            b = n(b).next;
        }
        a = n(a).next;
    } while (a != start);
}

// link every hole into the outer loop, producing a single-ring polygon without holes
template <typename Coord, typename N> template <typename Polygon>
N Earcut<Coord, N>::eliminateHoles(const Polygon &points, N outerNode) {const auto len = points.size();

    std::vector<N> queue;
    for (size_t i = 1; i < len; i++) {
        auto list = filterPoints(linkedList(points[i], false));
        if (list) {
            queue.push_back(getLeftmost(list));
        }
    }
    std::sort(queue.begin(), queue.end(), [this](const N a, const N b) {
        return v(a)[0] < v(b)[0];
    });

    // process holes from left to right
    for (size_t i = 0; i < queue.size(); i++) {
        eliminateHole(queue[i], outerNode);
        outerNode = filterPoints(outerNode, n(outerNode).next);
    }

    return outerNode;
}

// find a bridge between vertices that connects hole with an outer ring and and link it
template <typename Coord, typename N>
void Earcut<Coord, N>::eliminateHole(N holeNode, N outerNode) {
    outerNode = findHoleBridge(holeNode, outerNode);
    if (outerNode) {
        const N b = splitPolygon(outerNode, holeNode);
        filterPoints(b, n(b).next);
    }
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon
template <typename Coord, typename N>
N Earcut<Coord, N>::findHoleBridge(N const holeNode, N const outerNode) {N node = outerNode;
    const Vertex &p = v(holeNode);
    auto px = p[0];
    auto py = p[1];
    auto qMax = -std::numeric_limits<double>::infinity();
    N mNode = 0;

    // find a segment intersected by a ray from the hole's leftmost Vertex to the left;
    // segment's endpoint with lesser x will be potential connection Vertex
    do {
        auto &a = v(node);
        auto &b = v(n(node).next);

        if (py <= a[1] && py >= b[1]) {
          auto qx = double(a[0]) +
                    double(py - a[1]) *
                        double(b[0] - a[0]) /
                        double(b[1] - a[1]);
          if (qx <= px && qx > qMax) {
            qMax = qx;
            mNode = a[0] < b[0] ? node : n(node).next;
          }
        }
        node = n(node).next;
    } while (node != outerNode);

    if (!mNode) return 0;

    // look for points strictly inside the triangle of hole Vertex, segment intersection and endpoint;
    // if there are no points found, we have a valid connection;
    // otherwise choose the Vertex of the minimum angle with the ray as connection Vertex

    const double bx = v(mNode)[0];
    const double by = v(mNode)[1];
    const double pbd = px * by - py * bx;
    const double pcd = px * py - py * qMax;
    const double cpy = py - py;
    const double pcx = px - qMax;
    const double pby = py - by;
    const double bpx = bx - px;
    const double A = pbd - pcd - (qMax * by - py * bx);
    const int sign = A <= 0 ? -1 : 1;
    const N stop = mNode;
    double tanMin = std::numeric_limits<double>::infinity();
    double s = 0;
    double t = 0;
    double tanCur = 0;

    Coord mx;
    Coord my;
    Coord amx;

    node = n(mNode).next;

    while (node != stop) {

        mx = v(node)[0];
        my = v(node)[1];
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

        node = n(node).next;
    }

    return mNode;
}

// interlink polygon nodes in z-order
template <typename Coord, typename N>
void Earcut<Coord, N>::indexCurve(N start) {
    N node = start;

    do {
        n(node).z = n(node).z ? n(node).z : zOrder(v(node)[0], v(node)[1]);
        n(node).prevZ = n(node).prev;
        n(node).nextZ = n(node).next;
        node = n(node).next;
    } while (node != start);

    n(n(node).prevZ).nextZ = 0;
    n(node).prevZ = 0;

    sortLinked(node);
}

// Simon Tatham's linked list merge sort algorithm
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
template <typename Coord, typename N>
N Earcut<Coord, N>::sortLinked(N list) {
    N p, q, e, tail;
    int i, numMerges, pSize, qSize;
    int inSize = 1;

    while (true) {
        p = list;
        list = 0;
        tail = 0;
        numMerges = 0;

        while (p) {
            numMerges++;
            q = p;
            pSize = 0;
            for (i = 0; i < inSize; i++) {
                pSize++;
                q = n(q).nextZ;
                if (!q) break;
            }

            qSize = inSize;

            while (pSize > 0 || (qSize > 0 && q)) {

                if (pSize == 0) {
                    e = q;
                    q = n(q).nextZ;
                    qSize--;
                } else if (qSize == 0 || !q) {
                    e = p;
                    p = n(p).nextZ;
                    pSize--;
                } else if (n(p).z <= n(q).z) {
                    e = p;
                    p = n(p).nextZ;
                    pSize--;
                } else {
                    e = q;
                    q = n(q).nextZ;
                    qSize--;
                }

                if (tail) n(tail).nextZ = e;
                else list = e;

                n(e).prevZ = tail;
                tail = e;
            }

            p = q;
        }

        n(tail).nextZ = 0;

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
N Earcut<Coord, N>::getLeftmost(const N start) {
    N node = start;
    N leftmost = start;
    do {
        if (v(node)[0] < v(leftmost)[0]) leftmost = node;
        node = n(node).next;
    } while (node != start);

    return leftmost;
}

// check if a diagonal between two polygon nodes is valid (lies in polygon interior)
template <typename Coord, typename N>
bool Earcut<Coord, N>::isValidDiagonal(const N a, const N b) {
    return !intersectsPolygon(a, v(a), v(b)) &&
           locallyInside(a, b) && locallyInside(b, a) &&
           middleInside(a, v(a), v(b));
}

// winding order of triangle formed by 3 given points
template <typename Coord, typename N>
int8_t Earcut<Coord, N>::orient(const Vertex &p, const Vertex &q, const Vertex &r) const {
    const auto o = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    return o > 0 ? 1 :
           o < 0 ? -1 : 0;
}

// check if two points are equal
template <typename Coord, typename N>
bool Earcut<Coord, N>::equals(const Vertex &p1, const Vertex &p2) {
    return p1[0] == p2[0] && p1[1] == p2[1];
}

// check if two segments intersect
template <typename Coord, typename N>
bool Earcut<Coord, N>::intersects(const Vertex &p1, const Vertex &q1, const Vertex &p2, const Vertex &q2) {
    return orient(p1, q1, p2) != orient(p1, q1, q2) &&
           orient(p2, q2, p1) != orient(p2, q2, q1);
}

// check if a polygon diagonal intersects any polygon segments
template <typename Coord, typename N>
bool Earcut<Coord, N>::intersectsPolygon(const N start, const Vertex &a, const Vertex &b) {
    N node = start;
    do {
        auto &p1 = v(node);
        auto &p2 = v(n(node).next);

        if (p1 != a && p2 != a && p1 != b && p2 != b && intersects(p1, p2, a, b)) return true;

        node = n(node).next;
    } while (node != start);

    return false;
}

// check if a polygon diagonal is locally inside the polygon
template <typename Coord, typename N>
bool Earcut<Coord, N>::locallyInside(const N a, const N b) {
    return orient(v(n(a).prev), v(a), v(n(a).next)) == -1 ?
        orient(v(a), v(b), v(n(a).next)) != -1 && orient(v(a), v(n(a).prev), v(b)) != -1 :
        orient(v(a), v(b), v(n(a).prev)) == -1 || orient(v(a), v(n(a).next), v(b)) == -1;
}

// check if the middle Vertex of a polygon diagonal is inside the polygon
template <typename Coord, typename N>
bool Earcut<Coord, N>::middleInside(const N start, const Vertex &a, const Vertex &b) {
    N node = start;
    bool inside = false;
    auto px = double(a[0] + b[0]) / 2;
    auto py = double(a[1] + b[1]) / 2;
    do {
        auto &p1 = v(node);
        auto &p2 = v(n(node).next);

        if (((p1[1] > py) != (p2[1] > py)) &&
            (px < (p2[0] - p1[0]) * (py - p1[1]) / (p2[1] - p1[1]) + p1[0])) inside = !inside;

        node = n(node).next;
    } while (node != start);

    return inside;
}

// link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits
// polygon into two; if one belongs to the outer ring and another to a hole, it merges it into a
// single ring
template <typename Coord, typename N>
N Earcut<Coord, N>::splitPolygon(const N a, const N b) {
    const auto a2 = createNode(v(a));
    const auto b2 = createNode(v(b));
    const auto an = n(a).next;
    const auto bp = n(b).prev;

    n(a).next = b;
    n(b).prev = a;

    n(a2).next = an;
    n(a2).next = an;
    n(an).prev = a2;

    n(b2).next = a2;
    n(a2).prev = b2;

    n(bp).next = b2;
    n(b2).prev = bp;

    return b2;
}

template <typename Coord, typename N> template <typename Point>
N Earcut<Coord, N>::createNode(const Point &p) {
    N i = nodes.size();
    assert(vertices.size() == i);
    assert(used.size() == i);
    nodes.emplace_back();
    vertices.emplace_back(Vertex {{ Coord(util::nth<0, Point>::get(p)),
                                    Coord(util::nth<1, Point>::get(p)) }});
    used.push_back(false);
    return i;

}

// create a node and util::optionally link it with previous one (in a circular doubly linked list)
template <typename Coord, typename N> template <typename Point>
N Earcut<Coord, N>::insertNode(const Point &p, N last) {
    N node = createNode(p);

    if (!last) {
        n(node).prev = node;
        n(node).next = node;

    } else {
        assert(last);
        n(node).next = n(last).next;
        n(node).prev = last;
        n(n(last).next).prev = node;
        n(last).next = node;
    }
    return node;
}

}
