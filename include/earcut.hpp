#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>

using Point = std::tuple<int, int>;
using Ring = std::vector<Point>;
using Polygon = std::vector<Ring>;
using Triangles = std::vector<Point>;

struct Node {
    Node(const Point &p_) : p(p_) {
    }

    using X = std::tuple_element<0, Point>::type;
    using Y = std::tuple_element<1, Point>::type;

    // vertex coordinates
    const Point p;

    // previous and next vertice nodes in a polygon ring
    Node *prev = nullptr;
    Node *next = nullptr;

    // z-order curve value
    uint16_t z = 0;

    // previous and next nodes in z-order
    Node *prevZ = nullptr;
    Node *nextZ = nullptr;
};

// create a node and optionally link it with previous one (in a circular doubly linked list)
Node *insertNode(const Point &point, Node *last) {
    auto node = new Node(point);

    if (!last) {
        node->prev = node;
        node->next = node;

    } else {
        node->next = last->next;
        node->prev = last;
        last->next->prev = node;
        last->next = node;
    }
    return node;
}

// create a circular doubly linked list from polygon points in the specified winding order
Node *linkedList(const Ring &points, const bool clockwise) {
    double sum = 0;
    const size_t len = points.size();
    Node *last = nullptr;

    Point p1, p2;

    // calculate original winding order of a polygon ring
    for (size_t i = 0, j = len - 1; i < len; j = i++) {
        p1 = points[i];
        p2 = points[j];
        sum += (std::get<0>(p2) - std::get<0>(p1)) * (std::get<1>(p1) + std::get<1>(p2));
    }

    // link points into circular doubly-linked list in the specified winding order
    if (clockwise == (sum > 0)) {
        for (size_t i = 0; i < len; i++) {
            last = insertNode(points[i], last);
        }
    } else {
        for (int i = len - 1; i >= 0; i--) {
            last = insertNode(points[i], last);
        }
    }

    return last;
}

// winding order of triangle formed by 3 given points
int orient(const Point &p, const Point &q, const Point &r) {
    int o = (std::get<1>(q) - std::get<1>(p)) * (std::get<0>(r) - std::get<0>(q)) -
            (std::get<0>(q) - std::get<0>(p)) * (std::get<1>(r) - std::get<1>(q));
    return o > 0 ? 1 : o < 0 ? -1 : 0;
}

// check if two points are equal
bool equals(const Point &p1, const Point &p2) {
    return std::get<0>(p1) == std::get<0>(p2) && std::get<1>(p1) == std::get<1>(p2);
}

// eliminate colinear or duplicate points
Node *filterPoints(Node *const start, Node *end = nullptr) {
    end = end ? end : start;

    auto node = start;
    bool again;
    do {
        again = false;

        if (equals(node->p, node->next->p) || orient(node->prev->p, node->p, node->next->p) == 0) {

            // remove node
            node->prev->next = node->next;
            node->next->prev = node->prev;

            if (node->prevZ) {
                node->prevZ->nextZ = node->nextZ;
            }
            if (node->nextZ) {
                node->nextZ->prevZ = node->prevZ;
            }

            node = end = node->prev;

            if (node == node->next) {
                // single node left
                return nullptr;
            }
            again = true;

        } else {
            node = node->next;
        }
    } while (again || node != end);

    return end;
}

// find the leftmost node of a polygon ring
Node *getLeftmost(Node *const start) {
    Node *node = start;
    Node *leftmost = start;
    do {
        if (std::get<0>(node->p) < std::get<0>(leftmost->p))
            leftmost = node;
        node = node->next;
    } while (node != start);

    return leftmost;
}

bool jsSortOrder(int value) {
    return value < 0;
}

bool compareX(const Node *const a, const Node *const b) {
    return jsSortOrder(std::get<0>(a->p) - std::get<0>(b->p));
}

// check if a polygon diagonal is locally inside the polygon
bool locallyInside(Node *const a, Node *const b) {
    return orient(a->prev->p, a->p, a->next->p) == -1
               ? orient(a->p, b->p, a->next->p) != -1 && orient(a->p, a->prev->p, b->p) != -1
               : orient(a->p, b->p, a->prev->p) == -1 || orient(a->p, a->next->p, b->p) == -1;
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon
Node *findHoleBridge(Node *const holeNode, Node *const outerNode) {
    Node *node = outerNode;
    const Point &p = holeNode->p;
    auto px = std::get<0>(p);
    auto py = std::get<1>(p);
    auto qMax = -std::numeric_limits<double>::infinity();
    Node *mNode = nullptr;

    // find a segment intersected by a ray from the hole's leftmost point to the left;
    // segment's endpoint with lesser x will be potential connection point
    do {
        auto &a = node->p;
        auto &b = node->next->p;

        if (py <= std::get<1>(a) && py >= std::get<1>(b)) {
          auto qx = double(std::get<0>(a)) +
                    double(py - std::get<1>(a)) *
                        double(std::get<0>(b) - std::get<0>(a)) /
                        double(std::get<1>(b) - std::get<1>(a));
          if (qx <= px && qx > qMax) {
            qMax = qx;
            mNode = std::get<0>(a) < std::get<0>(b) ? node : node->next;
          }
        }
        node = node->next;
    } while (node != outerNode);

    if (!mNode) return nullptr;

    // look for points strictly inside the triangle of hole point, segment intersection and endpoint;
    // if there are no points found, we have a valid connection;
    // otherwise choose the point of the minimum angle with the ray as connection point

    const double bx = std::get<0>(mNode->p);
    const double by = std::get<1>(mNode->p);
    const double pbd = px * by - py * bx;
    const double pcd = px * py - py * qMax;
    const double cpy = py - py;
    const double pcx = px - qMax;
    const double pby = py - by;
    const double bpx = bx - px;
    const double A = pbd - pcd - (qMax * by - py * bx);
    const int sign = A <= 0 ? -1 : 1;
    Node *const stop = mNode;
    double tanMin = std::numeric_limits<double>::infinity();
    double s = 0;
    double t = 0;
    double tanCur = 0;

    Node::X mx;
    Node::Y my;
    Node::X amx;

    node = mNode->next;

    while (node != stop) {

        mx = std::get<0>(node->p);
        my = std::get<1>(node->p);
        amx = px - mx;

        if (amx >= 0 && mx >= bx) {
            s = (cpy * mx + pcx * my - pcd) * sign;
            if (s >= 0) {
                t = (pby * mx + bpx * my + pbd) * sign;

                if (t >= 0 && A * sign - s - t >= 0) {
                    tanCur = std::abs(double(py - my)) / amx; // tangential
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

// link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits
// polygon into two; if one belongs to the outer ring and another to a hole, it merges it into a
// single ring
Node *splitPolygon(Node *const a, Node *const b) {
    Node *a2 = new Node(a->p);
    Node *b2 = new Node(b->p);
    Node *an = a->next;
    Node *bp = b->prev;

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

// find a bridge between vertices that connects hole with an outer ring and and link it
void eliminateHole(Node *const holeNode, Node *outerNode) {
    outerNode = findHoleBridge(holeNode, outerNode);
    if (outerNode) {
        Node *b = splitPolygon(outerNode, holeNode);
        filterPoints(b, b->next);
    }
}

// link every hole into the outer loop, producing a single-ring polygon without holes
Node *eliminateHoles(const Polygon &points, Node *outerNode) {
    const auto len = points.size();

    std::vector<Node *> queue;
    for (size_t i = 1; i < len; i++) {
        auto list = filterPoints(linkedList(points[i], false));
        if (list) {
            queue.push_back(getLeftmost(list));
        }
    }
    std::sort(queue.begin(), queue.end(), compareX);

    // process holes from left to right
    for (size_t i = 0; i < queue.size(); i++) {
        eliminateHole(queue[i], outerNode);
        outerNode = filterPoints(outerNode, outerNode->next);
    }

    return outerNode;
}

template <typename T>
class optional_number {
public:
    inline optional_number() : valid(false) {
    }
    inline optional_number(T val) : value(val), valid(true) {
    }
    inline optional_number &operator=(T val) {
        value = val;
        valid = true;
        return *this;
    }
    inline bool defined() const {
        return valid;
    }
    inline operator T &() {
        return value;
    }

private:
    T value;
    bool valid;
};

// z-order of a point given coords and size of the data bounding box
uint16_t zOrder(Node::X x, Node::Y y, optional_number<Node::X> minX, optional_number<Node::Y> minY, const int size) {
    // coords are transformed into (0..1000) integer range
    x = 1000 * (x - minX) / size;
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;

    y = 1000 * (y - minY) / size;
    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;

    return x | (y << 1);
}

// Simon Tatham's linked list merge sort algorithm
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
Node *sortLinked(Node *list) {
    // var i, p, q, e, tail, numMerges, pSize, qSize,
    //     inSize = 1;

    Node *p, *q, *e, *tail;
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
// interlink polygon nodes in z-order
void indexCurve(Node *const start, const optional_number<Node::X> minX, const optional_number<Node::Y> minY, const int size) {
    Node *node = start;

    do {
        node->z = node->z ||  zOrder(std::get<0>(node->p), std::get<1>(node->p), minX, minY, size);
        node->prevZ = node->prev;
        node->nextZ = node->next;
        node = node->next;
    } while (node != start);

    node->prevZ->nextZ = nullptr;
    node->prevZ = nullptr;

    sortLinked(node);
}


// check whether a polygon node forms a valid ear with adjacent nodes
bool isEar(Node *const ear, const optional_number<Node::X> minX, const optional_number<Node::Y> minY, const int size) {
    auto &a = ear->prev->p;
    auto &b = ear->p;
    auto &c = ear->next->p;

    const auto ax = std::get<0>(a);
    const auto bx = std::get<0>(b);
    const auto cx = std::get<0>(c);
    const auto ay = std::get<1>(a);
    const auto by = std::get<1>(b);
    const auto cy = std::get<1>(c);

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

    Node *node;
    Node::X px;
    Node::Y py;
    Node::X s, t, k;

    // if we use z-order curve hashing, iterate through the curve
    if (minX.defined()) {

        // triangle bbox; min & max are calculated like this for speed
        auto const minTX = ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx);
        auto const minTY = ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy);
        auto const maxTX = ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx);
        auto const maxTY = ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy);

        // z-order range for the current triangle bbox;
        auto const minZ = zOrder(minTX, minTY, minX, minY, size);
        auto const maxZ = zOrder(maxTX, maxTY, minX, minY, size);

        // first look for points inside the triangle in increasing z-order
        node = ear->nextZ;

        while (node && node->z <= maxZ) {
            auto &p = node->p;
            node = node->nextZ;
            if (p == a || p == c) continue;

            px = std::get<0>(p);
            py = std::get<1>(p);

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
            auto &p = node->p;
            node = node->prevZ;
            if (p == a || p == c) continue;

            px = std::get<0>(p);
            py = std::get<1>(p);

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
            auto &p = node->p;
            node = node->next;

            px = std::get<0>(p);
            py = std::get<1>(p);

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

// check if two segments intersect
bool intersects(const Point &p1, const Point &q1, const Point &p2, const Point &q2) {
    return orient(p1, q1, p2) != orient(p1, q1, q2) &&
           orient(p2, q2, p1) != orient(p2, q2, q1);
}

// go through all polygon nodes and cure small local self-intersections
Node *cureLocalIntersections(Node *start, Triangles &triangles) {
    Node *node = start;
    do {
        Node *a = node->prev;
        Node *b = node->next->next;

        // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2])
        if (intersects(a->p, node->p, node->next->p, b->p) && locallyInside(a, b) && locallyInside(b, a)) {

            triangles.push_back(a->p);
            triangles.push_back(node->p);
            triangles.push_back(b->p);

            // remove two nodes involved
            a->next = b;
            b->prev = a;

            Node *az = node->prevZ;
            Node *bz = node->nextZ ? node->nextZ->nextZ : nullptr;

            if (az) az->nextZ = bz;
            if (bz) bz->prevZ = az;

            node = start = b;
        }
        node = node->next;
    } while (node != start);

    return node;
}

// check if a polygon diagonal intersects any polygon segments
bool intersectsPolygon(Node *const start, const Point &a, const Point &b) {
    Node *node = start;
    do {
        auto &p1 = node->p;
        auto &p2 = node->next->p;

        if (p1 != a && p2 != a && p1 != b && p2 != b && intersects(p1, p2, a, b)) return true;

        node = node->next;
    } while (node != start);

    return false;
}

// check if the middle point of a polygon diagonal is inside the polygon
bool middleInside(Node *const start, const Point &a, const Point &b) {
    Node *node = start;
    bool inside = false;
    auto px = (std::get<0>(a) + std::get<0>(b)) / 2;
    auto py = (std::get<0>(a) + std::get<0>(b)) / 2;
    do {
        auto &p1 = node->p;
        auto &p2 = node->next->p;

        if (((std::get<1>(p1) > py) != (std::get<1>(p2) > py)) &&
            (px < (std::get<0>(p2) - std::get<0>(p1)) * (py - std::get<1>(p1)) /
                          (std::get<1>(p2) - std::get<1>(p1)) +
                      std::get<0>(p1))) {
            inside = !inside;
        }

        node = node->next;
    } while (node != start);

    return inside;
}
// check if a diagonal between two polygon nodes is valid (lies in polygon interior)
bool isValidDiagonal(Node *const a, Node *const b) {
    return !intersectsPolygon(a, a->p, b->p) &&
           locallyInside(a, b) && locallyInside(b, a) &&
           middleInside(a, a->p, b->p);
}

void splitEarcut(Node *start, Triangles &triangles, const optional_number<Node::X> minX, const optional_number<Node::Y> minY, const int size);

// main ear slicing loop which triangulates a polygon (given as a linked list)
void earcutLinked(Node *ear, Triangles &triangles, const optional_number<Node::X> minX, const optional_number<Node::Y> minY, const int size, int pass = 0) {
    if (!ear) return;

    // interlink polygon nodes in z-order
    if (!pass && minX.defined()) indexCurve(ear, minX, minY, size);

    Node *stop = ear;
    Node *prev, *next;

    // iterate through ears, slicing them one by one
    while (ear->prev != ear->next) {
        prev = ear->prev;
        next = ear->next;

        if (isEar(ear, minX, minY, size)) {
            // cut off the triangle
            triangles.push_back(prev->p);
            triangles.push_back(ear->p);
            triangles.push_back(next->p);

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
            if (!pass) earcutLinked(filterPoints(ear), triangles, minX, minY, size, 1);

            // if this didn't work, try curing all small self-intersections locally
            else if (pass == 1) {
                ear = cureLocalIntersections(ear, triangles);
                earcutLinked(ear, triangles, minX, minY, size, 2);

            // as a last resort, try splitting the remaining polygon into two
            } else if (pass == 2) splitEarcut(ear, triangles, minX, minY, size);

            break;
        }
    }
}

// try splitting polygon into two and triangulate them independently
void splitEarcut(Node *start, Triangles &triangles, const optional_number<Node::X> minX, const optional_number<Node::Y> minY, const int size) {
    // look for a valid diagonal that divides the polygon into two
    Node *a = start;
    do {
        Node *b = a->next->next;
        while (b != a->prev) {
            if (isValidDiagonal(a, b)) {
                // split the polygon in two by the diagonal
                auto c = splitPolygon(a, b);

                // filter colinear points around the cuts
                a = filterPoints(a, a->next);
                c = filterPoints(c, c->next);

                // run earcut on each half
                earcutLinked(a, triangles, minX, minY, size);
                earcutLinked(c, triangles, minX, minY, size);
                return;
            }
            b = b->next;
        }
        a = a->next;
    } while (a != start);
}

Triangles earcut(const Polygon &points) {
    auto outerNode = filterPoints(linkedList(points[0], true));
    Triangles triangles;

    if (!outerNode) return triangles;

    optional_number<Node::X> minX, maxX;
    optional_number<Node::Y> minY, maxY;
    int threshold = 80;
    int size = 0;

    for (size_t i = 0; threshold >= 0 && i < points.size(); i++) threshold -= points[i].size();

    // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
    if (threshold < 0) {
        auto node = outerNode->next;
        minX = std::get<0>(node->p);
        maxX = std::get<0>(node->p);
        minY = std::get<1>(node->p);
        maxY = std::get<1>(node->p);

        auto x = minX, y = minY;

        do {
            x = std::get<0>(node->p);
            y = std::get<1>(node->p);
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

    earcutLinked(outerNode, triangles, minX, minY, size);

    return triangles;
}
