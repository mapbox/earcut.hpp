#include "comparison/earcut.hpp"
#include "comparison/libtess2.hpp"

#include "fixtures/geometries.hpp"

#if _MSC_VER >= 1900
#pragma comment(lib, "legacy_stdio_definitions.lib")
#endif

#include <GLFW/glfw3.h>

#include <cstdlib>
#include <cmath>
#include <vector>

static GLFWwindow *window = nullptr;
static const int width = 1024;
static const int height = 1024;
static int fbWidth = 0;
static int fbHeight = 0;
static bool drawFill = true;
static bool drawMesh = true;
static bool dirty = true;

static std::size_t shapeIndex = 0;

static std::size_t tesselator = 0;
const static int totalTesselators = 3;
const static std::array<std::string, totalTesselators> tesselatorNames = {{ "earcut", "libtess2", "scanline-fill" }};

template <class Ring>
void scanLineFill(Ring const& ring, double top, double bottom, double lineWidth) {
    assert(top < bottom);
    assert(lineWidth > 0);

    struct Edge {
        float yMin;
        float yMax;
        float scale;
        float offset;
        Edge* next = nullptr;
        Edge(double x1, double y1, double x2, double y2)
            : yMin((float)std::min<double>(y1, y2)),
              yMax((float)std::max<double>(y1, y2))
        {
            const double dx = x1 - x2;
            const double dy = y1 - y2;
            scale = (float)(dx / dy);
            offset = (float)((y1 * x2 - x1 * y2) / dy);
        }
        inline float intersection(float scanline) {
            // horizontal scanline intersection from the left
            return scanline * scale + offset;
        }
    };

    std::vector<float> intersections; /* contains current sorted intersections */
    std::vector<Edge> edgeTable; /* contains all edges sorted by yMin */
    edgeTable.reserve(ring.size());
    for (std::size_t i = 1; i <= ring.size(); i++) {
        const auto& p0 = ring[i - 1], p1 = i == ring.size() ? ring[0] : ring[i];
        const double x1 = std::get<0>(p0), y1 = std::get<1>(p0), x2 = std::get<0>(p1), y2 = std::get<1>(p1);
        if (y1 != y2) { edgeTable.emplace_back(x1, y1, x2, y2); }
    }

    // sort edges
    std::sort(edgeTable.begin(), edgeTable.end(), [&](Edge const& a, Edge const& b) { return a.yMin < b.yMin; });
    // create intrusive sorted edge list
    for (std::size_t i = 1; i < edgeTable.size(); i++) {
        edgeTable[i-1].next = &edgeTable[i];
    }
    Edge* edgeList = edgeTable.empty() ? nullptr : &edgeTable[0];

    double y = top;
    while(y < bottom && edgeList) {
        intersections.clear();
        Edge* prevEdge = nullptr;
        for (auto edge = edgeList; edge != nullptr; edge = edge->next) {
            const auto min = edge->yMin, max = edge->yMax;
            if (min <= y && y <= max) {
                intersections.push_back(edge->intersection(static_cast<float>(y)));
            }
            if (min > y)  {
                break;
            } else if (y > max) {
                // unlink edge
                Edge** next = prevEdge ? &prevEdge->next : &edgeList;
                *next = edge->next;
            } else {
                prevEdge = edge;
            }
        }
        std::sort(intersections.begin(), intersections.end());

        auto x1 = .0f;
        for (std::size_t i = 0; i < intersections.size(); i++) {
            auto x2 = intersections[i];
            if ((i % 2) != 0) {
                glVertex2f(static_cast<GLfloat>(x1), static_cast<GLfloat>(y));
                glVertex2f(static_cast<GLfloat>(x2), static_cast<GLfloat>(y));
            }
            x1 = x2;
        }

        y += lineWidth;
    }
}

void drawPolygon(mapbox::fixtures::FixtureTester* fixture) {
    glfwSetWindowTitle(window, (tesselatorNames[tesselator] + ": " + fixture->name).c_str());
    const auto& polygon = fixture->polygon();

    auto minX = std::numeric_limits<double>::max();
    auto maxX = std::numeric_limits<double>::min();
    auto minY = std::numeric_limits<double>::max();
    auto maxY = std::numeric_limits<double>::min();
    if (!polygon.empty()) {
        for (const auto &pt : polygon[0]) {
            if (std::get<0>(pt) < minX) minX = std::get<0>(pt);
            if (std::get<1>(pt) < minY) minY = std::get<1>(pt);
            if (std::get<0>(pt) > maxX) maxX = std::get<0>(pt);
            if (std::get<1>(pt) > maxY) maxY = std::get<1>(pt);
        }
    }
    const auto dimX = minX < maxX ? maxX - minX : 0;
    const auto dimY = minY < maxY ? maxY - minY : 0;

    auto midX = minX + dimX / 2;
    auto midY = minY + dimY / 2;
    auto ext = 1.10 * std::max<double>(dimX, dimY) / 2;

    glViewport(0, 0, fbWidth, fbHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    auto left = midX - ext;
    auto right = midX + ext;
    auto bottom = midY + ext;
    auto top = midY - ext;
    glOrtho(left, right, bottom, top, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT);

    if (tesselator == 0 || tesselator == 1) {
        auto shape = tesselator == 0 ? fixture->earcut() : fixture->libtess();
        const auto &v = shape.vertices;
        const auto &x = shape.indices;

        // Draw triangle fill
        if (drawFill) {
            glBegin(GL_TRIANGLES);
            glColor3f(0.3f, 0.3f, 0.3f);
            for (const auto pt : x) {
                glVertex2f(static_cast<GLfloat>(v[pt][0]), static_cast<GLfloat>(v[pt][1]));
            }
            glEnd();
        }

        // Draw triangle mesh
        if (drawMesh) {
            glLineWidth(float(fbHeight) / height);
            glBegin(GL_LINES);
            glColor3f(1, 0, 0);
            for (size_t i = 0; i < x.size(); i += 3) {
                glVertex2f(static_cast<GLfloat>(v[x[i]][0]), static_cast<GLfloat>(v[x[i]][1]));
                glVertex2f(static_cast<GLfloat>(v[x[i + 1]][0]), static_cast<GLfloat>(v[x[i + 1]][1]));
                glVertex2f(static_cast<GLfloat>(v[x[i + 1]][0]), static_cast<GLfloat>(v[x[i + 1]][1]));
                glVertex2f(static_cast<GLfloat>(v[x[i + 2]][0]), static_cast<GLfloat>(v[x[i + 2]][1]));
                glVertex2f(static_cast<GLfloat>(v[x[i + 2]][0]), static_cast<GLfloat>(v[x[i + 2]][1]));
                glVertex2f(static_cast<GLfloat>(v[x[i]][0]), static_cast<GLfloat>(v[x[i]][1]));
            }
            glEnd();
        }
    } else {
        glLineWidth(float(fbHeight) / height);
        // Draw line fill
        if (drawFill) {
            for (std::size_t i = 0; i < polygon.size(); i++) {
                auto& ring = polygon[i];
                glBegin(GL_LINES);
                if (i == 0) {
                    glColor3f(0.3f, 0.3f, 0.3f);
                } else {
                    glColor3f(1, 1, 1);
                }
                scanLineFill(ring, top, bottom, (bottom - top) / fbHeight);
                glEnd();
            }
        }

        // Draw outline
        if (drawMesh) {
            for (std::size_t i = 0; i < polygon.size(); i++) {
                auto& ring = polygon[i];
                glBegin(GL_LINES);
                if (i == 0) {
                    glColor3f(1, 0, 0);
                } else {
                    glColor3f(1, 1, 0);
                }
                for (std::size_t j = 0; j < ring.size(); j++) {
                    auto& p0 = ring[j];
                    auto& p1 = ring[(j+1) % ring.size()];
                    glVertex2f(static_cast<GLfloat>(std::get<0>(p0)), static_cast<GLfloat>(std::get<1>(p0)));
                    glVertex2f(static_cast<GLfloat>(std::get<0>(p1)), static_cast<GLfloat>(std::get<1>(p1)));
                }
                glEnd();
            }

        }
    }
}


int main() {
    if (!glfwInit()) {
        return 1;
    }

    glfwWindowHint(GLFW_RESIZABLE, 0);
    window = glfwCreateWindow(width, height, "Tessellation", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    glfwSetKeyCallback(window,
                       [](GLFWwindow *win, int key, int /*scancode*/, int action, int /*mods*/) {
        if (action != GLFW_PRESS && action != GLFW_REPEAT) {
            return;
        }

        if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
            glfwSetWindowShouldClose(win, 1);
        } else if (key == GLFW_KEY_F) {
            drawFill = !drawFill;
            dirty = true;
        } else if (key == GLFW_KEY_M) {
            drawMesh = !drawMesh;
            dirty = true;
        } else if (key == GLFW_KEY_RIGHT) {
            if (shapeIndex + 1 < mapbox::fixtures::FixtureTester::collection().size()) {
                shapeIndex++;
                dirty = true;
            }
        } else if (key == GLFW_KEY_LEFT) {
            if (shapeIndex >= 1) {
                shapeIndex--;
                dirty = true;
            }
        } else if (key == GLFW_KEY_T) {
            tesselator = (tesselator + 1) % totalTesselators;
            dirty = true;
        }
    });

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow * /*win*/, int w, int h) {
        fbWidth = w;
        fbHeight = h;
    });

    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    glfwMakeContextCurrent(window);

    glClearColor(1, 1, 1, 1);


    while (!glfwWindowShouldClose(window)) {

        if (dirty) {
            auto& fixtures = mapbox::fixtures::FixtureTester::collection();
            auto& fixture = fixtures[shapeIndex % fixtures.size()];
            drawPolygon(fixture);

            glfwSwapBuffers(window);
            dirty = false;
        }

        glfwWaitEvents();
    }

    glfwTerminate();
    return 0;
}
