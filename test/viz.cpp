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
#include <memory>

static GLFWwindow *window = nullptr;
static const int width = 1024;
static const int height = 1024;
static int fbWidth = 0;
static int fbHeight = 0;
static bool drawFill = true;
static bool drawMesh = true;
static bool dirtyViewport = true, dirtyShape = true, dirtyTessellator = true;

static double left = 0, right = 0, bottom = 0, top = 0;
static bool mouseDrag = false;

static std::size_t shapeIndex = 0;

static std::size_t tessellator = 0;

class DrawablePolygon {
public:
    static std::unique_ptr<DrawablePolygon> makeDrawable(std::size_t index, mapbox::fixtures::FixtureTester* fixture);
    DrawablePolygon() = default;
    virtual ~DrawablePolygon() = default;
    virtual const char* name() = 0;
    virtual void drawMesh() = 0;
    virtual void drawFill() = 0;
};

class DrawableTesselator : public DrawablePolygon {
    mapbox::fixtures::FixtureTester::TesselatorResult shape;
public:
    explicit DrawableTesselator(mapbox::fixtures::FixtureTester::TesselatorResult tessellation) : shape(tessellation) { }
    void drawMesh() override {
        const auto &v = shape.vertices;
        const auto &x = shape.indices;
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
    };
    void drawFill() override {
        const auto &v = shape.vertices;
        const auto &x = shape.indices;
        glBegin(GL_TRIANGLES);
        glColor3f(0.3f, 0.3f, 0.3f);
        for (const auto pt : x) {
            glVertex2f(static_cast<GLfloat>(v[pt][0]), static_cast<GLfloat>(v[pt][1]));
        }
        glEnd();
    };
};

class DrawableEarcut : public DrawableTesselator {
public:
    explicit DrawableEarcut(mapbox::fixtures::FixtureTester* fixture) : DrawableTesselator(fixture->earcut()) { }
    const char *name() override { return "earcut"; };
};

class DrawableLibtess : public DrawableTesselator {
public:
    explicit DrawableLibtess(mapbox::fixtures::FixtureTester* fixture) : DrawableTesselator(fixture->libtess()) { }
    const char *name() override { return "libtess2"; };
};

class DrawableScanLineFill : public DrawablePolygon {
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
    mapbox::fixtures::FixtureTester* shape;
    std::vector<float> intersections; /* contains current sorted intersections */
    std::vector<std::vector<Edge>> edgeTables; /* contains all edges sorted by yMin */
public:
    explicit DrawableScanLineFill(mapbox::fixtures::FixtureTester* fixture) : shape(fixture) {
        auto& polygon = shape->polygon();
        glLineWidth(float(fbHeight) / height);
        edgeTables.reserve(polygon.size());
        for (const auto &ring : polygon) {
            std::vector<Edge> edgeTable;
            edgeTable.reserve(ring.size());
            for (std::size_t i = 1; i <= ring.size(); i++) {
                const auto &p0 = ring[i - 1], p1 = i == ring.size() ? ring[0] : ring[i];
                const double x1 = std::get<0>(p0), y1 = std::get<1>(p0), x2 = std::get<0>(p1), y2 = std::get<1>(p1);
                if (y1 != y2) { edgeTable.emplace_back(x1, y1, x2, y2); }
            }
            std::sort(edgeTable.begin(), edgeTable.end(), [&](Edge const &a, Edge const &b) {
                return a.yMin < b.yMin;
            });
            edgeTables.push_back(std::move(edgeTable));
        }
    }
    void scanLineFill(std::vector<Edge>& edgeTable, double top, double bottom, double lineWidth) {
        assert(top < bottom);
        assert(lineWidth > 0);

        // create intrusive sorted edge list
        for (std::size_t i = 1; i < edgeTable.size(); i++) {
            edgeTable[i-1].next = &edgeTable[i];
        }
        Edge* edgeList = edgeTable.empty() ? nullptr : &edgeTable[0];

        // halfWidth used to center lines on pixels
        const double halfWidth = lineWidth * 0.5;
        top -= halfWidth;
        bottom += halfWidth;

        double y = top;
        while(y < bottom && edgeList) {
            intersections.clear();
            Edge* prevEdge = nullptr;
            for (auto edge = edgeList; edge != nullptr; edge = edge->next) {
                const auto min = edge->yMin, max = edge->yMax;
                if (min <= y && y < max) {
                    intersections.push_back(edge->intersection(static_cast<float>(y)));
                }
                if (min > y)  {
                    break;
                } else if (y >= max) {
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
                    glVertex2f(static_cast<GLfloat>(x1 + halfWidth), static_cast<GLfloat>(y));
                    glVertex2f(static_cast<GLfloat>(x2 + halfWidth), static_cast<GLfloat>(y));
                }
                x1 = x2;
            }

            y += lineWidth;
        }
    }
    void drawFill() override {
        for (std::size_t i = 0; i < edgeTables.size(); i++) {
            auto& edgeTable = edgeTables[i];
            glBegin(GL_LINES);
            if (i == 0) {
                glColor3f(0.3f, 0.3f, 0.3f);
            } else {
                glColor3f(1, 1, 1);
            }
            scanLineFill(edgeTable, top, bottom, (bottom - top) / fbHeight);
            glEnd();
        }
    }
    void drawMesh() override {
        auto& polygon = shape->polygon();
        glLineWidth(float(fbHeight) / height);
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
    const char *name() override { return "scanline-fill"; }
};

std::unique_ptr<DrawablePolygon> DrawablePolygon::makeDrawable(std::size_t index, mapbox::fixtures::FixtureTester* fixture) {
    if (index == 0) {
        return std::unique_ptr<DrawablePolygon>(new DrawableEarcut(fixture));
    } else if (index == 1) {
        return std::unique_ptr<DrawablePolygon>(new DrawableLibtess(fixture));
    } else {
        return std::unique_ptr<DrawablePolygon>(new DrawableScanLineFill(fixture));
    }
}

static std::array<std::unique_ptr<DrawablePolygon>, 3> tessellators;

void scaleViewport(double zoom) {
    zoom = 1. - std::pow(1.1, zoom);
    double dx = zoom * (right - left);
    double dy = zoom * (bottom - top);
    left -= dx;
    right += dx;
    top -= dy;
    bottom += dy;
    dirtyViewport = true;
}

void translateViewport(double x, double y) {
    double dx = x / fbWidth * (right - left);
    double dy = y / fbHeight * (bottom - top);
    left += dx;
    right += dx;
    top += dy;
    bottom += dy;
    dirtyViewport = true;
}

mapbox::fixtures::FixtureTester *getFixture(std::size_t i) {
    auto& fixtures = mapbox::fixtures::FixtureTester::collection();
    if (fixtures.empty()) {
        assert(false);
        exit(1);
    }
    return fixtures[i % fixtures.size()];
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
            dirtyViewport = true;
        } else if (key == GLFW_KEY_M) {
            drawMesh = !drawMesh;
            dirtyViewport = true;
        } else if (key == GLFW_KEY_RIGHT) {
            if (shapeIndex + 1 < mapbox::fixtures::FixtureTester::collection().size()) {
                shapeIndex++;
                dirtyShape = true;
            }
        } else if (key == GLFW_KEY_LEFT) {
            if (shapeIndex >= 1) {
                shapeIndex--;
                dirtyShape = true;
            }
        } else if (key == GLFW_KEY_T || key == GLFW_KEY_UP) {
            tessellator = (tessellator + 1) % tessellators.size();
            dirtyTessellator = true;
        } else if (key == GLFW_KEY_DOWN) {
            tessellator = (tessellator + tessellators.size() - 1) % tessellators.size();
            dirtyTessellator = true;
        } else if (key == GLFW_KEY_KP_ADD) {
            scaleViewport(1.);
        } else if (key == GLFW_KEY_KP_SUBTRACT) {
            scaleViewport(-1.);
        } else if (key == GLFW_KEY_R) {
            dirtyViewport = dirtyShape = true;
        } else if (key == GLFW_KEY_W) {
            translateViewport(.0, -fbWidth / 50.);
        } else if (key == GLFW_KEY_A) {
            translateViewport(-fbWidth / 50., .0);
        } else if (key == GLFW_KEY_S) {
            translateViewport(.0, fbWidth / 50.);
        } else if (key == GLFW_KEY_D) {
            translateViewport(fbWidth / 50., .0);
        }
    });

    glfwSetScrollCallback(window, [](GLFWwindow* /* window */, double /* xoffset */, double yoffset) {
        scaleViewport(yoffset);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow* /* window */, int button, int action, int /* mods */){
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if(!mouseDrag && action == GLFW_PRESS) {
                mouseDrag = true;
            } else if(mouseDrag && action == GLFW_RELEASE) {
                mouseDrag = false;
            }
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
        static double mouseX = 0, mouseY = 0;
        double mousePrevX = mouseX, mousePrevY = mouseY;
        glfwGetCursorPos(window, &mouseX, &mouseY);
        double mouseDeltaX = mouseX - mousePrevX, mouseDeltaY = mouseY - mousePrevY;
        if (mouseDrag) {
            translateViewport(-mouseDeltaX, -mouseDeltaY);
        }

        if (dirtyShape) {
            for (auto &tessellator : tessellators) {
                tessellator.reset(nullptr);
            }

            const auto& polygon = getFixture(shapeIndex)->polygon();
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

            left = midX - ext;
            right = midX + ext;
            bottom = midY + ext;
            top = midY - ext;
        }

        if (dirtyViewport || dirtyShape || dirtyTessellator) {
            glViewport(0, 0, fbWidth, fbHeight);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            glOrtho(left, right, bottom, top, 0, 1);
            glMatrixMode(GL_MODELVIEW);

            glClear(GL_COLOR_BUFFER_BIT);

            auto& drawable = tessellators[tessellator];

            if (!drawable) {
                drawable = DrawablePolygon::makeDrawable(tessellator, getFixture(shapeIndex));
            }

            if (dirtyTessellator || dirtyShape) {
                glfwSetWindowTitle(window, (std::string(drawable->name()) + ": "
                                            + getFixture(shapeIndex)->name).c_str());
            }

            if (!drawMesh && !drawFill) {
                drawMesh = drawFill = true;
            }
            if (drawFill) {
                drawable->drawFill();
            }
            if (drawMesh) {
                drawable->drawMesh();
            }

            glfwSwapBuffers(window);
        }

        dirtyTessellator = dirtyShape = dirtyViewport = false;
        glfwWaitEvents();
    }

    glfwTerminate();
    return 0;
}
