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
static bool drawFill = true, drawMesh = true, drawOutline = true;
static bool dirtyViewport = true, dirtyShape = true, dirtyTessellator = true;
static float colorBackground[4] = {1.f, 1.f, 1.f, 1.f};
static float colorMesh[4] = {1.f, 0.f, 0.f, 0.2f}, colorFill[4] = {1.f, 1.f, 0.f, 0.2f};
static float colorOutline[4] = {0.2f, 0.2f, 0.2f, 0.9f}, colorInline[4] = {0.7f, 0.6f, .2f, 0.9f};

static bool mouseDrag = false;

static std::size_t shapeIndex = 0;
static std::size_t tessellator = 0;

struct Camera2D {
    double left = .0, right = .0, bottom = .0, top = .0;
    double translateX = .0, translateY = .0;
    int viewWidth = 0, viewHeight = 0;
    double zoom = -1.;
    double mx = .0, cx = .0, my = .0, cy = .0;
    inline float dpi() { return float(viewHeight) / height; }
    inline double scaling() { return std::pow(1.1, zoom); }
    void setView(int width, int height) {
        viewWidth = width;
        viewHeight = height;
    }
    void limits(double l, double r, double b, double t) {
        left = l;
        right = r;
        bottom = b;
        top = t;
    }
    bool scale(double z) {
        if (z == 0) return false;
        zoom += z;
        return true;
    }
    bool move(double x, double y) {
        const double s = scaling();
        const double dx = x / double(viewWidth) * (right - left) / s;
        const double dy = y / double(viewHeight) * (bottom - top) / s;
        if (dx == 0 && dy == 0) {
            return false;
        }
        translateX += dx;
        translateY += dy;
        return true;
    }
    /* apply current transform to opengl context */
    void apply() {
        glViewport(0, 0, viewWidth, viewHeight);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        const double s = scaling();
        const double w2 = .5 * (right - left);
        const double h2 = .5 * (top - bottom);
        mx = s / w2;
        cx = (-left - w2 + translateX) * mx;
        my = s / h2;
        cy = (-bottom - h2 + translateY) * my;
    }
    void setDefaults() {
        zoom = -1.;
        translateX = .0;
        translateY = .0;
    }
    /* transforms world coordinate to screen range [-1,1] with double precision */
    inline void toScreen(double *x, double *y) {
        *x = *x * mx + cx;
        *y = *y * my + cy;
    }
    /* transforms screen coordinates in range [-1,1] to world coordinates with double precision */
    inline void toWorld(double *x, double *y) {
        *x = (*x - cx) / mx;
        *y = (*y - cy) / my;
    }
    inline void vec2(double x, double y) {
        toScreen(&x, &y);
        glVertex2d(x, y);
    }
};

static Camera2D cam;

class DrawablePolygon {
public:
    static std::unique_ptr<DrawablePolygon> makeDrawable(std::size_t index, mapbox::fixtures::FixtureTester* fixture);
    DrawablePolygon() = default;
    virtual ~DrawablePolygon() = default;
    virtual const char* name() = 0;
    virtual void drawMesh() = 0;
    virtual void drawOutline() = 0;
    virtual void drawFill() = 0;
};

class DrawableTesselator : public DrawablePolygon {
    mapbox::fixtures::FixtureTester::TesselatorResult shape;
    mapbox::fixtures::DoublePolygon const& polygon;
public:
    explicit DrawableTesselator(mapbox::fixtures::FixtureTester::TesselatorResult tessellation,
                                mapbox::fixtures::DoublePolygon const& poly) : shape(tessellation), polygon(poly) { }
    void drawMesh() override {
        const auto &v = shape.vertices;
        const auto &x = shape.indices;
        glBegin(GL_LINES);
        glColor4fv(colorMesh);
        for (size_t i = 0; i < x.size(); i += 3) {
            cam.vec2(v[x[i]][0], v[x[i]][1]);
            cam.vec2(v[x[i + 1]][0], v[x[i + 1]][1]);
            cam.vec2(v[x[i + 1]][0], v[x[i + 1]][1]);
            cam.vec2(v[x[i + 2]][0], v[x[i + 2]][1]);
            cam.vec2(v[x[i + 2]][0], v[x[i + 2]][1]);
            cam.vec2(v[x[i]][0], v[x[i]][1]);
        }
        glEnd();
    }
    void drawOutline() override {
        glBegin(GL_LINES);
        for (std::size_t i = 0; i < polygon.size(); i++) {
            auto& ring = polygon[i];
            glColor4fv(i == 0 ? colorOutline : colorInline);
            for (std::size_t j = 0; j < ring.size(); j++) {
                auto& p0 = ring[j];
                auto& p1 = ring[(j+1) % ring.size()];
                cam.vec2(std::get<0>(p0), std::get<1>(p0));
                cam.vec2(std::get<0>(p1), std::get<1>(p1));
            }
        }
        glEnd();
    }
    void drawFill() override {
        const auto &v = shape.vertices;
        const auto &x = shape.indices;
        glBegin(GL_TRIANGLES);
        glColor4fv(colorFill);
        for (const auto pt : x) {
            cam.vec2(v[pt][0], v[pt][1]);
        }
        glEnd();
    }
};

class DrawableEarcut : public DrawableTesselator {
public:
    explicit DrawableEarcut(mapbox::fixtures::FixtureTester* fixture)
            : DrawableTesselator(fixture->earcut(), fixture->polygon()) { }
    const char *name() override { return "earcut"; };
};

class DrawableLibtess : public DrawableTesselator {
public:
    explicit DrawableLibtess(mapbox::fixtures::FixtureTester* fixture)
            : DrawableTesselator(fixture->libtess(), fixture->polygon()) { }
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
        inline double intersection(double scanline) const {
            // horizontal scan-line intersection from the left
            // double precision for the calculation is required to avoid artifacts
            return scanline * scale + offset;
        }
    };
    mapbox::fixtures::FixtureTester* shape;
    std::vector<Edge*> activeList; /* contains current sorted intersections */
    std::vector<std::vector<Edge>> edgeTables; /* contains all edges sorted by yMin */
public:
    explicit DrawableScanLineFill(mapbox::fixtures::FixtureTester* fixture) : shape(fixture) {
        auto& polygon = shape->polygon();
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

        const double halfWidth = lineWidth * 0.5;
        top -= lineWidth;
        bottom += lineWidth;

        double y0 = top;
        while(y0 < bottom && edgeList) {
            double y1 = y0 + lineWidth;
            if (y1 == y0) { y1 = std::nextafter(y1, bottom); }
            const double y = y0 + halfWidth;

            activeList.clear();
            Edge* prevEdge = nullptr;
            for (auto edge = edgeList; edge != nullptr; edge = edge->next) {
                const auto min = edge->yMin, max = edge->yMax;
                if (min <= y && y < max) {
                    activeList.push_back(edge);
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
            std::sort(activeList.begin(), activeList.end(), [&](Edge *a, Edge *b) {
                return a->intersection(y) < b->intersection(y);
            });

            double x1y0 = 0, x1y1 = 0;
            for (std::size_t i = 0; i < activeList.size(); i++) {
                Edge *edge = activeList[i];

                // use slope to make MSAA possible
                const double x2y0 = edge->intersection(std::max<double>(edge->yMin, y0));
                const double x2y1 = edge->intersection(std::min<double>(edge->yMax, y1));
                if ((i % 2) != 0) {
                    cam.vec2(x1y0, y0);
                    cam.vec2(x1y1, y1);
                    cam.vec2(x2y1, y1);
                    cam.vec2(x2y0, y0);
                }
                x1y0 = x2y0;
                x1y1 = x2y1;
            }

            y0 = y1;
        }
    }
    void drawFill() override {
        for (std::size_t i = 0; i < edgeTables.size(); i++) {
            auto& edgeTable = edgeTables[i];
            glBegin(GL_QUADS);
            glColor4fv(i == 0 ? colorFill : colorBackground);
            double x0 = 1;
            double y0 = 1;
            cam.toWorld(&x0, &y0);
            double x1 = -1;
            double y1 = -1;
            cam.toWorld(&x1, &y1);
            scanLineFill(edgeTable, y0, y1, (y1 - y0) / cam.viewHeight);
            glEnd();
        }
    }
    void drawOutline() override {
        auto& polygon = shape->polygon();
        glBegin(GL_LINES);
        for (std::size_t i = 0; i < polygon.size(); i++) {
            auto& ring = polygon[i];
            glColor4fv(i == 0 ? colorOutline : colorInline);
            for (std::size_t j = 0; j < ring.size(); j++) {
                auto& p0 = ring[j];
                auto& p1 = ring[(j+1) % ring.size()];
                cam.vec2(std::get<0>(p0), std::get<1>(p0));
                cam.vec2(std::get<0>(p1), std::get<1>(p1));
            }
        }
        glEnd();
    }
    void drawMesh() override { }
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
    glfwWindowHint(GLFW_SAMPLES, 4);
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
        } else if (key == GLFW_KEY_O) {
            drawOutline = !drawOutline;
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
            dirtyViewport |= cam.scale(1.);
        } else if (key == GLFW_KEY_KP_SUBTRACT) {
            dirtyViewport |= cam.scale(-1.);
        } else if (key == GLFW_KEY_R) {
            dirtyTessellator = dirtyViewport = dirtyShape = true;
        } else if (key == GLFW_KEY_W) {
            dirtyViewport |= cam.move(.0, cam.viewHeight / 50.);
        } else if (key == GLFW_KEY_A) {
            dirtyViewport |= cam.move(cam.viewWidth / 50., .0);
        } else if (key == GLFW_KEY_S) {
            dirtyViewport |= cam.move(.0, -cam.viewHeight / 50.);
        } else if (key == GLFW_KEY_D) {
            dirtyViewport |= cam.move(-cam.viewWidth / 50., .0);
        }
    });

    glfwSetScrollCallback(window, [](GLFWwindow* /* window */, double /* xoffset */, double yoffset) {
        dirtyViewport |= cam.scale(yoffset);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow* /* window */, int button, int action, int /* mods */){
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            mouseDrag = action != GLFW_RELEASE;
        }
    });

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow * /*win*/, int w, int h) {
        cam.setView(w, h);
    });

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    cam.setView(fbWidth, fbHeight);

    glfwMakeContextCurrent(window);

    glfwSwapInterval(1);

    glClearColor(colorBackground[0], colorBackground[1], colorBackground[2], colorBackground[3]);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    while (!glfwWindowShouldClose(window)) {
        static double mouseX = 0, mouseY = 0;
        double mousePrevX = mouseX, mousePrevY = mouseY;
        glfwGetCursorPos(window, &mouseX, &mouseY);
        double mouseDeltaX = mouseX - mousePrevX, mouseDeltaY = mouseY - mousePrevY;
        if (mouseDrag) {
            dirtyViewport |= cam.move(mouseDeltaX, mouseDeltaY);
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
                    minX = std::min<double>(minX, std::get<0>(pt));
                    minY = std::min<double>(minY, std::get<1>(pt));
                    maxX = std::max<double>(maxX, std::get<0>(pt));
                    maxY = std::max<double>(maxY, std::get<1>(pt));
                }
            }
            const auto dimX = minX < maxX ? maxX - minX : 0;
            const auto dimY = minY < maxY ? maxY - minY : 0;

            auto midX = minX + dimX / 2;
            auto midY = minY + dimY / 2;
            auto ext = std::max<double>(dimX, dimY) / 2;

            cam.setDefaults();
            cam.limits(midX - ext, midX + ext, midY + ext, midY - ext);
        }

        if (dirtyViewport || dirtyShape || dirtyTessellator) {
            glClear(GL_COLOR_BUFFER_BIT);

            cam.apply();
            glLineWidth(cam.dpi() * std::sqrt(2.f));

            auto& drawable = tessellators[tessellator];

            if (!drawable) {
                drawable = DrawablePolygon::makeDrawable(tessellator, getFixture(shapeIndex));
            }

            if (dirtyTessellator || dirtyShape) {
                glfwSetWindowTitle(window, (std::string(drawable->name()) + ": "
                                            + getFixture(shapeIndex)->name).c_str());
            }

            if (!drawMesh && !drawFill && !drawOutline) {
                drawMesh = drawFill = drawOutline = true;
            }

            if (drawFill) {
                drawable->drawFill();
            }
            if (drawMesh) {
                drawable->drawMesh();
            }
            if (drawOutline) {
                drawable->drawOutline();
            }

            glFlush(); /* required for Mesa 3D driver */
            glfwSwapBuffers(window);
        }

        dirtyTessellator = dirtyShape = dirtyViewport = false;
        glfwWaitEvents();
    }

    glfwTerminate();
    return 0;
}
