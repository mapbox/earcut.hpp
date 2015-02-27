#include "comparison/earcut.hpp"
#include "comparison/libtess2.hpp"

#include "fixtures/geometries.hpp"

#include <GLFW/glfw3.h>

#include <cstdlib>
#include <cmath>


static GLFWwindow *window = nullptr;
static const int width = 1024;
static const int height = 1024;
static int fbWidth = 0;
static int fbHeight = 0;
static bool drawFill = true;
static bool drawMesh = true;
static bool dirty = true;

static int shapeIndex = 0;
const static int totalShapes = 12;

static int tesselator = 0;
const static int totalTesselators = 2;
const static std::array<std::string, totalTesselators> tesselatorNames = {{ "earcut", "libtess2" }};

struct Shape {
    Shape(std::vector<std::array<int, 2>> vertices_, std::vector<uint32_t> indices_)
        : vertices(std::move(vertices_)), indices(std::move(indices_)) {
        auto minX = std::numeric_limits<int>::max();
        auto maxX = std::numeric_limits<int>::min();
        auto minY = std::numeric_limits<int>::max();
        auto maxY = std::numeric_limits<int>::min();
        for (const auto &pt : vertices) {
            if (pt[0] < minX) minX = pt[0];
            if (pt[1] < minY) minY = pt[1];
            if (pt[0] > maxX) maxX = pt[0];
            if (pt[1] > maxY) maxY = pt[1];
        }

        const auto dimX = maxX - minX;
        const auto dimY = maxY - minY;
        midX = minX + dimX / 2;
        midY = minY + dimY / 2;
        ext = 1.10 * std::max(dimX, dimY) / 2;
    }

    const std::vector<std::array<int, 2>> vertices;
    const std::vector<uint32_t> indices;
    int midX, midY, ext;
};

template <typename Polygon>
auto buildPolygon(const Polygon &polygon) -> const Shape {
    if (tesselator == 0) {
        EarcutTesselator<int, Polygon> tess(polygon);
        tess.run();
        return Shape(std::move(tess.vertices()), std::move(tess.indices()));
    } else if (tesselator == 1) {
        Libtess2Tesselator<int, Polygon> tess(polygon);
        tess.run();
        return Shape(std::move(tess.vertices()), std::move(tess.indices()));
    }
    assert(false);
    return Shape(std::vector<std::array<int, 2>>(), std::vector<uint32_t>());
}

template <typename Polygon>
void drawPolygon(const char *name, const Polygon &polygon) {
    glfwSetWindowTitle(window, (tesselatorNames[tesselator] + ": " + name).c_str());

    const auto shape = buildPolygon(polygon);

    glViewport(0, 0, fbWidth, fbHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(shape.midX - shape.ext, shape.midX + shape.ext, shape.midY + shape.ext, shape.midY - shape.ext, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    glClear(GL_COLOR_BUFFER_BIT);

    const auto &v = shape.vertices;
    const auto &x = shape.indices;

    // Draw triangle fill
    if (drawFill) {
        glBegin(GL_TRIANGLES);
        glColor3f(0.3f, 0.3f, 0.3f);
        for (const auto pt : x) {
            glVertex2f(v[pt][0], v[pt][1]);
        }
        glEnd();
    }

    // Draw triangle mesh
    if (drawMesh) {
        glLineWidth(float(fbWidth) / width);
        glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        for (size_t i = 0; i < x.size(); i += 3) {
            glVertex2f(v[x[i]][0], v[x[i]][1]);
            glVertex2f(v[x[i + 1]][0], v[x[i + 1]][1]);
            glVertex2f(v[x[i + 1]][0], v[x[i + 1]][1]);
            glVertex2f(v[x[i + 2]][0], v[x[i + 2]][1]);
            glVertex2f(v[x[i + 2]][0], v[x[i + 2]][1]);
            glVertex2f(v[x[i]][0], v[x[i]][1]);
        }
        glEnd();
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
        if (action != GLFW_PRESS) {
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
            if (shapeIndex + 1 < totalShapes) {
                shapeIndex++;
                dirty = true;
            }
        } else if (key == GLFW_KEY_LEFT) {
            if (shapeIndex - 1 >= 0) {
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
            switch (shapeIndex) {
                case 0: drawPolygon("bad_hole", mapbox::fixtures::bad_hole); break;
                case 1: drawPolygon("building", mapbox::fixtures::building); break;
                case 2: drawPolygon("degenerate", mapbox::fixtures::degenerate); break;
                case 3: drawPolygon("dude", mapbox::fixtures::dude); break;
                case 4: drawPolygon("empty_square", mapbox::fixtures::empty_square); break;
                case 5: drawPolygon("water_huge", mapbox::fixtures::water_huge); break;
                case 6: drawPolygon("water_huge2", mapbox::fixtures::water_huge2); break;
                case 7: drawPolygon("water", mapbox::fixtures::water); break;
                case 8: drawPolygon("water2", mapbox::fixtures::water2); break;
                case 9: drawPolygon("water3", mapbox::fixtures::water3); break;
                case 10: drawPolygon("water3b", mapbox::fixtures::water3b); break;
                case 11: drawPolygon("water4", mapbox::fixtures::water4); break;
                default: assert(false); break;
            }

            glfwSwapBuffers(window);
            dirty = false;
        }

        glfwWaitEvents();
    }

    glfwTerminate();
    return 0;
}
