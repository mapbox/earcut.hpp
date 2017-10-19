#include "comparison/earcut.hpp"
#include "comparison/libtess2.hpp"

#include "fixtures/geometries.hpp"

#if _MSC_VER >= 1900
#pragma comment(lib, "legacy_stdio_definitions.lib")
#endif

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

static std::size_t shapeIndex = 0;

static std::size_t tesselator = 0;
const static int totalTesselators = 2;
const static std::array<std::string, totalTesselators> tesselatorNames = {{ "earcut", "libtess2" }};

template <typename T>
struct Shape {
    Shape(std::vector<std::array<T, 2>> vertices_, std::vector<uint32_t> indices_)
        : vertices(std::move(vertices_)), indices(std::move(indices_)) {
        auto minX = std::numeric_limits<T>::max();
        auto maxX = std::numeric_limits<T>::min();
        auto minY = std::numeric_limits<T>::max();
        auto maxY = std::numeric_limits<T>::min();
        for (const auto &i : indices) {
            auto& pt = vertices[i];
            if (pt[0] < minX) minX = pt[0];
            if (pt[1] < minY) minY = pt[1];
            if (pt[0] > maxX) maxX = pt[0];
            if (pt[1] > maxY) maxY = pt[1];
        }

        const auto dimX = minX < maxX ? maxX - minX : 0;
        const auto dimY = minY < maxY ? maxY - minY : 0;

        midX = static_cast<double>(minX + dimX / 2);
        midY = static_cast<double>(minY + dimY / 2);
        ext = static_cast<double>(1.10 * std::max<double>(dimX, dimY) / 2);
    }

    Shape<T> & operator=(const Shape<T>&) = delete;

    const std::vector<std::array<T, 2>> vertices;
    const std::vector<uint32_t> indices;
    double midX, midY, ext;
};

template <typename Coord, typename Polygon>
Shape<Coord> buildPolygon(const Polygon &polygon) {
    if (tesselator == 0) {
        EarcutTesselator<Coord, Polygon> tess(polygon);
        tess.run();
        return Shape<Coord>(std::move(tess.vertices()), std::move(tess.indices()));
    } else if (tesselator == 1) {
        Libtess2Tesselator<Coord, Polygon> tess(polygon);
        tess.run();
        return Shape<Coord>(std::move(tess.vertices()), std::move(tess.indices()));
    }
    assert(false);
    return Shape<Coord>(std::vector<std::array<Coord, 2>>(), std::vector<uint32_t>());
}

template <typename Polygon>
void drawPolygon(const char *name, const Polygon &polygon) {
    glfwSetWindowTitle(window, (tesselatorNames[tesselator] + ": " + name).c_str());

    using Coord = typename Polygon::value_type::value_type::first_type;
    const auto shape = buildPolygon<Coord>(polygon);

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
            glVertex2f(static_cast<GLfloat>(v[pt][0]), static_cast<GLfloat>(v[pt][1]));
        }
        glEnd();
    }

    // Draw triangle mesh
    if (drawMesh) {
        glLineWidth(float(fbWidth) / width);
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
            drawPolygon(fixture->name.c_str(), fixture->polygon());

            glfwSwapBuffers(window);
            dirty = false;
        }

        glfwWaitEvents();
    }

    glfwTerminate();
    return 0;
}
