#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "../comparison/earcut.hpp"
#include "../comparison/libtess2.hpp"

namespace mapbox {
namespace fixtures {
template <typename T>
using Polygon = std::vector<std::vector<T>>;
template <typename T>
using Triangles = std::vector<T>;
using DoublePoint = std::pair<double, double>;
using DoubleTriangles = Triangles<DoublePoint>;
using DoublePolygon = Polygon<DoublePoint>;
const double Infinity = std::numeric_limits<double>::infinity();

template <class T>
class Collector {
    std::vector<T> objects;
    Collector() = default;

public:
    static std::vector<T>& collection() {
        static Collector singleton;
        return singleton.objects;
    }
    static void add(T const& object) { collection().push_back(object); }
    static void remove(T const& object) {
        auto& objects = collection();
        objects.erase(std::remove(objects.begin(), objects.end(), object), objects.end());
    }
};

class FixtureTester {
public:
    struct TesselatorResult {
        std::vector<std::array<double, 2>> const& vertices;
        std::vector<uint32_t> const& indices;
    };
    const std::string name;
    const std::size_t expectedTriangles;
    const double expectedEarcutDeviation;
    const double expectedLibtessDeviation;
    FixtureTester(std::string testname, std::size_t triangles, double deviation, double libtessdeviation)
        : name(std::move(testname)),
          expectedTriangles(triangles),
          expectedEarcutDeviation(deviation),
          expectedLibtessDeviation(libtessdeviation) {
        Collector<FixtureTester*>::add(this);
    }
    virtual ~FixtureTester() { Collector<FixtureTester*>::remove(this); }
    virtual TesselatorResult earcut() = 0;
    virtual TesselatorResult libtess() = 0;
    virtual DoublePolygon const& polygon() = 0;
    virtual int64_t earcutTriangles() const { return 0; }
    static std::vector<FixtureTester*>& collection() {
        auto& objects = Collector<FixtureTester*>::collection();
        std::sort(objects.begin(), objects.end(), [](FixtureTester* a, FixtureTester* b) { return a->name < b->name; });
        return objects;
    }
};

template <class T>
class Fixture : public FixtureTester {
private:
    using InputPolygon = Polygon<std::pair<T, T>>;
    InputPolygon inputPolygon;
    DoublePolygon doublePolygon;
    EarcutTesselator<double, InputPolygon> earcutTesselator;
    Libtess2Tesselator<double, InputPolygon> libtessTesselator;

    // Only instantiated for integral T. Holds the int64 tesselator and a
    // proxy test fixture that registers itself in the collection.
    template <typename U, typename = void>
    struct Int64AdapterHelper {
        explicit Int64AdapterHelper(Fixture*) {}
    };

    template <typename U>
    struct Int64AdapterHelper<U, typename std::enable_if<std::is_integral<U>::value>::type> {
        class Proxy : public FixtureTester {
            Fixture* parent;
            EarcutTesselator<double, InputPolygon, int64_t> tesselator;
        public:
            Proxy(Fixture* p)
                : FixtureTester(p->name + "_i64", p->expectedTriangles,
                                std::max(p->expectedEarcutDeviation, 0.05), p->expectedLibtessDeviation),
                  parent(p), tesselator(p->inputPolygon) {}
            TesselatorResult earcut() override { tesselator.run(); return {tesselator.vertices(), tesselator.indices()}; }
            TesselatorResult libtess() override { return parent->libtess(); }
            DoublePolygon const& polygon() override { return parent->doublePolygon; }
        };

        Proxy proxy;
        explicit Int64AdapterHelper(Fixture* p) : proxy(p) {}
    };

    Int64AdapterHelper<T> int64Adapter;

public:
    Fixture(std::string const& name,
            std::size_t expectedTriangles,
            double expectedDeviation,
            double expectedLibtessDeviation,
            Polygon<std::pair<T, T>> const& p)
        : FixtureTester(name, expectedTriangles, expectedDeviation, expectedLibtessDeviation),
          inputPolygon(p),
                    earcutTesselator(inputPolygon),
                    libtessTesselator(inputPolygon),
                    int64Adapter(this) {
        doublePolygon.reserve(inputPolygon.size());
        for (auto& ring : inputPolygon) {
            std::vector<std::pair<double, double>> r;
            r.reserve(ring.size());
            for (auto& point : ring) {
                r.emplace_back(static_cast<double>(std::get<0>(point)), static_cast<double>(std::get<1>(point)));
            }
            doublePolygon.push_back(r);
        }
    }
    TesselatorResult earcut() override {
        earcutTesselator.run();
        return {earcutTesselator.vertices(), earcutTesselator.indices()};
    }
    TesselatorResult libtess() override {
        libtessTesselator.run();
        return {libtessTesselator.vertices(), libtessTesselator.indices()};
    }
    DoublePolygon const& polygon() override { return doublePolygon; }
};

} // namespace fixtures
} // namespace mapbox
