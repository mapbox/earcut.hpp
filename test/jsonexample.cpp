/*
A Simple Example of using Earcut to read GeoJSON style data points,
using nlohmann's single-header json parser, available at
https://github.com/nlohmann/json/blob/develop/include/nlohmann/json.hpp
Input: Json files, ex "[ [[0,0],[10,0],[10,10]], [[1,1],[9,0],[9,9]] ]"
(https://github.com/mapbox/earcut/tree/master/test/fixtures has more)
Output: simple count report. Triangle indexes are in 'tris' vector.
*/

#include "earcut.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include "json.hpp"

using Coord = double;
using Point = std::array<Coord, 2>;
using N = uint32_t;
using json = nlohmann::json;
using namespace std;
int main() {
        int numpts = 0;
        std::vector<std::vector<Point>> polygon;
        ifstream f("water.json"); // from earcut.js fixtures
        json j;
        f >> j;
        for (json::iterator it = j.begin(); it != j.end(); ++it) {
                polygon.push_back(it->get<std::vector<Point>>());
                numpts += polygon[polygon.size()-1].size();
        }
        cout << "number of points input: " << numpts << endl;
        cout << "number of holes: " << polygon.size()-1 << endl;
        std::vector<N> tris = mapbox::earcut<N>(polygon);
        cout << "number of triangles output: " << tris.size()/3 << endl;
}


