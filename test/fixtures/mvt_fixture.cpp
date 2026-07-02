#include "mvt_fixture.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifndef EARCUT_TILES_BIN
#error "EARCUT_TILES_BIN must be defined (path to the fetched JS bench/tiles-fixture.bin)"
#endif

namespace mapbox {
namespace fixtures {
namespace {

// Little-endian LEB128 unsigned varint reader over a byte buffer + cursor (mirrors the JS decoder).
uint32_t readVarint(const std::string& buf, std::size_t& pos) {
    uint32_t val = 0;
    int shift = 0;
    uint8_t b;
    do {
        if (pos >= buf.size()) throw std::runtime_error("tiles-fixture.bin: varint runs past EOF");
        b = static_cast<uint8_t>(buf[pos++]);
        val |= static_cast<uint32_t>(b & 0x7f) << shift;
        shift += 7;
    } while (b & 0x80);
    return val;
}

int32_t zigZagDecode(uint32_t n) {
    return static_cast<int32_t>(n >> 1) ^ -static_cast<int32_t>(n & 1);
}

// Signed ring area (shoelace), sign preserved to classify exterior (>0) vs hole (<0).
double ringArea(const std::vector<MvtPoint>& ring) {
    double sum = 0;
    for (std::size_t i = 0, len = ring.size(), j = len - 1; i < len; j = i++) {
        sum += double(ring[j].first - ring[i].first) * double(ring[i].second + ring[j].second);
    }
    return sum / 2;
}

// Decode one feature's raw MVT command/parameter integers into rings.
std::vector<std::vector<MvtPoint>> decodeMvtRings(const std::vector<uint32_t>& geom) {
    std::vector<std::vector<MvtPoint>> rings;
    int x = 0, y = 0;
    std::vector<MvtPoint> ring;
    bool haveRing = false;
    std::size_t i = 0;

    while (i < geom.size()) {
        const uint32_t cmd = geom[i] & 0x7;
        const uint32_t count = geom[i] >> 3;
        i++;

        if (cmd == 1) { // MoveTo: starts a new ring
            for (uint32_t k = 0; k < count; k++) {
                x += zigZagDecode(geom[i++]);
                y += zigZagDecode(geom[i++]);
                if (haveRing) rings.push_back(std::move(ring));
                ring = {{x, y}};
                haveRing = true;
            }
        } else if (cmd == 2) { // LineTo
            for (uint32_t k = 0; k < count; k++) {
                x += zigZagDecode(geom[i++]);
                y += zigZagDecode(geom[i++]);
                ring.emplace_back(x, y);
            }
        } else if (cmd == 7 && haveRing) { // ClosePath
            rings.push_back(std::move(ring));
            ring.clear();
            haveRing = false;
        }
    }

    if (haveRing) rings.push_back(std::move(ring));
    return rings;
}

std::vector<MvtFeature> decodeTilesFixture() {
    std::ifstream stream(EARCUT_TILES_BIN, std::ios::binary);
    if (!stream) throw std::runtime_error("failed to open tiles fixture: " EARCUT_TILES_BIN);
    std::stringstream sbuf;
    sbuf << stream.rdbuf();
    const std::string buf = sbuf.str();

    std::vector<MvtFeature> features;
    std::size_t pos = 0;

    while (pos < buf.size()) {
        const int z = static_cast<int>(readVarint(buf, pos));
        const uint32_t featureCount = readVarint(buf, pos);

        for (uint32_t f = 0; f < featureCount; f++) {
            const uint32_t count = readVarint(buf, pos);
            std::vector<uint32_t> geom(count);
            for (uint32_t i = 0; i < count; i++) geom[i] = readVarint(buf, pos);

            // Split multipolygons: a positive-area ring opens a new polygon, negative-area rings
            // become holes of the current one (per the MVT spec / JS reference decoder).
            MvtPolygon current;
            auto flush = [&] {
                if (current.empty()) return;
                std::size_t verts = 0;
                for (const auto& ring : current) verts += ring.size();
                const std::size_t holes = current.size() - 1;
                features.push_back({std::move(current), verts, holes, z});
                current.clear();
            };

            for (auto& ring : decodeMvtRings(geom)) {
                if (ring.size() < 3) continue;
                const double area = ringArea(ring);
                if (area == 0) continue;
                if (area > 0) {
                    flush();
                    current.push_back(std::move(ring));
                } else if (!current.empty()) {
                    current.push_back(std::move(ring));
                }
            }
            flush();
        }
    }

    return features;
}

} // namespace

const std::vector<MvtFeature>& mvtFixture() {
    static const std::vector<MvtFeature> features = decodeTilesFixture();
    return features;
}

} // namespace fixtures
} // namespace mapbox
