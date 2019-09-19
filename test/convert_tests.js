'use strict';
/* jshint node: true */

var fs = require('fs');
var path = require('path');
var earcut = require('../../earcut/src/earcut.js');

var integerPolygons = '';
var doublePolygons = '';

var base = '../earcut/test/fixtures';
fs.readdirSync(base).filter(function (name) {
    return path.extname(name) === '.json';
}).forEach(function (name) {
    var json = JSON.parse(fs.readFileSync(path.join(base, name), 'utf-8'));
    var data = earcut.flatten(json),
        indices = earcut(data.vertices, data.holes, data.dimensions),
        deviation = earcut.deviation(data.vertices, data.holes, data.dimensions, indices);

    var id = path.basename(name, path.extname(name)).replace(/[^a-z0-9]+/g, '_');

    var integer = true;
    var short_integer = true;

    function processPoint(p) {
        if (integer && (p[0] % 1 !== 0 || p[1] % 1 !== 0)) {
            integer = false;
            short_integer = false;
        }
        if (short_integer && (p[0] < -32767 || p[0] > 32767 || p[1] < -32767 || p[1] > 32767)) {
            short_integer = false;
        }
        return p.join(',');
    }

    var geometry = '';
    for (var i = 0; i < json.length; i++) {
        geometry += '    {{' + (json[i].map(processPoint).join('},{')) + '}},\n';
    }

    var className = "Fixture<double>"
    if (short_integer) {
        className = "Fixture<short>"
    } else if (integer) {
        className = "Fixture<int>"
    }

    var expectedTriangles = indices.length / 3;
    var expectedDeviation = deviation;
    expectedDeviation += 1e-14;
    var libtessDeviationMap = {
        "water": 0.00002,
        "water_huge": 0.0002,
        "water_huge2": 0.00015,
        "bad_hole": 0.0022,
        "issue16": 0.0255,
        "self_touching": 0.002,
        "simplified_us_border": 0.001,
        "issue45": 0.094,
        "empty_square": Infinity,
        "issue83": Infinity,
        "issue107": Infinity,
        "issue119": 0.04,
        "touching4": 0.06
    };
    var expectedLibtessDeviation = libtessDeviationMap[id];
    if (!expectedLibtessDeviation) expectedLibtessDeviation = 0.000001;
    var cpp = '// This file is auto-generated, manual changes will be lost if the code is regenerated.\n\n';
    cpp += '#include "geometries.hpp"\n\n';
    cpp += 'namespace mapbox {\n';
    cpp += 'namespace fixtures {\n\n';
    cpp += 'static const ' + className + ' ' + id + '("' + id + '", ' + expectedTriangles + ', ' + expectedDeviation + ', ' + expectedLibtessDeviation +', {\n';
    cpp += geometry;
    cpp += '});\n\n';
    cpp += '}\n';
    cpp += '}\n';

    fs.writeFileSync('test/fixtures/' + id + '.cpp', cpp);
});
