{
  'includes': [
    'common.gypi',
  ],

  'targets': [
    { 'target_name': 'fixtures',
      'type': 'static_library',
      'sources': [
        '<!@(find test/fixtures -name "*.cpp" -o -name "*.hpp")',
      ],
    },

    { 'target_name': 'libtess2',
      'type': 'static_library',
      'sources': [
        '<!@(find test/comparison/libtess2 -name "*.c" -o -name "*.h")',
      ],
    },

    { 'target_name': 'test',
      'type': 'executable',
      'dependencies': [ 'fixtures', 'libtess2' ],
      'include_dirs': [
        'include',
      ],
      'sources': [
        'include/earcut.hpp',
        'test/test.cpp',
        'test/tap.cpp',
        'test/comparison/earcut.hpp',
        'test/comparison/libtess2.hpp',
      ],
    },

    { 'target_name': 'bench',
      'type': 'executable',
      'dependencies': [ 'fixtures', 'libtess2' ],
      'include_dirs': [
        'include',
      ],
      'sources': [
        'include/earcut.hpp',
        'test/bench.cpp',
        'test/comparison/earcut.hpp',
        'test/comparison/libtess2.hpp',
      ],
    },
  ],
}
