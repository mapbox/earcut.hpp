{
  'target_defaults': {
    'default_configuration': 'Release',
    'cflags_cc': [ '-std=c++11', '-Wall', '-Wextra', '-Wshadow', '-fno-rtti', '-fexceptions' ],
    'cflags_cc': [ '-std=c99', '-Wall', '-Wextra', '-Wshadow' ],
    'xcode_settings': {
      'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
      'CLANG_C_LANGUAGE_STANDARD':'c99',
      'MACOSX_DEPLOYMENT_TARGET': '10.7',
      'CLANG_CXX_LIBRARY': 'libc++',
      'OTHER_CPLUSPLUSFLAGS': [ '-Wall', '-Wextra', '-Wshadow', '-fno-rtti', '-fexceptions' ],
      'OTHER_CFLAGS': [ '-Wall', '-Wextra', '-Wshadow' ],
    },
    'configurations': {
      'Debug': {
        'cflags_cc': [ '-g', '-O0', '-fno-omit-frame-pointer','-fwrapv', '-fstack-protector-all', '-fno-common' ],
        'defines': [ 'DEBUG' ],
        'xcode_settings': {
          'GCC_OPTIMIZATION_LEVEL': '0',
          'GCC_GENERATE_DEBUGGING_SYMBOLS': 'YES',
          'DEAD_CODE_STRIPPING': 'NO',
          'GCC_INLINES_ARE_PRIVATE_EXTERN': 'NO',
          'OTHER_CPLUSPLUSFLAGS': [ '-fno-omit-frame-pointer','-fwrapv', '-fstack-protector-all', '-fno-common']
        }
      },
      'Release': {
        'cflags_cc': [ '-g', '-O3' ],
        'defines': [ 'NDEBUG' ],
        'xcode_settings': {
          'GCC_OPTIMIZATION_LEVEL': '3',
          'GCC_GENERATE_DEBUGGING_SYMBOLS': 'YES',
          'DEAD_CODE_STRIPPING': 'YES',
          'GCC_INLINES_ARE_PRIVATE_EXTERN': 'NO'
        }
      },
    },
  },

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
        'test/test.cpp',
        'test/tap.cpp',
        'test/comparison/earcut.hpp',
        'test/comparison/libtess2.hpp',
      ],
    },

    { 'target_name': 'viz',
      'type': 'executable',
      'dependencies': [ 'fixtures', 'libtess2' ],
      'include_dirs': [
        'include',
      ],
      'libraries': [ '<@(glfw3_static_libs)' ],
      'xcode_settings': {
        'OTHER_CPLUSPLUSFLAGS': [ '<@(glfw3_cflags)' ],
        'OTHER_LDFLAGS': [ '<@(glfw3_ldflags)' ],
      },
      'cflags_cc': [ '<@(glfw3_cflags)' ],
      'sources': [
        'test/viz.cpp',
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
        'test/bench.cpp',
        'test/comparison/earcut.hpp',
        'test/comparison/libtess2.hpp',
      ],
    },
  ],
}
