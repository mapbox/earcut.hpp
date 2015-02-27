{
  'includes': [
    '../common.gypi',
  ],

  'targets': [
    { 'target_name': 'viz',
      'type': 'executable',
      'dependencies': [ '../earcut.gyp:fixtures', '../earcut.gyp:libtess2' ],
      'include_dirs': [
        '../include',
      ],
      'libraries': [ '<@(glfw3_static_libs)' ],
      'xcode_settings': {
        'OTHER_CPLUSPLUSFLAGS': [ '<@(glfw3_cflags)' ],
        'OTHER_LDFLAGS': [ '<@(glfw3_ldflags)' ],
      },
      'cflags_cc': [ '<@(glfw3_cflags)' ],
      'sources': [
        '../include/earcut.hpp',
        'viz.cpp',
        'comparison/earcut.hpp',
        'comparison/libtess2.hpp',
      ],
    },
  ],
}
