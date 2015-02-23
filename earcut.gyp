{
  'target_defaults': {
    'default_configuration': 'Release',
    'cflags_cc': [ '-std=c++11', '-stdlib=libc++', '-Wall', '-Wextra', '-Wshadow', '-fno-rtti', '-fexceptions' ],
    'xcode_settings': {
      'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
      'MACOSX_DEPLOYMENT_TARGET': '10.7',
      'CLANG_CXX_LIBRARY': 'libc++',
      'OTHER_CPLUSPLUSFLAGS': [ '-stdlib=libc++', '-Wall', '-Wextra', '-Wshadow', '-fno-rtti', '-fexceptions' ],
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

    { 'target_name': 'test',
      'type': 'executable',
      'dependencies': [ 'fixtures' ],
      'include_dirs': [
        'include',
      ],
      'sources': [
        'test/test.cpp',
        'test/tap.cpp',
      ],
    },

    { 'target_name': 'bench',
      'type': 'executable',
      'dependencies': [ 'fixtures' ],
      'include_dirs': [
        'include',
      ],
      'sources': [
        'test/bench.cpp',
      ],
    },
  ],
}
