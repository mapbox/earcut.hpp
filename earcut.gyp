{
  'target_defaults': {
    'default_configuration': 'Release',
    'cflags_cc': [ '-std=c++11', '-Wall', '-Wextra', '-Wshadow', '-frtti', '-fexceptions' ],
    'xcode_settings': {
      'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
      'MACOSX_DEPLOYMENT_TARGET': '10.7',
      'CLANG_CXX_LIBRARY': 'libc++',
      'OTHER_CPLUSPLUSFLAGS': [ '-Wall', '-Wextra', '-Wshadow', '-frtti', '-fexceptions' ],
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
    { 'target_name': 'test',
      'product_name': 'test',
      'type': 'executable',
      'include_dirs': [
        'include',
      ],
      'sources': [
        'src/main.cpp',
        'src/impl.cpp',
        'include/earcut.hpp',
      ],
    },
  ],
}
