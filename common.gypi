{
  'target_defaults': {
    'default_configuration': 'Release',
    'cflags': [ '-std=c99', '-Wall', '-Wextra', '-Wshadow' ],
    'cflags_cc': [ '-std=c++11', '-Wall', '-Wextra', '-Wshadow', '-fno-rtti', '-fexceptions' ],
    'xcode_settings': {
      'CLANG_CXX_LANGUAGE_STANDARD':'c++11',
      'GCC_C_LANGUAGE_STANDARD':'c99',
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
}
