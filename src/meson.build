
sources = [
  'main.cpp',
  'UDPClient.cpp',
  'Kalman.cpp',
]

link_args = []
if host_machine.system() == 'windows'
    cpp_args += ['/std:c++17']
    link_args += ['-lws2_32']
else
    cpp_args += ['-std=c++17']
endif


if get_option('enable_sanitizers')
    link_args += ['-fsanitize=address']
endif

executable('kalman',
  sources,
  dependencies: eigen_dependency,
  link_args: link_args,
  install: true
)
