
sources = [
  'main.cpp',
  'UDPClient.cpp',
  'Parser.cpp',
  'KalmanMatrices.cpp',
  'Kalman.cpp',
]

link_args = []
if host_machine.system() == 'windows'
    link_args += ['-lws2_32']
endif

if get_option('enable_sanitizers')
    link_args += ['-fsanitize=address']
endif

executable('kalman',
  sources,
  dependencies: [eigen_dependency, imgui_dep, implot_dep, sdl2_dependency],
  link_args: link_args,
  install: true
)
