# Define your source files
sources = [
  'main.cpp',
  'UDPClient.cpp',
  'Kalman.cpp',
]

# Conditionally add Windows-specific linker arguments
link_args = []
if host_machine.system() == 'windows'
    link_args += ['-lws2_32']
endif

# Define your executable, passing all arguments including platform-specific ones
executable('kalman',
  sources,
  dependencies: eigen_dependency,
  link_args: link_args,
  install: true
)
