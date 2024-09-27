project(
    'kalman_filter',
    ['cpp', 'c'],
    meson_version: '>=1.0.0',
    default_options: [ 'b_ndebug=if-release', 'buildtype=debug', 'cpp_std=c++17', 'default_library=static']
)

# Ensure C++17 standard is set explicitly
add_project_arguments('-std=c++17', language: 'cpp')

eigen_dependency = dependency('eigen', fallback: ['eigen', 'eigen_dep'])

subdir('src')
