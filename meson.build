
project(
    'kalman_filter',
    ['cpp', 'c'],
    meson_version: '>=1.0.0',
    default_options: [ 'b_ndebug=if-release', 'buildtype=debug', 'cpp_std=c++14', 'default_library=static']
)

eigen_dependency = dependency('eigen', fallback: ['eigen', 'eigen_dep'])

# Add subdirectories
subdir('src')