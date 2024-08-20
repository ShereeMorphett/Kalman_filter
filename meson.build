
project(
    'kalman_filter',
    ['cpp', 'c'],
    meson_version: '>=1.0.0',
    default_options: [ 'b_ndebug=if-release', 'buildtype=debug', 'cpp_std=c++14', 'default_library=static']
)
# Include directories
include_directories = include_directories('include')

# Add subdirectories
subdir('src')
subdir('tests')