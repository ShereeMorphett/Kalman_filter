project(
    'kalman_filter',
    ['cpp', 'c'],
    meson_version: '>=1.0.0',
    default_options: [ 
        'b_ndebug=if-release', 
        'buildtype=debug', 
        'cpp_std=c++17', 
        'default_library=static'
    ]
)

# Get the value of the enable_sanitizers option
enable_sanitizers = get_option('enable_sanitizers')

# Add sanitizer flags if the option is enabled
if enable_sanitizers
    add_project_arguments('-fsanitize=address', language: 'cpp')
    add_project_arguments('-fsanitize=address', language: 'c')
    add_project_arguments('-g', language: 'cpp')
add_project_arguments('-g', language: 'c')
endif

# Ensure C++17 standard is set explicitly
add_project_arguments('-std=c++17', language: 'cpp')

# Set up the eigen dependency
eigen_dependency = dependency('eigen', fallback: ['eigen', 'eigen_dep'])

# Include the source directory
subdir('src')
