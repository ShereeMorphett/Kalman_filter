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
sdl2_dependency = dependency('sdl2', fallback: ['sdl2', 'sdl2_dependency'])

# Get dependencies from subprojects
imgui = subproject('imgui')
implot = subproject('implot')


imgui_dep = imgui.get_variable('imgui_dep')
implot_dep = implot.get_variable('implot_dep')

# Include the source directory
subdir('src')
