cmake_minimum_required(VERSION 2.8.3)
project(lecture)

find_package(catkin REQUIRED COMPONENTS roscpp interactive_markers kdl_parser tf_conversions)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lecture
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

## QT stuff
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_compile_options(${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS})

add_subdirectory(src)
