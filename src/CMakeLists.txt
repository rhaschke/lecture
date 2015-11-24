add_subdirectory(euler)

find_package(catkin REQUIRED COMPONENTS interactive_markers kdl_parser tf_conversions)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${EIGEN_INCLUDE_DIRS})

## enable c++11
add_definitions(-std=c++11)

## Declare a C++ executable
add_executable(ikkdl ikkdl.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ikkdl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ikkdl ${catkin_LIBRARIES})
