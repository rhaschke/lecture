cmake_minimum_required(VERSION 3.10.2)
project(lecture)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(angles REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)

## QT stuff
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_compile_options(${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS})

add_subdirectory(src)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY notebook/
  DESTINATION share/${PROJECT_NAME}/notebook
  FILES_MATCHING PATTERN "*.rviz" PATTERN "*.launch.*"
)

install(PROGRAMS
  notebook/controller.py
  notebook/fk.py
  notebook/markers.py
  notebook/robot_model.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
