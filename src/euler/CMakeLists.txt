QT5_WRAP_UI(UI_SOURCES
   ui/euler.ui
   ui/quaternion.ui
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Qt5Widgets_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(euler
   main.cpp
   MainWindow.cpp
   EulerWidget.cpp
   QuaternionWidget.cpp
   RotationControl.cpp
   Interpolation.cpp
   ${UI_SOURCES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(euler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(euler ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
