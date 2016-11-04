QT5_WRAP_UI(UI_SOURCES
   ui/euler.ui
   ui/quaternion.ui
)

## Declare a C++ executable
add_executable(euler
   main.cpp
   MainWindow.cpp
   EulerWidget.cpp
   QuaternionWidget.cpp
   RotationControl.cpp
   Interpolation.cpp
   ${UI_SOURCES}
)

target_link_libraries(euler
   Qt5::Widgets
   rclcpp::rclcpp
   interactive_markers::interactive_markers
   angles::angles
   Eigen3::Eigen
)
