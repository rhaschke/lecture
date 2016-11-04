set(UI_FILES
   ui/euler.ui
   ui/quaternion.ui
)

if (QT_VERSION_MAJOR EQUAL 5)
   qt5_wrap_ui(UI_SOURCES ${UI_FILES})
else()
   qt6_wrap_ui(UI_SOURCES ${UI_FILES})
endif()

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
   Qt${QT_VERSION_MAJOR}::Widgets
   rclcpp::rclcpp
   interactive_markers::interactive_markers
   angles::angles
   Eigen3::Eigen
)
