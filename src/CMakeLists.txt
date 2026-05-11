add_subdirectory(euler)

## Declare a C++ executable
add_executable(ikkdl ikkdl.cpp)
target_include_directories(ikkdl PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(ikkdl
	rclcpp::rclcpp
	interactive_markers::interactive_markers
	kdl_parser::kdl_parser
	tf2_kdl::tf2_kdl
	angles::angles
)

install(TARGETS ikkdl euler
	DESTINATION lib/${PROJECT_NAME}
)
