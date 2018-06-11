cmake_minimum_required(VERSION 3.5)

project(mwave_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


file(GLOB_RECURSE SOURCES "src/*.cpp")
include_directories(include)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

add_library(${PROJECT_NAME} ${SOURCES})

# Dependents before Dependencies
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "rclcpp_lifecycle"
    "std_msgs"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RCLCPP_BUILDING_LIBRARY")
install(
	TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)

ament_package()

install(
	DIRECTORY include/
	DESTINATION include
)
