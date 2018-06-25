cmake_minimum_required(VERSION 3.5)

project(i2cbridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()
if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mwave_util REQUIRED)
find_package(mwave_messages REQUIRED)


include_directories(include)
#include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")
file(GLOB_RECURSE ${PROJECT_NAME}_SRCS "src/*.cpp")
message(${${PROJECT_NAME}_SRCS})

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# Dependents before Dependencies
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "std_msgs"
    "mwave_util"
    "mwave_messages"
)
target_link_libraries(${PROJECT_NAME} i2cpp)

if(NOT LINUX)
	set(MOCK $ENV{I2CPP})
	message(STATUS "Including mock resources from ${MOCK}")
	target_include_directories(${PROJECT_NAME}
		PUBLIC "${MOCK}/include")
	link_directories("${MOCK}/lib")
endif()

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
ament_export_dependencies(mwave_util)
ament_export_dependencies(mwave_messages)
ament_export_dependencies(std_msgs)

ament_package()

install(
	DIRECTORY include/
	DESTINATION include
)
