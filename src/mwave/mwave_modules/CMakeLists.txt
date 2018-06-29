cmake_minimum_required(VERSION 3.5)

project(mwave_modules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

message("Looking in '${CMAKE_CURRENT_LIST_DIR}'")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(mwave_util REQUIRED)
find_package(i2cbridge REQUIRED)

include_directories("include")
include_directories("${i2cbridge_INCLUDE_DIRS}")
file(GLOB_RECURSE SOURCES "src/*.cpp")


add_executable(seed ${SOURCES})
ament_target_dependencies(seed
    "rclcpp"
    "rcutils"
    "std_msgs"
    "mwave_util"
    "i2cbridge")
target_link_libraries(seed
    ${i2cbridge_LIBRARIES})

install(TARGETS seed
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
