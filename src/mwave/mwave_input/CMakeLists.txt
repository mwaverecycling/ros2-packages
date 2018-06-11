cmake_minimum_required(VERSION 3.5)

project(mwave_input)

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
find_package(mwave_util REQUIRED)
find_package(rcutils)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(i2cbridge REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rclcpp_lifecycle"
    "mwave_util"
    "rcutils"
    "std_msgs"
    "i2cbridge")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()


custom_executable(lifecycle input)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
