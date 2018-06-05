cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME mwave_orchestrator)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(mwave_config REQUIRED)

add_executable(config
  src/services/mariadb.cpp
)
target_link_libraries(config
  mariadbclientpp
  mysqlclient
)
ament_target_dependencies(config
  "rclcpp"
  "rcutils"
  "mwave_config"
) 

install(TARGETS
config
DESTINATION lib/${PROJECT_NAME})

ament_package()
