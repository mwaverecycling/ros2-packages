cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME mwave_messages)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB MSGS "msg/*.msg")
file(GLOB SRVS "srv/*.srv")

rosidl_generate_interfaces(mwave_messages
    ${MSGS}
    ${SRVS}
    DEPENDENCIES builtin_interfaces
)

ament_package()
