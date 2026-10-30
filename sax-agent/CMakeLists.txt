cmake_minimum_required(VERSION 3.16)
project(sax_agent VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SAX_BUILD_NODE "Build the ROS 2 node and the ixwebsocket client" ON)
option(SAX_BUILD_TESTS "Build the unit tests" ON)

find_package(Threads REQUIRED)

function(sax_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
  endif()
endfunction()

# Core library: SX1 encoder, voxelizer, command parser and agent logic. No third-party dependencies.
add_library(sax_agent_core STATIC
  src/sx1_encoder.cpp
  src/command_parser.cpp
  src/agent_core.cpp)
target_include_directories(sax_agent_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(sax_agent_core PUBLIC cxx_std_20)
target_link_libraries(sax_agent_core PUBLIC Threads::Threads)
sax_warnings(sax_agent_core)

if(SAX_BUILD_NODE)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  find_package(ixwebsocket QUIET)
  if(NOT ixwebsocket_FOUND)
    message(FATAL_ERROR
      "ixwebsocket was not found. Install it with 'sudo apt install libixwebsocket-dev' or build it from "
      "https://github.com/machinezone/IXWebSocket (cmake -DUSE_TLS=1 -DUSE_ZLIB=1), or configure with "
      "-DSAX_BUILD_NODE=OFF to build only the core library and tests.")
  endif()

  add_library(sax_agent_ws STATIC src/websocket_client.cpp)
  target_link_libraries(sax_agent_ws PUBLIC sax_agent_core ixwebsocket::ixwebsocket)
  sax_warnings(sax_agent_ws)

  # ament_target_dependencies picks its own target_link_libraries signature. Applying it to an INTERFACE
  # library keeps that choice off the executable, so the keyword signature below is valid on every distro.
  add_library(sax_agent_ros_deps INTERFACE)
  ament_target_dependencies(sax_agent_ros_deps INTERFACE
    rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs tf2 tf2_ros)

  add_executable(agent_node src/agent_node.cpp)
  target_link_libraries(agent_node PRIVATE sax_agent_ws sax_agent_ros_deps)
  sax_warnings(agent_node)

  install(TARGETS agent_node RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/run_agent_cli.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
endif()

if(SAX_BUILD_TESTS)
  enable_testing()

  foreach(test_name IN ITEMS test_sx1_encoder test_command_parser test_agent_core)
    add_executable(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE test)
    target_link_libraries(${test_name} PRIVATE sax_agent_core)
    sax_warnings(${test_name})
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()

  # Runs against the in-process ixwebsocket fake under test/fakes, so it needs neither a network nor the real library.
  add_executable(test_websocket_client test/test_websocket_client.cpp src/websocket_client.cpp)
  target_include_directories(test_websocket_client BEFORE PRIVATE test/fakes test)
  target_link_libraries(test_websocket_client PRIVATE sax_agent_core)
  sax_warnings(test_websocket_client)
  add_test(NAME test_websocket_client COMMAND test_websocket_client)
endif()

if(SAX_BUILD_NODE)
  ament_package()
endif()
