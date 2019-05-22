cmake_minimum_required(VERSION 3.5)
project(interactive_markers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(AMENT_PKGS
  rclcpp
  rclpy
  #rosconsole
  #rostest
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(ament_cmake REQUIRED)

foreach(ament_pkg IN LISTS AMENT_PKGS)
  find_package(${ament_pkg} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/interactive_marker_server.cpp
  src/tools.cpp
  src/menu_handler.cpp
  src/interactive_marker_client.cpp
  src/single_client.cpp
  src/message_context.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(${PROJECT_NAME} ${AMENT_PKGS})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_interfaces(${PROJECT_NAME})
ament_export_dependencies(${AMENT_PKGS})

install(TARGETS ${PROJECT_NAME}
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

ament_package()

# C++ Unit Tests
#
#if(CATKIN_ENABLE_TESTING)
#  include_directories(${GTEST_INCLUDE_DIRS})

#  add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
#  target_link_libraries(server_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests server_test)
#  add_rostest(test/cpp_server.test)

#  add_executable(client_test EXCLUDE_FROM_ALL src/test/client_test.cpp)
#  target_link_libraries(client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests client_test)
#  add_rostest(test/cpp_client.test)

#  add_executable(server_client_test EXCLUDE_FROM_ALL src/test/server_client_test.cpp)
#  target_link_libraries(server_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests server_client_test)
#  add_rostest(test/cpp_server_client.test)

  # Test program to simulate Interactive Marker with missing tf information
  #  add_executable(bursty_tf EXCLUDE_FROM_ALL src/test/bursty_tf.cpp)
  #  target_link_libraries(bursty_tf ${PROJECT_NAME})
  #  add_dependencies(tests bursty_tf)

  # Test program to simulate Interactive Marker with wrong tf information
  #  add_executable(missing_tf EXCLUDE_FROM_ALL src/test/missing_tf.cpp)
  #  target_link_libraries(missing_tf ${PROJECT_NAME})
  #  add_dependencies(tests missing_tf)
#endif()

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interactive_markers
#  CATKIN_DEPENDS roscpp rosconsole rospy tf visualization_msgs
#)
#catkin_python_setup()
