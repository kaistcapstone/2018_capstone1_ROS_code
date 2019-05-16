cmake_minimum_required(VERSION 2.8.3)
project(data_integrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  dynamixel_sdk
  geometry_msgs
)

find_package(OpenCV REQUIRED )
find_package(Curses REQUIRED )

include_directories( ${CURSES_INCLUDE_DIRS} )

include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include/dynamixel_sdk
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

  add_library(data_integrate
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
  )

add_executable(data_show_node src/data_show.cpp)
add_dependencies(data_show_node core_msgs_generate_messages_cpp)

add_executable(data_integation_node src/data_integration.cpp)
add_dependencies(data_integation_node core_msgs_generate_messages_cpp)

add_executable(integration_test_node src/integration_test.cpp)
add_dependencies(integration_test_node core_msgs_generate_messages_cpp)

add_executable(keyboard_ctrl src/keyboard_ctrl.cpp)
add_dependencies(keyboard_ctrl core_msgs_generate_messages_cpp)

add_executable(arrange_test src/arrange_test.cpp)
add_dependencies(arrange_test core_msgs_generate_messages_cpp)

add_executable(station_test src/station_test.cpp)
add_dependencies(station_test core_msgs_generate_messages_cpp)

add_executable(vibration_analysis src/vibration_analysis.cpp)
add_dependencies(vibration_analysis core_msgs_generate_messages_cpp)

target_link_libraries(data_show_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(data_integation_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(integration_test_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(keyboard_ctrl
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} ${cv_bridge_LIBRARIES}
)
target_link_libraries(arrange_test
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} ${cv_bridge_LIBRARIES}
)
target_link_libraries(station_test
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} ${cv_bridge_LIBRARIES}
)
target_link_libraries(vibration_analysis
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} ${cv_bridge_LIBRARIES}
)

# install(TARGETS dynamixel_sdk
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
