cmake_minimum_required(VERSION 2.8.3)
project(karto_scan_matcher)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  roscpp
  rosconsole
  sensor_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/karto_scan_matcher_node.cpp src/Mapper.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
