cmake_minimum_required(VERSION 2.8.3)
project(karto_scan_matcher)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES karto_scan_macther
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(karto_scan_macther SHARED src/Mapper.cpp)
target_link_libraries(karto_scan_macther ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto_scan_macther
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
