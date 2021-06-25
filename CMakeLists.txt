cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

add_compile_options(-std=c++11 -fpermissive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/QRDetector.cpp src/QRDetector.h)
target_link_libraries(main ${catkin_LIBRARIES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBRARIES})

include_directories(${ZBAR_INCLUDE_DIRS})
target_link_libraries(main ${ZBAR_LIBRARIES} zbar)
