cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

add_compile_options(-std=c++11 -fpermissive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Comm.msg
  Diagnostic.msg
  Monitor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime	
)

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

find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(main nlohmann_json::nlohmann_json)
