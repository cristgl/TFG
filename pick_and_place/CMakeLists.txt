cmake_minimum_required(VERSION 2.8.3)
project(pick_and_place)

include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
)

catkin_package(
  LIBRARIES pick_and_place
  CATKIN_DEPENDS OpenCv roscpp sensor_msg/Image
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(baxter_imagen baxter_img.cpp)
target_link_libraries(baxter_imagen
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${sensor_msgs_LIBS}
)

