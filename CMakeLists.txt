cmake_minimum_required(VERSION 2.8.3)
project(localization_with_artrack_cv)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  tf2
  tf2_ros
  usb_cam
  ros_can_gps_msg
)

# link own build OpenCV(3.4.8) instead of ros-kinetic-opencv(3.3.1)
find_package(OpenCV 3.4 REQUIRED)

# for eigen3
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization_with_artrack_cv
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs tf2 tf2_ros usb_cam
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

MESSAGE(STATUS "This is the SOURCE dir Opencv" ${OpenCV_INCLUDE_DIRS})

add_executable(localization_cv src/localization_cv.cpp)
target_link_libraries(localization_cv ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ukf src/ukf.cpp)
target_link_libraries(ukf ${catkin_LIBRARIES})

add_executable(GPS_transfor src/GPS_transfor.cpp)
target_link_libraries(GPS_transfor ${catkin_LIBRARIES})


add_executable(VideoToRosTopic src/VideoToRosTopic.cpp)
target_link_libraries(VideoToRosTopic ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(RosTopicToImage src/RosTopicToImage.cpp)
target_link_libraries(RosTopicToImage ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(Drawmap src/Drawmap.cpp)
target_link_libraries(Drawmap ${catkin_LIBRARIES} ${OpenCV_LIBS})