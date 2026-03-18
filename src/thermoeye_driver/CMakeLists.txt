cmake_minimum_required(VERSION 3.8)
project(thermoeye_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# TmSDK 경로 설정
include_directories(/usr/include/TmSDK)
link_directories(/usr/lib/TmSDK)

# 실행 파일 선언
add_executable(centerdegree_node src/centerdegree_pub.cpp)
add_executable(thermo_pub_node src/thermo_pub.cpp)
add_executable(video_pub_node src/video_pub.cpp)

# 1. centerdegree_node 설정
ament_target_dependencies(centerdegree_node rclcpp std_msgs)
target_link_libraries(centerdegree_node
  /usr/lib/TmSDK/libTmCore.2.0.0.so
  /usr/lib/TmSDK/libTmRtspClient.1.1.4.so
  pthread
)

# 2. thermo_pub_node 설정 (여기가 주요 수정 포인트입니다)
# ROS 관련 의존성은 ament_target_dependencies에 넣어야 경로를 제대로 찾습니다.
ament_target_dependencies(thermo_pub_node rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(thermo_pub_node
  /usr/lib/TmSDK/libTmCore.2.0.0.so
  /usr/lib/TmSDK/libTmRtspClient.1.1.4.so
  pthread
)

# 3. video_pub_node 설정
ament_target_dependencies(video_pub_node rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(video_pub_node
  ${OpenCV_LIBS}
  /usr/lib/TmSDK/libTmCore.2.0.0.so
  /usr/lib/TmSDK/libTmRtspClient.1.1.4.so
  pthread
)

# 설치 설정
install(TARGETS 
  centerdegree_node
  video_pub_node
  thermo_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()