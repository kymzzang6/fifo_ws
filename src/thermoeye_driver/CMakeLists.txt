cmake_minimum_required(VERSION 3.8)
project(thermoeye_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1) TmSDK 헤더 경로
include_directories(
  /usr/include/TmSDK
)

# 2) TmSDK 라이브러리 경로
link_directories(
  /usr/lib/TmSDK
)

add_executable(centerdegree_node src/centerdegree_pub.cpp)
ament_target_dependencies(centerdegree_node rclcpp std_msgs)

# 3) TmSDK 라이브러리 이름 (libTmCore.so => TmCore, libTmRtspClient.so => TmRtspClient)
target_link_libraries(centerdegree_node
  /usr/lib/TmSDK/libTmCore.1.2.0.so
  /usr/lib/TmSDK/libTmRtspClient.1.1.3.so
  pthread
)


install(TARGETS centerdegree_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
