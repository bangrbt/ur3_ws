cmake_minimum_required(VERSION 3.8)
project(my_ur3_draw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED) # Bổ sung thư viện xử lý ảnh

add_executable(draw_letter_node src/draw_letter_node.cpp)

# Liên kết OpenCV vào Node C++
target_include_directories(draw_letter_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(draw_letter_node ${OpenCV_LIBRARIES})

ament_target_dependencies(draw_letter_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  visualization_msgs
  ament_index_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  draw_letter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES letter.png
  DESTINATION share/${PROJECT_NAME}
)

ament_package()