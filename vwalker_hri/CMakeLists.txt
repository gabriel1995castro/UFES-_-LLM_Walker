cmake_minimum_required(VERSION 3.8)
project(vwalker_hri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(force_filter_node src/force_filter_node.cpp)
ament_target_dependencies(force_filter_node PUBLIC rclcpp geometry_msgs)
target_include_directories(force_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(force_filter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS force_filter_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(admittance_controller_node src/admittance_controller_node.cpp)
ament_target_dependencies(admittance_controller_node PUBLIC rclcpp nav_msgs geometry_msgs message_filters)
target_include_directories(admittance_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(admittance_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS admittance_controller_node
  DESTINATION lib/${PROJECT_NAME})


  add_executable(virtual_trail_admittance_controller_node src/virtual_trail_admittance_controller_node.cpp)
  ament_target_dependencies(virtual_trail_admittance_controller_node PUBLIC rclcpp nav_msgs geometry_msgs message_filters std_msgs)
  target_include_directories(virtual_trail_admittance_controller_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(virtual_trail_admittance_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
  install(TARGETS virtual_trail_admittance_controller_node
    DESTINATION lib/${PROJECT_NAME})


add_executable(superviser_node src/superviser_node.cpp)
ament_target_dependencies(superviser_node PUBLIC rclcpp geometry_msgs std_msgs)
target_include_directories(superviser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(superviser_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS superviser_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(path_following src/path_following.cpp)
ament_target_dependencies(path_following PUBLIC rclcpp tf2_ros tf2_geometry_msgs std_msgs nav_msgs)

install(TARGETS path_following
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
  ${PROJECT_NAME}/scan_physical2virtual.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# # install the launch directory
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )


ament_package()