cmake_minimum_required(VERSION 3.8)
project(hcs_ollama_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Response.msg"
  "msg/PartialResponse.msg"
  "msg/SystemConfigurationAi.msg"
  "msg/EvaluatorResponse.msg"
  "msg/LocalizatorResponse.msg"
  "msg/ExecutorResponse.msg"
  "msg/PartialMultiResponse.msg"
  "msg/ObjectiveCoordinates.msg"
  "msg/FinalMultiResponse.msg"
  "msg/MultiAgentResponse.msg"
  "srv/Tokenize.srv"
  "srv/Detokenize.srv"
  "srv/Embedding.srv"
  "srv/DistanceChangeService.srv"
  "action/GetLlmResponse.action"
  "action/GetVlmResponse.action"
  "action/GetMultiAgentResponse.action"
  "action/GetMultiAgentMoveRespose.action"
  DEPENDENCIES sensor_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
