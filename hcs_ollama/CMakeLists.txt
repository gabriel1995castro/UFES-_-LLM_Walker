cmake_minimum_required(VERSION 3.8)
project(hcs_ollama)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Llamado de dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Importe de paquete de include

# Creacion de ejecutables de nodos

# Instalacion de source para ejecucion c++ y python
ament_python_install_package(${PROJECT_NAME})

# instalacion de paquetes adicionales
install(
  DIRECTORY config assets ollama_config map_config retriever_data
  DESTINATION share/${PROJECT_NAME}
)

# Instalacion de ejecutables de cpp

# Instalacion de ejecutables de python
install(PROGRAMS
  ${PROJECT_NAME}/ollama_node_vlm.py
  ${PROJECT_NAME}/ollama_node.py
  ${PROJECT_NAME}/ollama_vlsm_client.py
  ${PROJECT_NAME}/ollama_img_saver.py
  ${PROJECT_NAME}/multiagent_ollama_node.py
  ${PROJECT_NAME}/multi_movement_action.py
  DESTINATION lib/${PROJECT_NAME}
)

# instalacion de construccion de testeo
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
