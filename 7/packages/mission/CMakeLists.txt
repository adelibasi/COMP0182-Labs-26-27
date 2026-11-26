cmake_minimum_required(VERSION 3.8)
project(mission)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(lab_dir "${CMAKE_CURRENT_SOURCE_DIR}/../..")
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module("${lab_dir}/scripts/sim_assets.py")
install(PROGRAMS mission/autonomous_search.py DESTINATION lib/${PROJECT_NAME})
install(FILES "${lab_dir}/config/static/sim_task.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
