cmake_minimum_required(VERSION 3.5)
project(igris_c_description_public)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf mujoco DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/imu_tf_broadcaster.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
