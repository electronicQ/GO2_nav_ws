cmake_minimum_required(VERSION 3.8)
project(go2_twist_bridge)

find_package(ament_cmake REQUIRED)

# Remove the obsolete installed executable when upgrading an existing workspace.
install(CODE "file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/twist_bridge\")")

install(PROGRAMS scripts/sdk_twist_bridge.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
