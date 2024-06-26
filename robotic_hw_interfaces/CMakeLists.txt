cmake_minimum_required(VERSION 3.8)
project(robotic_hw_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/LoadCellData.msg"
    "msg/LoadCellDataArray.msg"
    "srv/GetLoadCellData.srv"
    DEPENDENCIES builtin_interfaces 
    ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
