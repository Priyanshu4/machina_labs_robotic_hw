cmake_minimum_required(VERSION 3.5)
project(robotic_hw)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robotic_hw_interfaces REQUIRED)

# Include headers
include_directories(include)

# Add executable
add_executable(load_cell_data_publisher src/load_cell_data_publisher.cpp)

# Link dependencies
ament_target_dependencies(load_cell_data_publisher 
    rclcpp 
    rclpy
    robotic_hw_interfaces 
)
    
# Install executable
install(TARGETS load_cell_data_publisher
    DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()