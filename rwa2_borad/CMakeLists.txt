cmake_minimum_required(VERSION 3.8)
project(rwa2_borad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include/rwa2_borad)
add_executable(thermostat_house src/thermostat_interface.cpp)
ament_target_dependencies(thermostat_house rclcpp std_msgs)

install(TARGETS 
        thermostat_house 
        DESTINATION lib/${PROJECT_NAME})

ament_package()
