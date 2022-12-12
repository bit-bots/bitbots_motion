cmake_minimum_required(VERSION 3.5)
project(bitbots_hcm)

set(PYBIND11_PYTHON_VERSION 3)
set(PYBIND11_FINDPYTHON ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(biped_interfaces REQUIRED)

find_package(bitbots_splines REQUIRED)
find_package(bitbots_quintic_walk REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_league_msgs REQUIRED)
find_package(irobot_events_executor REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(rot_conv REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fvisibility=hidden)
endif()

include_directories(include ${PYTHON_INCLUDE_DIRS})
set(SOURCES src/hcm.cpp
        src/walk_visualizer.cpp
        src/walk_engine.cpp
        src/walk_stabilizer.cpp
        src/walk_ik.cpp
        src/walk_node.cpp)

add_executable(HCM ${SOURCES})

ament_target_dependencies(HCM
    ament_cmake
    backward_ros
    bitbots_msgs
    bitbots_splines
    biped_interfaces
    bitbots_quintic_walk    
    builtin_interfaces
    control_msgs
    control_toolbox
    geometry_msgs
    geometry_msgs
    humanoid_league_msgs
    irobot_events_executor
    moveit_ros
    moveit_ros_planning_interface
    nav_msgs
    rclcpp
    ros2_python_extension
    rot_conv
    sensor_msgs
    std_msgs    
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros)

target_link_libraries(HCM ${PYTHON_LIBRARIES})

enable_bitbots_docs()

install(TARGETS HCM
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/ USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bitbots_hcm/classifier DESTINATION share/${PROJECT_NAME})

install(DIRECTORY bitbots_hcm/hcm_dsd/actions DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")

install(DIRECTORY bitbots_hcm/hcm_dsd/decisions DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")

install(FILES bitbots_hcm/hcm_dsd/hcm.dsd DESTINATION share/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})
ament_package()
