cmake_minimum_required(VERSION 2.8.3)
project(pepper_mtc)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
        add_compile_options(-std=c++14)
else ()
        set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        moveit_ros_planning_interface
        moveit_task_constructor_core
        rviz_marker_tools
        roslaunch
        std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PepperGrasping.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

##################
## launch files ##
##################

roslaunch_add_file_check(launch)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_stages
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_subdirectory(src/stages)

add_executable(pepper src/pepper.cpp)
target_link_libraries(pepper ${catkin_LIBRARIES} ${PROJECT_NAME}_stages)

add_executable(object src/object.cpp)
target_link_libraries(object ${catkin_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
