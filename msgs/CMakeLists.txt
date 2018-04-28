cmake_minimum_required(VERSION 2.8.3)
project(pepper_mtc_msgs)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
        add_compile_options(-std=c++14)
else ()
        set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        actionlib_msgs
        std_msgs
        message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY srv
   FILES PepperFindGraspPlan.srv
   FILES PepperExecuteSolution.srv
   FILES PepperExecuteSolution.srv
)
## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES PepperExecuteSolution.action
         PepperVisualizeSolution.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES actionlib_msgs
                std_msgs  # Or other packages containing msgs
)

catkin_package()

