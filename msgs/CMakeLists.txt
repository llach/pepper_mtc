cmake_minimum_required(VERSION 2.8.3)
project(pepper_mtc_msgs)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
        add_compile_options(-std=c++14)
else ()
        set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PepperFindGraspPlan.srv
)
## Generate actions in the 'action' folder
add_action_files(
   FILES
   PepperExecuteSolution.action
   PepperVisualizeSolution.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

