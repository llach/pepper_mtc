add_library(${PROJECT_NAME}_stages
        ${PROJECT_INCLUDE}/stages/mirror_grasp_generator.h

        mirror_grasp_generator.cpp
        bimanual_grasp_pose.cpp
)

target_link_libraries(${PROJECT_NAME}_stages ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
