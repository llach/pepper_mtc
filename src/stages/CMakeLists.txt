add_library(${PROJECT_NAME}_stages
        ${PROJECT_INCLUDE}/stages/mirror_grasp_generator.h
        ${PROJECT_INCLUDE}/stages/simple_mirror_grasp.h
        ${PROJECT_INCLUDE}/stages/compute_ik_mod.h

        mirror_grasp_generator.cpp
        simple_mirror_grasp.cpp
        compute_ik_mod.cpp
)

target_link_libraries(${PROJECT_NAME}_stages ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_stages
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
