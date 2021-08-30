###############################realsense image save######################

add_executable( image_save
    EKF.cpp
    camera.cpp
    realsense_image.cpp
    image.cpp
)

target_link_libraries( image_save
    ${THIRD_PARTY_LIBS}
)

###############################pose to camera######################
#add_executable( pose_camera
#    EKF.cpp
#    camera.cpp
#    pose_camera.cpp
#    image.cpp
#)

#target_link_libraries( pose_camera
#    ${THIRD_PARTY_LIBS}
#)



###############################pose to image ######################

add_executable( pose_image
    EKF.cpp
    camera.cpp
    pose_image.cpp
    image.cpp
)

target_link_libraries( pose_image
    ${THIRD_PARTY_LIBS}
)

###############################pose to image ######################

add_executable( pose_image_TUM
    EKF.cpp
    camera.cpp
    pose_image_TUM.cpp
    image.cpp
)

target_link_libraries( pose_image_TUM
    ${THIRD_PARTY_LIBS}
)




