cmake_minimum_required(VERSION 2.8.3)
project(slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    message_filters
    pcl_ros
    pcl_conversions
    std_msgs
    message_generation


 )

find_package(OpenCV REQUIRED)



# message
add_message_files(
    FILES PnP_result.msg
)

generate_messages(
    DEPENDENCIES std_msgs
)




catkin_package(CATKIN_DEPENDS 
    roscpp
    cv_bridge
    sensor_msgs
    message_filters
    pcl_ros
    pcl_conversions
    std_msgs
    message_generation




    INCLUDE_DIRS include



    LIBRARIES slambase

)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)



add_executable(generatePointCloud 
    src/generatePointCloud.cpp
)

target_link_libraries(generatePointCloud
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


# detectFeatures
add_executable(detectFeatures 
    src/detectFeatures.cpp
)



target_link_libraries(detectFeatures
    slambase
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)



# jointPointCloud
add_executable(jointPointCloud 
    src/jointPointCloud.cpp
)



target_link_libraries(jointPointCloud
    slambase
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)



# visualOdometry
add_executable(visualOdometry 
    src/visualOdometry.cpp
)



target_link_libraries(visualOdometry
    slambase
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)





# Add g2o dependence
# Because g2o instead of the usual library file you want to add it findg2o.cmake
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( G2O_ROOT /usr/local/include/g2o )
find_package(G2O REQUIRED)

# CSparse
find_package(CSparse REQUIRED)

include_directories(
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIRS}
)


# slamEnd
add_executable(slamEnd 
    src/slamEnd.cpp
)



target_link_libraries(slamEnd
    slambase
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY}
)




# slam
add_executable(slam 
    src/slam.cpp
)



target_link_libraries(slam
    slambase
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY}
)




# library
add_library(slambase src/slamBase.cpp)

target_link_libraries(slambase 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)





