cmake_minimum_required(VERSION 3.5)
project(galaxy_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        message_generation
        dynamic_reconfigure
        image_transport
        camera_info_manager
        )

generate_dynamic_reconfigure_options(
        cfg/camera.cfg
)


catkin_package(
        LIBRARIES galaxy_camera
        CATKIN_DEPENDS nodelet roscpp dynamic_reconfigure
        image_transport camera_info_manager
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include/
        ./
)


add_library(galaxy_camera
        src/galaxy_camera.cpp
        )

target_link_libraries(galaxy_camera
        gxiapi ${catkin_LIBRARIES})

add_executable(galaxy_camera_node
        src/galaxy_camera_node.cpp)
add_dependencies(galaxy_camera_node ${PROJECT_NAME}_gencfg)
target_link_libraries(galaxy_camera_node
        gxiapi ${catkin_LIBRARIES})
