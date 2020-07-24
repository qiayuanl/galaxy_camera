cmake_minimum_required(VERSION 3.6)
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
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
)

add_executable(galaxy_camera
    "src/galaxy_camera.cpp"
    )
target_link_libraries(galaxy_camera
    gxiapi ${catkin_LIBRARIES})
