cmake_minimum_required(VERSION 3.6)
project(galaxy_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    dynamic_reconfigure
    image_transport
    cv_bridge
    )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
)

add_executable(main "src/main.cpp" "src/galaxy_camera.cpp")
target_link_libraries(main gxiapi pthread ${OpenCV_LIBS} ${catkin_LIBRARIES})
