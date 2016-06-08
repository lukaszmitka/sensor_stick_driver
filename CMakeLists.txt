cmake_minimum_required(VERSION 2.8.3)
project(ahrs)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ahrs_algorithm
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)
include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)
add_executable(ahrs_node src/gnublin.cpp src/main.cpp)

target_link_libraries(ahrs_node
        ${catkin_LIBRARIES}
        )
