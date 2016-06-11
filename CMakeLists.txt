cmake_minimum_required(VERSION 2.8.3)
project(sensor_stick_driver)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ahrs
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)
include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)
add_executable(sensor_stick_driver_node src/gnublin.cpp src/main.cpp)

target_link_libraries(sensor_stick_driver_node
        ${catkin_LIBRARIES}
        )
