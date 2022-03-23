cmake_minimum_required(VERSION 3.0.2)
project(rico_gmapping)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS 
    roscpp  # need this for everything
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    message_generation
    )


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rico_gmapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
# Specify locations for headers!
include_directories(${catkin_INCLUDE_DIRS} include/)
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
# Linking executable to packages in find_package
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Test node
add_executable(test_2d_scan_processor tests/test_2d_scan_processor.cpp)
# Linking executable to packages in find_package
target_link_libraries(test_2d_scan_processor
  ${catkin_LIBRARIES}
)

#  catkin_add_nosetests(test)
