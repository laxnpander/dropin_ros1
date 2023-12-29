cmake_minimum_required(VERSION 3.9)

################################################################################
# Include CMake dependencies
################################################################################

################################################################################
# Compiler specific configuration
################################################################################

################################################################################
# Dependencies
################################################################################


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        cv_bridge
        geodesy
)

find_package(OpenCV REQUIRED)

################################################################################
# Catkin Package
################################################################################

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
            ${PROJECT_NAME}_ros
        CATKIN_DEPENDS
            roscpp
            roslib
            cv_bridge
            geodesy
        DEPENDS
            OpenCV
)


################################################################################
# Build
################################################################################

include_directories(
        include
        src
        ../..
)

add_library(${PROJECT_NAME}_ros SHARED
        src/ros_node.cpp
)

target_include_directories(${PROJECT_NAME}_ros PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_ros PUBLIC
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

################################################################################
# Executables
################################################################################

add_executable(${PROJECT_NAME}_ros_node src/ros_node_main.cpp)
add_dependencies(${PROJECT_NAME}_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros_node PUBLIC ${PROJECT_NAME}_ros)

################################################################################
# Install
################################################################################


# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}_ros/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
)
