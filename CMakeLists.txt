cmake_minimum_required(VERSION 3.0.2)
project(social_octomap_ros)

find_package(catkin REQUIRED COMPONENTS social_octomap_msgs sensor_msgs tf)
find_package(SOCIAL_OCTOMAP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS social_octomap_msgs sensor_msgs tf
  DEPENDS SOCIAL_OCTOMAP)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/conversions.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SOCIAL_OCTOMAP_LIBRARIES})
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${SOCIAL_OCTOMAP_INCLUDE_DIRS}
  PUBLIC include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
