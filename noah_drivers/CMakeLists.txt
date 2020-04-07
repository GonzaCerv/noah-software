cmake_minimum_required(VERSION 2.8.3)
project(noah_drivers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14  -Werror -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  noah_msgs
  roscpp
  rospy
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    geometry_msgs
    roscpp 
    rospy 
    std_msgs
    noah_msgs
  LIBRARIES
    ${PROJECT_NAME}_uart_comms_node 
    ${PROJECT_NAME}_uart
  DEPENDS 
)


###########
## Build ##
###########

## C++ executables ##
add_executable(${PROJECT_NAME}_uart_comms_node 
  src/PackageManager.cpp
  src/PCBCommsNode.cpp
  src/PCBCommsNodeRunner.cpp)

target_link_libraries(${PROJECT_NAME}_uart_comms_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_uart
)
add_dependencies(${PROJECT_NAME}_uart_comms_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## C++ libraries ##

add_library(${PROJECT_NAME}_uart
  src/Uart.cpp
)
target_link_libraries(${PROJECT_NAME}_uart
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_uart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  ###### Enable Gmock ##########
  set(gmock_SOURCE_DIR /usr/src/gmock)
  set(gtest_SOURCE_DIR /usr/src/gtest)
  set(gtest_INCLUDE_DIR /usr/include/gtest)
  include_directories(
    ${gmock_SOURCE_DIR}/include
    ${gmock_SOURCE_DIR}
    ${gmock_SOURCE_DIR}/gtest/include
    ${gmock_SOURCE_DIR}/gtest
    ${gtest_INCLUDE_DIR}
    ${gtest_SOURCE_DIR}
    ${gtest_SOURCE_DIR}/src
  )

  # GMock + Catkin need help: https://answers.ros.org/question/199680/catkin-doesnt-play-nice-with-google-mock/
  # TODO(build_time): Only build gmock if not already built
  add_library(${PROJECT_NAME}_gmock
    ${gmock_SOURCE_DIR}/src/gmock-all.cc
    ${gtest_SOURCE_DIR}/src/gtest-all.cc
  )
  target_link_libraries(${PROJECT_NAME}_gmock pthread)
  install(
    TARGETS
      ${PROJECT_NAME}_gmock
    LIBRARY DESTINATION
      ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
      ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  # Replace catkin variable
  set(GTEST_LIBRARIES ${PROJECT_NAME}_gmock)
  ###### END Enable Gmock ##########


  add_rostest_gtest(uart_tests test/uart_tests.test test/uart_tests.cpp)
  target_link_libraries(uart_tests
    ${catkin_LIBRARIES} 
    ${PROJECT_NAME}_uart 
    ${GTEST_LIBRARIES}) 

  add_rostest_gtest(package_manager_tests test/package_manager_tests.test 
    test/package_manager_tests.cpp
    src/PackageManager.cpp)
  target_link_libraries(package_manager_tests 
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME}_uart ) 

endif()

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS 
    ${PROJECT_NAME}_uart
    ${PROJECT_NAME}_uart_comms_node 
    
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )