include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_MSGS_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Wno-reorder")

set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
set(GUROBI_LIBRARY_DIR "${GUROBI_HOME}/lib")
include_directories(${GUROBI_INCLUDE_DIR})
link_directories(${GUROBI_LIBRARY_DIR})


include_directories(${PCL_INCLUDE_DIRS})

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_core
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  REQUIRED)

message("CATKIN LIBS ${catkin_LIBRARY_DIRS}")
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(test_ros_rave test_ros_rave.cpp)
target_link_libraries(test_ros_rave simulation robots sqp2 json moveit_collision_detection moveit_collision_detection_fcl)

set(MOVEIT_LIB_NAME moveit_sqpp_interface_ros)

add_library(${MOVEIT_LIB_NAME} sqpp_interface_ros.cpp rosconversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}
  simulation
  robots
  sqp2
  json
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# TODO: Build planning server
#add_executable(moveit_sqpp_planner src/sqpp_planner.cpp)
#target_link_libraries(moveit_sqpp_planner ${MOVEIT_LIB_NAME})

add_library(moveit_sqpp_planner_plugin SHARED sqpp_plugin.cpp)
target_link_libraries(moveit_sqpp_planner_plugin ${MOVEIT_LIB_NAME})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_sqpp_planner_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

