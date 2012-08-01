# Build the files/ libraries for collaborative folding.

add_library(StepState step_state.h)
target_link_libraries(StepState simulation robots)

add_library(CollabUtils collab_folding_utils.cpp)
target_link_libraries(CollabUtils StepState simulation)

option (USE_RADIUS_CONTACT
        "Search around a radius of the gripper to find nodes to attach to."
        ON)
option (USE_TABLE "Use a table in the simulation." OFF)
option (DO_ROTATION "Calculate for roation of the gripers." ON)
option (PROFILER "Profile the code." OFF)
option (USE_PR2 "Use R2 in simulation." ON)

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/test_collab_folding.cpp.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_collab_folding.cpp")

add_library(GripperKinematicObject GripperKinematicObject.cpp)
target_link_libraries(GripperKinematicObject CollabUtils simulation robots)

add_library(CustomScene CustomScene.cpp)
target_link_libraries(CustomScene StepState
                                  GripperKinematicObject
				  CollabUtils
				  simulation
				  profiler robots)

add_library(CustomKeyHandler CustomKeyHandler.cpp)
target_link_libraries(CustomKeyHandler CustomScene simulation profiler robots)

add_executable(test_folding test_collab_folding.cpp)
target_link_libraries(test_folding CustomScene CustomKeyHandler simulation profiler robots)
