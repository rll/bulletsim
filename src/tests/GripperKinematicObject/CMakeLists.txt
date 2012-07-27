# Build the files/ libraries for collaborative folding.

add_library(StepState step_state.h)
target_link_libraries(StepState simulation robots)

add_library(CollabUtils collab_folding_utils.cpp)
target_link_libraries(CollabUtils StepState simulation)


option (USE_RADIUS_CONTACT
        "Search around a radius of the gripper to find nodes to attach to."
        ON)
configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/GripperKinematicObject.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/GripperKinematicObject.h")
add_library(GripperKinematicObject GripperKinematicObject.cpp)
target_link_libraries(GripperKinematicObject CollabUtils simulation robots)