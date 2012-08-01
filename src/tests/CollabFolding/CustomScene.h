/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta. 30th July 2012. */


#ifndef _CUSTOM_SCENE_
#define _CUSTOM_SCENE_

#include "config.h"
#include "collab_folding_utils.h"
#include "GripperKinematicObject.h"
#include "step_state.h"
#include "PR2SoftBodyGripperAction.h"

#include "simulation/simplescene.h"
#include <omp.h>

#ifdef USE_PR2
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#endif

class CustomScene : public Scene {
public:  
#ifdef USE_PR2
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
    PR2Manager pr2m;
#endif
     

  GripperKinematicObject::Ptr left_gripper1, right_gripper1,
    left_gripper1_orig, right_gripper1_orig,
    left_gripper1_fork, right_gripper1_fork;

  GripperKinematicObject::Ptr left_gripper2, right_gripper2;


  /** This structure maintains various state variables
     which are changed during user interaction. 

    - transGrabberX, rotateGrabberX: booleans which select whether to
                                    translate/ rotate grabber number X.

    - startDragging: is true when the mouse button is pressed and
                    mouse is being moved by the user.

    - dx,dy : change in mouse position.

    - lastX,lastY : used to store last mouse position. */
  struct {
    bool transGrabber0,rotateGrabber0,
      transGrabber1,rotateGrabber1,
      transGrabber2,rotateGrabber2,
      transGrabber3,rotateGrabber3;
    bool startDragging;
    float dx, dy, lastX, lastY;
  } inputState;

  /** Number of automatic grippers in the scene. */
  int num_auto_grippers;

  /** The time for which to simulate physics to calculate
      dx (change in position of the cloth) for computing the Jacobian. */
  float jacobian_sim_time;

  
  bool bTracking, bInTrackingLoop;

  /** Reflects a node about the center line on the softbody (cloth). */
  PointReflector::Ptr point_reflector;

  /** Bullet softbodies. */
  BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;

  /** Bullet simulation environemt instance. */
  BulletInstance::Ptr bullet2;

  /** OpenSceneGraph instance for rendering scenes. */
  OSGInstance::Ptr osg2;

  /** Fork: for simulating physics in a new thread. */
  Fork::Ptr fork;

  /** OpenRave instance for the simulation. */
  RaveRobotObject::Ptr origRobot, tmpRobot;

  /** Stores the correspondance b/w a node index and the index of
      the node that it is supposed to mirror. */
  std::map<int, int> node_mirror_map;

  /** For every automatic gripper, it stores the distance to all
      nodes in the softbody from the corner point corresponding
      to where the automatic gripper is holding the cloth. */
  std::vector<std::vector<double> > gripper_node_distance_map;

  /** Stores the positions of the nodes of the softbody. */
  std::vector<btVector3> prev_node_pos;

  /** Points which are to be plotted in the scene : correspond to
      nodes in the softbody (cloth). */
  PlotPoints::Ptr plot_points;

  
  PlotPoints::Ptr left_center_point;

  /** Axes corresponding to the location where the
      left grippers are. */
  PlotAxes::Ptr left_axes1,left_axes2;
  PlotLines::Ptr rot_lines;

  /** It is a symmetric matrix which store the euclidean distance
      b/w the nodes of the softybody (cloth). Its dimensions are NxN,
      where N = number of cloth nodes. */
  Eigen::MatrixXf cloth_distance_matrix;

  /** Indices of the nodes in the softbody, which are closest
      to the mid-point of the human and robot grippers respectively. */
  int user_mid_point_ind, robot_mid_point_ind;


  /** Constructor. */
  CustomScene();
  
  /* Creates a square cloth with side length 2s.
     The four coordinates of the cloth are:
     {(s,s) (-s,s,) (-s,-s) (s,-s)}
     Then, the center of the cloth (initially at (0,0,0)
     is translated to CENTER.*/
  BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);


  void createFork();
  void destroyFork();

  /** Swaps the forked objects with the ones in the main loop. */
  void swapFork();


  /** Simulates the cloth in bullet for JACOBIAN_SIM_TIME
      and takes the difference in the positions of the node and
      divides by JACOBIAN_SIM_TIME.*/
  Eigen::MatrixXf computeJacobian();

    
  /** Very similar to computeJacobian except is parallelized
      (using openMP).*/
  Eigen::MatrixXf computeJacobian_parallel();

    
  /** Computes an approximation to the Jacobian of the cloth's
      node positions wrt to the robot grippers, by using an 
      scaling factor which decays exponentially with increasing
      distance from the gripper. */
  Eigen::MatrixXf computeJacobian_approx();


  /** Finds the distance from a node corresponding to 
      INPUT_IND on the cloth to the closest node attached to the gripper.*/
  double getDistfromNodeToClosestAttachedNodeInGripper
    (GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind);


  /** Simulates in a new fork.*/
  void simulateInNewFork(StepState& innerstate, float sim_time,
			 btTransform& left_gripper1_tm,
			 btTransform& left_gripper2_tm);

  /** Main loop which is responsible for jacobian tracking. */
  void doJTracking();

  /** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
  void drawAxes();

  /** Attaches GRIPPER_TO_ATTACH with the softbody and detaches
      GRIPPER_TO_DETACH from it.*/
  void regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach,
			     GripperKinematicObject::Ptr  gripper_to_detach);

  /* Sets up the scene and UI event handlers,
     initializes various structures.*/
  void run();
};
#endif
