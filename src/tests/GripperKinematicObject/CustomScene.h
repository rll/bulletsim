/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta. 30th July 2012. */


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

  struct {
    bool transGrabber0,rotateGrabber0,
      transGrabber1,rotateGrabber1,
      transGrabber2,rotateGrabber2,
      transGrabber3,rotateGrabber3;
    bool startDragging;
    float dx, dy, lastX, lastY;
  } inputState;
  
  int num_auto_grippers;
  float jacobian_sim_time;
  bool bTracking, bInTrackingLoop;

  PointReflector::Ptr point_reflector;

  BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;
  BulletInstance::Ptr bullet2;

  OSGInstance::Ptr osg2;
  Fork::Ptr fork;

  RaveRobotObject::Ptr origRobot, tmpRobot;

  std::map<int, int> node_mirror_map;
  std::vector<std::vector<double> > gripper_node_distance_map;
  std::vector<btVector3> prev_node_pos;

  PlotPoints::Ptr plot_points;
  PlotPoints::Ptr left_center_point;
  PlotAxes::Ptr left_axes1,left_axes2;
  PlotLines::Ptr rot_lines;
  Eigen::MatrixXf cloth_distance_matrix;
  int user_mid_point_ind, robot_mid_point_ind;


  /** Constructor. */
#ifdef USE_PR2
 CustomScene() : pr2m(*this){
#else
    CustomScene() {
#endif
      bTracking = bInTrackingLoop = false;
      
      inputState.transGrabber0 =  inputState.rotateGrabber0 =
	inputState.transGrabber1 =  inputState.rotateGrabber1 =
	inputState.transGrabber2 =  inputState.rotateGrabber2 =
	inputState.transGrabber3 =  inputState.rotateGrabber3 =
	inputState.startDragging = false;

      jacobian_sim_time = 0.05;


      left_gripper1_orig.reset(new GripperKinematicObject());
      left_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
							btVector3(0,-10,0)));
      env->add(left_gripper1_orig);


      btVector4 color(0.6,0.6,0.6,1);
      right_gripper1_orig.reset(new GripperKinematicObject(color));
      right_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
							 btVector3(0,10,0)));
      env->add(right_gripper1_orig);
 

      left_gripper1 = left_gripper1_orig;
      right_gripper1 = right_gripper1_orig;
      

      left_gripper2.reset(new GripperKinematicObject());
      left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						   btVector3(0,20,0)));
      env->add(left_gripper2);


      right_gripper2.reset(new GripperKinematicObject(color));
      right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						    btVector3(0,-20,0)));
      env->add(right_gripper2);

      num_auto_grippers = 2;      
      fork.reset();
    }

    /* Creates a square cloth with side length 2s.
       The four coordinates of the cloth are:
             {(s,s) (-s,s,) (-s,-s) (s,-s)}
       Then, the center of the cloth (initially at (0,0,0)
       is translated to CENTER.*/
    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);


    void createFork();
    void destroyFork();
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

    /** Main loop which is responsible for jacobian tracking.*/
    void doJTracking();

    /** */
    void drawAxes();
    void regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr  gripper_to_detach);

    void run();
};

