#include <sqpp_interface_ros/sqpp_interface_ros.h>
#include "simulation/util.h"
#include "sqp/traj_sqp.h"
#include "sqp/planning_problems2.h"
#include <moveit/kinematic_state/conversions.h>
#include <Eigen/Dense>
#include <sqpp_interface_ros/rosconversions.h>
#include "utils/logging.h"
#include "sqp/config_sqp.h"

namespace sqpp_interface_ros
{

SQPPInterfaceROS::SQPPInterfaceROS(const kinematic_model::KinematicModelConstPtr& kmodel) :
  kmodel(kmodel), nh_("~") 
{
  LoggingInit();
  // Perhaps this should be done in Solve, IDKLOL
  bullet.reset(new BulletInstance());
  osg.reset(new OSGInstance()); // Maybe don't need
  env.reset(new Environment(bullet, osg));
  env->bullet = bullet;
  rave.reset(new RaveInstance());
  loadParams();
}

void SQPPInterfaceROS::loadParams(void) {
  
}

bool SQPPInterfaceROS::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::GetMotionPlan::Request &req, 
                         moveit_msgs::GetMotionPlan::Response &res) const
{

    BulletConfig::linkPadding = 0;
    BulletConfig::margin = 0;
    GeneralConfig::verbose=20000;
    GeneralConfig::scale = 10.;
    SQPConfig::distDiscSafe = .01;
    SQPConfig::distContSafe = 0;
    SQPConfig::distPen = .02;
    SQPConfig::shapeExpansion = .04;

  ros::WallTime start_time = ros::WallTime::now();

  initializeGRB();
  Eigen::MatrixXd initTraj;

  // Initialize a scene
  // Scene scene;
  
  //TODO: I hate this but it's necessary for RaveObject/BulletObject to work
  util::setGlobalEnv(env);
  //util::setGlobalScene(&scene);
  TrajOptimizer opt;

  rave->env->SetDebugLevel(1);
  //OpenRAVE::RobotBasePtr robot = OpenRAVE::RaveCreateRobot(rave->env, kmodel->getName());
  OpenRAVE::RobotBasePtr robot = rave->env->ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml");
  rave->env->AddRobot(robot);
  LOG_INFO("Loaded robot XML");
  RaveRobotObject::Ptr rro(new RaveRobotObject(rave, robot, CONVEX_HULL, BulletConfig::kinematicPolicy <= 1));
  
  LOG_INFO("Created a robot??");
  sensor_msgs::JointState js;
  
  // Gathers the goal joint constraints into a JointState object
  // TODO: Handle all constraints, not just first joint constraints
  // TODO: Refactor, this is ugly
  for(unsigned int i = 0; i < req.motion_plan_request.goal_constraints[0].joint_constraints.size(); i++) {
    js.name.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name);
    js.position.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].position);
  }

  LOG_INFO("Gathered goal joint state");
  const kinematic_model::JointModelGroup* model_group = 
    planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

  // May not need this; need to set OpenRAVE robot to initial joint state

  int numJoints = model_group->getJointModels().size();
  Eigen::MatrixXd tempm(2, numJoints);
  Eigen::VectorXd initialState(numJoints);
  Eigen::VectorXd goalState(numJoints);
  LOG_INFO_FMT("Planning for %d joints", numJoints);
  //Eigen::MatrixXd trajectory(numJoints,100);
  jointStateToArray(planning_scene->getKinematicModel(),
                    req.motion_plan_request.start_state.joint_state, 
                    req.motion_plan_request.group_name,
                    initialState);
  LOG_INFO("Got initial joint states as array");
  // Note: May need to check req.mpr.start_state.multi_dof_joint_state for base transform and others
  // TODO: This function is broken
  setRaveRobotState(robot, req.motion_plan_request.start_state.joint_state);
  LOG_INFO("Set RAVE Robot State");
  // Get the goal state
  jointStateToArray(planning_scene->getKinematicModel(),
                    js, 
                    req.motion_plan_request.group_name, 
                    goalState);
  LOG_INFO ("Got Goal state");
  // optimize!
  kinematic_state::KinematicState start_state(planning_scene->getCurrentState());
  kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
    
  ros::WallTime create_time = ros::WallTime::now();
  LOG_INFO("Gathered start and goal states");
  // ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  // ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  // optimizer.optimize();
  // ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());

  // Create OpenRAVE world
  // Got a rave and a bullet instance
  // Copy planning scene obstacles into OpenRAVE world
  importCollisionWorld(env, rave, planning_scene->getCollisionWorld());
  LOG_INFO("Imported collision world");

  setupBulletForSQP(env->bullet->dynamicsWorld);

  // Create Robot Object

  // We want something like:
  

  OpenRAVE::RobotBase::ManipulatorPtr manip = getManipulatorFromGroup(robot, model_group);
  setupArmToJointTarget(opt, goalState, rro->createManipulator(manip->GetName(), false));

  trajOuterOpt(opt, AllowedCollisions());

  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_INFO("Output trajectory has %d joints", numJoints);
  Eigen::MatrixXd finalTraj = opt.m_traj;
  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(numJoints);
  for (size_t i = 0; i < model_group->getJointModels().size(); i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = model_group->getJointModels()[i]->getName();
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(finalTraj.rows());
  for (int i=0; i < finalTraj.rows(); i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(numJoints);
    for (size_t j=0; j < res.trajectory.joint_trajectory.points[i].positions.size(); j++)
    {
      res.trajectory.joint_trajectory.points[i].positions[j] = finalTraj(i,j);
      if(i == finalTraj.rows()-1) {
        ROS_INFO_STREAM("Joint " << j << " " << res.trajectory.joint_trajectory.points[i].positions[j]);
      }
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  
  ROS_INFO("Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[finalTraj.rows()].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time = ros::Duration((ros::WallTime::now() - start_time).toSec());
  return true;
}

}
