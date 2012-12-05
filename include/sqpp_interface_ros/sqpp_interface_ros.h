#ifndef _SQPP_INTERFACE_ROS_H_
#define _SQPP_INTERFACE_ROS_H_

#include "sqp/planning_problems2.h"
// #include <sqpp_motion_planner/sqpp_parameters.h>
// #include <planning_models/kinematic_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "simulation/simplescene.h"
#include "sqp/traj_sqp.h"
#include "sqp/planning_problems2.h"

namespace sqpp_interface_ros
{
/** @class SQPPInterfaceROS */
class SQPPInterfaceROS
{
public:
  SQPPInterfaceROS(const kinematic_model::KinematicModelConstPtr &kmodel);

  /* const sqpp::SqppParameters& getParams() const { */
  /*   return params_; */
  /* } */
  
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request &req, 
             moveit_msgs::GetMotionPlan::Response &res) const;

protected:
  
  /** @brief TODO: Configure everything using the param server */
  void loadParams(void);
  
  ros::NodeHandle nh_; /// The ROS node handle
  RaveInstance::Ptr rave;
  BulletInstance::Ptr bullet;
  OSGInstance::Ptr osg;
  Environment::Ptr env;
  kinematic_model::KinematicModelConstPtr kmodel;
  TrajOptimizer opt;
  // sqpp::SqppParameters params_;  
};

}

static inline void jointStateToArray(const kinematic_model::KinematicModelConstPtr& kmodel,
                              const sensor_msgs::JointState &joint_state, 
                              const std::string& planning_group_name, 
                              Eigen::VectorXd& joint_array)
{
// get the jointmodelgroup that we care about
  const kinematic_model::JointModelGroup* group = kmodel->getJointModelGroup(planning_group_name);
//vector of joints
  std::vector<const kinematic_model::JointModel*> models = group->getJointModels();
// Loop over joint_state to find the states that match the names
  for(unsigned int i=0; i < joint_state.position.size(); i++)
  {
    for(size_t j = 0; j < models.size(); j++)
    {
      if(models[j]->getName() == joint_state.name[i])
      {
        joint_array(j) = joint_state.position[i];
      }
    }
  }
}

#endif
