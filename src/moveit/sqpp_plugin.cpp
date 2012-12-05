#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <sqpp_interface_ros/sqpp_interface_ros.h>
#include <sqpp_interface_ros/rosconversions.h>

#include <boost/shared_ptr.hpp>

#include <pluginlib/class_list_macros.h>

namespace sqpp_interface_ros
{

class SQPPlanner : public planning_interface::Planner
{
public:
  void init(const kinematic_model::KinematicModelConstPtr& model)
  {
    sqpp_interface_.reset(new SQPPInterfaceROS(model));
  }

  bool canServiceRequest(const moveit_msgs::GetMotionPlan::Request &req) const
  {
    // TODO: Actually respond with something reasonable
    //      capabilities.dummy = false;
    return true;
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request &req, 
             moveit_msgs::GetMotionPlan::Response &res) const
  {
    return sqpp_interface_->solve(planning_scene, req, res);
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request &req,
             moveit_msgs::MotionPlanDetailedResponse &res) const
  {
    moveit_msgs::GetMotionPlan::Response res2;
    if (sqpp_interface_->solve(planning_scene, req,res2))
    {
      res.trajectory_start = res2.trajectory_start;
      res.trajectory.push_back(res2.trajectory);
      res.description.push_back("plan");
      res.processing_time.push_back(res2.planning_time);
      return true;
    }
    else
      return false;
  }

  std::string getDescription(void) const { return "SQPP"; }
  
  void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.resize(1);
    algs[0] = "SQPP";
  }

  void terminate(void) const
  {
    //TODO - make interruptible
  }
     
private:
  boost::shared_ptr<SQPPInterfaceROS> sqpp_interface_;
};

} // sqpp_interface_ros

PLUGINLIB_DECLARE_CLASS(sqpp_interface_ros, SQPPlanner,
                        sqpp_interface_ros::SQPPlanner, 
                        planning_interface::Planner);
