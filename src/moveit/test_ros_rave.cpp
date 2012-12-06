#include <sqpp_interface_ros/sqpp_interface_ros.h>
#include "simulation/util.h"
#include "sqp/traj_sqp.h"
#include "sqp/planning_problems2.h"
#include <moveit/kinematic_state/conversions.h>
#include <Eigen/Dense>
#include <sqpp_interface_ros/rosconversions.h>

#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/robot_manager.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/clock.h"
#include <osg/Depth>
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "sqp/kinematics_utils.h"
#include "sqp/plotters.h"
#include "sqp/traj_sqp.h"
#include "sqp/planning_problems2.h"

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world.h>

#include <iostream>

using namespace collision_detection;

int main(int argc, const char* argv[]){
  cout << "foo" << endl;
  Scene scene;
  scene.startViewer();
  scene.idle(true);

  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  CollisionWorldConstPtr cw(new collision_detection::CollisionWorldFCL());
  CollisionWorld::ObjectConstPtr cube(new CollisionWorld::Object("Cube"));
  CollisionWorld::Change addCube;
  cout << "done" << endl;

}
