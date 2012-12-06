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
#include <geometric_shapes/shapes.h>
#include <Eigen/Geometry>
#include <iostream>

using namespace collision_detection;

int main(int argc, const char* argv[]){
  cout << "foo" << endl;
  Scene scene;
  scene.startViewer();

  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  CollisionWorldPtr cw(new collision_detection::CollisionWorldFCL());
  shapes::ShapeConstPtr cube(new shapes::Box(1,2,.5));
  Eigen::Vector3d axis(1,0,0);
  Eigen::Vector3d trans(0,1,0);
  Eigen::Affine3d t = Eigen::Translation3d(trans) * Eigen::AngleAxisd(0,axis);

  Eigen::Affine3d pose;
  cw->addToObject("cube1", cube, t);

  importCollisionWorld(scene.env, scene.rave, cw);
  cout << "Imported world" <<endl;
  scene.idle(true);

  cout << "done" << endl;

}
