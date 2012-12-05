#include <btBulletCollisionCommon.h>
#include "simulation/simulation_fwd.h"
#include <moveit/collision_detection/collision_world.h>
#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "simulation/simplescene.h"
#include "sqp/traj_sqp.h"
#include "sqp/collisions.h"
#include "sqp/planning_problems2.h"

OpenRAVE::KinBodyPtr moveitObjectToKinBody(collision_detection::CollisionWorld::ObjectConstPtr object, OpenRAVE::EnvironmentBasePtr env);

void importCollisionWorld(Environment::Ptr env, RaveInstance::Ptr rave, const collision_detection::CollisionWorldConstPtr world);

bool setRaveRobotState(OpenRAVE::RobotBasePtr robot, sensor_msgs::JointState js);
