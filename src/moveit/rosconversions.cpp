#include <sqpp_interface_ros/rosconversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include <assert.h>

using namespace std;
OpenRAVE::KinBodyPtr moveitObjectToKinBody(collision_detection::CollisionWorld::ObjectConstPtr object, OpenRAVE::EnvironmentBasePtr env){
  vector<shapes::ShapeConstPtr> shapes = object->shapes_;
  vector<Eigen::Affine3d> poses; // Affine3D is a 4x4 affine transform matrix
  list<OpenRAVE::KinBody::Link::GeometryInfo> geometries;
  for(int i = 0; i < shapes.size(); i++){
	shapes::ShapeConstPtr shape = shapes[i];
    OpenRAVE::KinBody::Link::GeometryInfo info;
    // Convert the Eigen::Affine3D pose into a RaveTransform (translation + quaternion)
    Eigen::Vector3d translation = poses[i].translation();
    Eigen::Matrix3d rot = poses[i].rotation();
    info._t.trans = RaveVector<double>(translation[0], translation[1], translation[2]);
    OpenRAVE::geometry::RaveTransformMatrix<double> rotm;
    rotm.rotfrommat(rot(0,0), rot(0,1), rot(0,2),
		    rot(1,0), rot(1,1), rot(1,2),
		    rot(2,0), rot(2,1), rot(2,2));
    info._t.rot = OpenRAVE::geometry::quatFromMatrix(rotm);

    // Fill in the geometry data (_vGeomData)
    switch(shape->type){
    case shapes::SPHERE: {//for sphere it is radius
      info._type = OpenRAVE::KinBody::Link::GeomSphere;
	  boost::shared_ptr<const shapes::Sphere> sph = boost::dynamic_pointer_cast<const shapes::Sphere>(shape);
      info._vGeomData.x = sph->radius;
      break;
	}
    case shapes::CYLINDER: {//for cylinder, first 2 values are radius and height
      boost::shared_ptr<const shapes::Cylinder> cyl = boost::dynamic_pointer_cast<const shapes::Cylinder>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomCylinder;
      info._vGeomData.x = cyl->radius;
      info._vGeomData.y = cyl->length;
      break;
	}
    case shapes::BOX: { //for boxes, first 3 values are extents
      boost::shared_ptr<const shapes::Box> box = boost::dynamic_pointer_cast<const shapes::Box>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomBox;
      info._vGeomData.x = box->size[0];
      info._vGeomData.y = box->size[1];
      info._vGeomData.z = box->size[2];
      break;
	}
    case shapes::MESH: {
      // Not actually sure if mesh and trimesh are the same thing.
      // Hopefully don't have to triangulate non-triangular meshes
      boost::shared_ptr<const shapes::Sphere> sph = boost::dynamic_pointer_cast<const shapes::Sphere>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomTrimesh;
      continue; // TODO: Implement this and remove
      break;
	}
    default:
      //LOG_WARN("Unknown shape type");
      continue;
    }
    geometries.push_back(info);
  }
  //TODO: Set a name for the KinBody
  OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env);
  body->InitFromGeometries(geometries);
  // Perhaps it should do something with the return code, if it's false?
  return body;
}

void importCollisionWorld(Environment::Ptr env, RaveInstance::Ptr rave, const collision_detection::CollisionWorldConstPtr world){
  std::vector<string> objectIds = world->getObjectIds();
  for(int i = 0; i < objectIds.size(); i++){
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(objectIds[i]);
	OpenRAVE::KinBodyPtr body = moveitObjectToKinBody(obj, rave->env);
	// Not actually sure if the below is necessary, or what RaveCreateBody(env) actually does
    env->add(RaveObject::Ptr(new RaveObject(rave, body, CONVEX_HULL, BulletConfig::kinematicPolicy == 0)));

  }
}

bool setRaveRobotState(OpenRAVE::RobotBasePtr robot, sensor_msgs::JointState js){
  vector<string>::iterator nameit = js.name.begin();
  vector<double>::iterator posit = js.position.begin();
  vector<int> dofs;
  bool foundAllJoints = true;
  while(nameit != js.name.end()){
	// Not sure if different types of joints need to be handled separately
	OpenRAVE::KinBody::JointPtr joint = robot->GetJoint(*nameit);
	if(joint){
	  dofs.push_back(joint->GetDOFIndex());
	}else{
	  foundAllJoints = false;
	}
  }
  assert(js.position.size() == dofs.size());
  robot->SetDOFValues(js.position, OpenRAVE::KinBody::CLA_CheckLimits, dofs);
  return foundAllJoints;
}
