#ifndef _CUSTOM_KEY_HANDLER_
#define _CUSTOM_KEY_HANDLER_

#include "config.h"

#include "CustomScene.h"

#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <google/profiler.h>

#define PI 3.14159265

//WARNING: THIS IS THE WRONG TRANSFORM, WILL NOT WORK FOR ROTATION!
const btTransform TBullet_PR2GripperRight = 
  btTransform(  btQuaternion( btVector3(0,1,0), PI/2), btVector3(0,0,0))
  *btTransform( btQuaternion( btVector3(0,0,1), PI/2), btVector3(0,0,0))
  *btTransform( btQuaternion( btVector3(0,1,0), PI/2), btVector3(0,0,0));

const btTransform TBullet_PR2GripperLeft = 
  btTransform(  btQuaternion( btVector3(0,1,0), PI/2), btVector3(0,0,0))
  *btTransform( btQuaternion( btVector3(0,0,1),-PI/2), btVector3(0,0,0))
  *btTransform( btQuaternion( btVector3(0,1,0), PI/2), btVector3(0,0,0));

class CustomKeyHandler : public osgGA::GUIEventHandler {
  CustomScene &scene;
 public:
 CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

#endif
