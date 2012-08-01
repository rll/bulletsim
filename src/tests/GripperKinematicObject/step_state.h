/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta | 27th July 2012. */


#ifndef _STEP_STATE_
#define _STEP_STATE_

//#include "simulation/simplescene.h"
#include "GripperKinematicObject.h"

/** A class (rather a structure) to represent the state of the
    simulation at any given time. */
class StepState {
 public:
  BulletInstance::Ptr bullet;
  OSGInstance::Ptr osg;
  Fork::Ptr fork;
  BulletSoftObject::Ptr cloth;
  GripperKinematicObject::Ptr left_gripper1;
  GripperKinematicObject::Ptr left_gripper2;
};

#endif
