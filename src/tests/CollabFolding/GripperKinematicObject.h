/**
   Author: Dmitry Berenson
   Maintained by: Ankush Gupta | 27th July 2012.
**/

#ifndef _GRIPPER_KINEMATIC_OBJECT_
#define _GRIPPER_KINEMATIC_OBJECT_

#include "config.h"
#include "collab_folding_utils.h"

#include "simulation/softbodies.h"

/** The various states the GripperKinematic object can be in. */
enum GripperState {GripperState_DONE, GripperState_CLOSING,
		   GripperState_OPENING};


/** Class to model grippers in Bullet.
    The gripper is modelled by two cuboidal jaws which hold the cloth
    in the middle.
    The cubiods are BoxObjects, which derives from bullet's Kinematic Object,
    making the gripper itself a kinematic object. */
class GripperKinematicObject : public CompoundObject<BoxObject> {
  
 public:
  // The distance b/w the jaws of the gripper, when open
  float apperture;
  
  // The transform of the mid-pt joining top and the bottom jaws' origin
  btTransform cur_tm;

  // TRUE iff the gripper is open.
  bool bOpen;

  // TRUE iff the softBody is attached to the gripper.
  bool bAttached;

  // The half-extents of the box used to model the jaws of the gripper.
  btVector3 halfextents;

  /* Holds the indices of the nodes of the softbody attached
     to the gripper. */
  std::vector<int> vattached_node_inds;

  // The gap b/w the jaws, when the gripper is closed.
  double closed_gap;

  // The state of the gripper. One of the GRIPPERSTATE (defined above).
  GripperState state;


  // TYPEDEF for a pointer to an object of this class.
  typedef boost::shared_ptr<GripperKinematicObject> Ptr;


  // Constructor of this class. Color is the color in which the
  // gripper is rendered.
  GripperKinematicObject(btVector4 color = btVector4(0,0,1,0.3));


  // Translates the gripper by TRANSVEC.
  void translate(btVector3 transvec);


  // APPLIES the transform TM to the middle point
  // of the two jaws' origins.
  void applyTransform(btTransform tm);


  // The function which SETS the transform of the
  // middle point of the two jaws' origin.
  void setWorldTransform(btTransform tm);


  // The callback required by Bullet from KinematicObjects.
  // Returns the transform of the middle point of the two jaws.
  btTransform getWorldTransform(){return cur_tm;}


  // The callback required by Bullet in every simulation cycle.
  // Returns CUR_TM.
  void getWorldTransform(btTransform& in){in = cur_tm;}


  // Opens/ closes the gripper, i.e. sets the appropriate transform.
  void toggle();


  /** Toggles whether the SoftBody PSB is attached to the gripper.

      Has 2 modes to attach:
      ----------------------
      1. Radius search : All nodes of PSB within a radius RADIUS
      are attach to the gripper using an anchor.
      
      2. Collision Search : All nodes of PSB colliding with the
      gripper are attached using an anchor.
      The points of collision b/w the gripper
      and the PSB are calculated and the nodes
      in PSB closest to those points are anchored.
      
      When it detaches, all anchors, except those pertaining to this gripper
      attached to the PSB remain unchanged.
      This gripper's anchors are removed from the list of anchors of PSB and
      the list of the nodes attached maintained by this gripper is cleared.*/
  void toggleattach(btSoftBody * psb, double radius = 0);


  // Fills in the RCONTACTS-ARRAY with contact information between PSB and PCO.
  void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco,
			    btSoftBody::tRContactArray &rcontacts);


  /** Adapted from btSoftBody.cpp (btSoftBody::appendAnchor).
      
      Appends an anchor to the soft-body, connecting
      node and the given rigidBody.
      
      PSB : The softbody to which the anchor is appended.
      NODE : The specific node on which the anchor is attached.
      BODY : The other end of anchor. The rigid body to which
      the anchor is attached.
      INFLUENCE : \in [0,1] : value of 1 means max attachment. */
  void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body,
		    btScalar influence=1);


  // Clears the list of the anchors attached to this gripper.
  void releaseAllAnchors(btSoftBody * psb) {psb->m_anchors.clear();}


  /* Returns a pointer to a copy this gripper.
     The data memeber pointers point to the same objects as this gripper. */
  EnvironmentObject::Ptr copy(Fork &f) const {
    Ptr obj(new GripperKinematicObject());
    internalCopy(obj, f);
    return obj;
  }


  /* Copies (soft) the members of this gripper to GRIPPER,
     and forks off the <BoxObject> jaws into the fork F. */
  void internalCopy(GripperKinematicObject::Ptr gripper, Fork &f) const {
    gripper->apperture = apperture;
    gripper->cur_tm = cur_tm;
    gripper->bOpen = bOpen;
    gripper->state = state;
    gripper->closed_gap = closed_gap;
    gripper->vattached_node_inds = vattached_node_inds;
    gripper->halfextents = halfextents;
    gripper->bAttached = bAttached;

    gripper->children.clear();
    gripper->children.reserve(children.size());
    ChildVector::const_iterator iter;
    for (iter = children.begin(); iter != children.end(); iter++) {
      if (*iter)
	gripper->children.push_back(boost::static_pointer_cast<BoxObject>
				    ((*iter)->copy(f)));
      else
	gripper->children.push_back(BoxObject::Ptr());
    }
  }


  /* Copies (soft) the members of this gripper to GRIPPER. */
  void copy(GripperKinematicObject::Ptr gripper) {
    gripper->apperture = apperture;
    gripper->cur_tm = cur_tm;
    gripper->bOpen = bOpen;
    gripper->state = state;
    gripper->closed_gap = closed_gap;
    gripper->vattached_node_inds = vattached_node_inds;
    gripper->halfextents = halfextents;
    gripper->bAttached = bAttached;

    gripper->children.clear();
    gripper->children.reserve(children.size());
    ChildVector::const_iterator iter;
    for (iter = children.begin(); iter != children.end(); iter++)
	gripper->children.push_back(BoxObject::Ptr());
  }


  /* Opens/closes the gripper, while interacting with the given
     softBody PSB.
     Successive calls to this function update the gripper's jaws.
     Each call moves the jaws by a small bit. */
  void step_openclose(btSoftBody * psb) {
    if (state == GripperState_DONE) return;
    
    if(state == GripperState_OPENING && bAttached)
      toggleattach(psb);
    
    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    // The step length to move the jaws by in each call to this function.
    const double STEP_SIZE = 0.005;

    if(state == GripperState_CLOSING) {
      top_tm.setOrigin(top_tm.getOrigin()
		       + STEP_SIZE * top_tm.getBasis().getColumn(2));
      bottom_tm.setOrigin(bottom_tm.getOrigin()
			  - STEP_SIZE * bottom_tm.getBasis().getColumn(2));
    } else if(state == GripperState_OPENING) {
      top_tm.setOrigin(top_tm.getOrigin()
		       - STEP_SIZE * top_tm.getBasis().getColumn(2));
      bottom_tm.setOrigin(bottom_tm.getOrigin()
			  + STEP_SIZE * bottom_tm.getBasis().getColumn(2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    double current_gap_length =
      (top_tm.getOrigin() - bottom_tm.getOrigin()).length();

    if(state == GripperState_CLOSING
       && current_gap_length <= (closed_gap + 2*halfextents[2])) {
      state = GripperState_DONE;
      bOpen = false;
      if(!bAttached)
	toggleattach(psb);
    }

    if(state == GripperState_OPENING && current_gap_length >= apperture) {
      state = GripperState_DONE;
      bOpen = true;
    }
  }

};
#endif
