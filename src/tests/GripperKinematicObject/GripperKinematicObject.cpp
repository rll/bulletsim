/**
   Author: Dmitry Berenson
   Maintained by: Ankush Gupta | 27th July 2012.
 **/

#include "GripperKinematicObject.h"

/** Applies the transform TM to the mid-pt of the TOP-JAW and
    the BOTTOM-JAW.*/
void GripperKinematicObject::applyTransform(btTransform tm) {
  setWorldTransform(getWorldTransform()*tm);
}


/** Translates the Gripper.*/
void GripperKinematicObject::translate(btVector3 transvec) {
  btTransform tm = getWorldTransform();
  tm.setOrigin(tm.getOrigin() + transvec);
  setWorldTransform(tm);
}


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
   This gripper's anchors are removed from the list of anchors of PSB
   and the list of the nodes attached maintained by this gripper is cleared. */
void GripperKinematicObject::toggleattach(btSoftBody * psb, double radius) {

    if(bAttached) {
      btAlignedObjectArray<btSoftBody::Anchor> newanchors;
      for(int i = 0; i < psb->m_anchors.size(); i += 1) {
	if(psb->m_anchors[i].m_body != children[0]->rigidBody.get()
	   && psb->m_anchors[i].m_body != children[1]->rigidBody.get())
	  newanchors.push_back(psb->m_anchors[i]);
      }
      releaseAllAnchors(psb);

      // Restore all other anchors which do not correspond to this gripper.
      for(int i = 0; i < newanchors.size(); i += 1)
	psb->m_anchors.push_back(newanchors[i]);

      vattached_node_inds.clear();
    } else {
      #ifdef USE_RADIUS_CONTACT

      if(radius == 0)
	radius = halfextents[0];

      btTransform top_tm;
      children[0]->motionState->getWorldTransform(top_tm);

      btTransform bottom_tm;
      children[1]->motionState->getWorldTransform(bottom_tm);

      int closest_body = -1;
      unsigned int num_anchors = 0;
      for(int j = 0; j < psb->m_nodes.size(); j += 1) {

	if((psb->m_nodes[j].m_x - cur_tm.getOrigin()).length() < radius) {
	  num_anchors += 1;
	  if( (psb->m_nodes[j].m_x - top_tm.getOrigin()).length() <
	      (psb->m_nodes[j].m_x - bottom_tm.getOrigin()).length() )
	    closest_body = 0;
	  else
	    closest_body = 1;

	  vattached_node_inds.push_back(j);
	  appendAnchor(psb, &psb->m_nodes[j],
		       children[closest_body]->rigidBody.get());
	}
      }
      cout << "Attached "<< num_anchors<<" anchors to a gripper."<<endl;

     #else
      std::vector<btVector3> nodeposvec;
      nodeArrayToNodePosVector(psb->m_nodes, nodeposvec);

      // do for both the JAWS: top==children[0] | bottom==children[1]
      for(int k = 0; k < 2; k += 1) {

	BoxObject::Ptr part = children[k];
	btRigidBody* rigidBody = part->rigidBody.get();
	btSoftBody::tRContactArray rcontacts;
	getContactPointsWith(psb, rigidBody, rcontacts);
	cout << "Got " << rcontacts.size() << " contacts\n";

	/** If no contacts, then done with this gripper.
	    In this case no anchors are attached to the jaw in question. */
	if(rcontacts.size() == 0)
	  continue;

	/** In case the softBody collides with the gripper,
	    then get the node on the softbody closest to a given
	    collision point and attach an anchor to it.

	    In theory the RCONTATCS.M_NODE is all we want,
	    but we seek the node's index in the nodeposevec,
	    so are doing a linear search and allowing for a
	    slack of \epsilon = 0.0001.	*/
	float epsilon = 0.0001;
	for (int i = 0; i < rcontacts.size(); i += 1) {
	  btSoftBody::Node *node = rcontacts[i].m_node;
	  const btVector3 &contactPt = node->m_x;
	  int closest_ind = -1;
	  for(int n = 0; n < nodeposvec.size(); n += 1) {
	    if((contactPt - nodeposvec[n]).length() < epsilon) {
	      closest_ind = n;
	      break;
	    }
	  }
	  assert(closest_ind!=-1);

	  vattached_node_inds.push_back(closest_ind);
	  appendAnchor(psb, node, rigidBody);
	  cout << "\tAppending anchor to node indexed: "<< closest_ind << "\n";
	}
      }
    #endif
    }
    bAttached = !bAttached;
}


/** Fills in the RCONTACTS-ARRAY with contact information
    between PSB and PCO. */
void GripperKinematicObject::
getContactPointsWith(btSoftBody *psb,
		     btCollisionObject *pco,
		     btSoftBody::tRContactArray &rcontacts) {

  /** Custom contact checking adapted from
      btSoftBody.cpp and btSoftBodyInternals.h */
  struct Custom_CollideSDF_RS : btDbvt::ICollide {

    Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_):
      rcontacts(rcontacts_) { }

    void Process(const btDbvtNode* leaf) {
      btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
      DoNode(*node);
    }

    void DoNode(btSoftBody::Node& n) {
      // n.m_im : is the inverse mass of the node.
      const btScalar m = n.m_im > 0 ? dynmargin : stamargin;

      btSoftBody::RContact c;
      if (!n.m_battach && psb->checkContact(m_colObj1, n.m_x, m, c.m_cti)) {
	const btScalar ima = n.m_im;
	const btScalar imb = m_rigidBody? m_rigidBody->getInvMass() : 0.f;
	const btScalar ms = ima + imb;
	if(ms > 0) {
	  c.m_node = &n;
	  rcontacts.push_back(c);
	}
      }
    }

    btSoftBody*          psb;
    btCollisionObject*   m_colObj1;
    btRigidBody*         m_rigidBody;
    btScalar             dynmargin;
    btScalar             stamargin;
    btSoftBody::tRContactArray &rcontacts;
  };

  Custom_CollideSDF_RS  docollide(rcontacts);
  btRigidBody*  prb1=btRigidBody::upcast(pco);
  btTransform  wtr=pco->getWorldTransform();

  const btTransform ctr=pco->getWorldTransform();
  const btScalar timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
  const btScalar basemargin=psb->getCollisionShape()->getMargin();

  btVector3 mins;
  btVector3 maxs;
  ATTRIBUTE_ALIGNED16(btDbvtVolume) volume;
  pco->getCollisionShape()->getAabb(pco->getWorldTransform(),
				    mins, maxs);
  volume = btDbvtVolume::FromMM(mins,maxs);
  volume.Expand(btVector3(basemargin,basemargin,basemargin));

  docollide.psb = psb;
  docollide.m_colObj1 = pco;
  docollide.m_rigidBody = prb1;

  docollide.dynmargin = basemargin+timemargin;
  docollide.stamargin = basemargin;

  /* Ask the sofbody to do the collision checking and give it the callback
     which stores the collidin node on the softbody. */
  psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root, volume, docollide);
}


/** Adapted from btSoftBody.cpp (btSoftBody::appendAnchor).

    Appends an anchor to the soft-body, connecting
    node and the given rigidBody.

    PSB  : The softbody to which the anchor is appended.
    NODE : The specific node on which the anchor is attached.
    BODY : The other end of anchor. The rigid body to which
           the anchor is attached.
    INFLUENCE : \in [0,1] : value of 1 means max attachment. */
void GripperKinematicObject::appendAnchor(btSoftBody *psb,
					  btSoftBody::Node *node,
					  btRigidBody *body,
					  btScalar influence) {
    btSoftBody::Anchor a;
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = 1;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
}


/** Constructor for the GRIPPER object.
    COLOR: is the color in which the top_jaw and bottom_jaw of
           the gripper would be rendered. */
GripperKinematicObject::GripperKinematicObject(btVector4 color) {
  bAttached = false;
  closed_gap = 0.1;
  apperture = 2;
  // The gripper jaws are 0.6x0.6x0.1 (lxbxh) cuboids.
  halfextents = btVector3(.3,.3,0.1);

  // The top jaw of the gripper.
  BoxObject::Ptr top_jaw(new BoxObject(0, halfextents,
				       btTransform(btQuaternion(0,0,0,1),
						   btVector3(0,0,apperture/2)),
				       true));
  top_jaw->setColor(color[0],color[1],color[2],color[3]);

  // The bottom jaw of the gripper.
  BoxObject::Ptr bottom_jaw(new BoxObject(0, halfextents,
					  btTransform(btQuaternion(0,0,0,1),
						  btVector3(0,0,-apperture/2)),
					  true));
  bottom_jaw->setColor(color[0],color[1],color[2],color[3]);

  // CUR_TM is the mid point of top_jaw's and bottom_jaw's origins.
  top_jaw->motionState->getWorldTransform(cur_tm);
  cur_tm.setOrigin(cur_tm.getOrigin() - btVector3(0,0,-apperture/2));

  // Initially the gripper is open.
  bOpen = true;

  // Store the jaws in a vector.
  children.push_back(top_jaw);
  children.push_back(bottom_jaw);
}


/** Sets the transform of the gripper.
    TM : The transform to be applied to the mid-point
         of the top and the bottom jaws' origins. 
    Updates this.cur_tm : the transfrom returned by
    this.getWorldTransform() */
void GripperKinematicObject::setWorldTransform(btTransform tm) {
    btTransform top_tm = tm;
    btTransform bottom_tm = tm;

    btTransform top_offset;

    children[0]->motionState->getWorldTransform(top_offset);
    top_offset = cur_tm.inverse()*top_offset;

    top_tm.setOrigin(top_tm.getOrigin()
		     + (top_tm.getBasis().getColumn(2)
			*(top_offset.getOrigin()[2])));
    bottom_tm.setOrigin(bottom_tm.getOrigin()
			- (bottom_tm.getBasis().getColumn(2)
			   *(top_offset.getOrigin()[2])));

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    cur_tm = tm;
}


/** Opens/ closes the gripper, i.e. sets the appropriate transform. */
void GripperKinematicObject::toggle() {
    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    if(bOpen) { // then close
      top_tm.setOrigin(cur_tm.getOrigin()
		       + (cur_tm.getBasis().getColumn(2)
			  *(children[0]->halfExtents[2] + closed_gap/2)));

      bottom_tm.setOrigin(cur_tm.getOrigin()
			  - (cur_tm.getBasis().getColumn(2)
			     *(children[1]->halfExtents[2] + closed_gap/2)));
    } else { // open
      top_tm.setOrigin(cur_tm.getOrigin()
		       - cur_tm.getBasis().getColumn(2)*(apperture/2));
      bottom_tm.setOrigin(cur_tm.getOrigin()
			  + cur_tm.getBasis().getColumn(2)*(apperture/2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    bOpen = !bOpen;
}
