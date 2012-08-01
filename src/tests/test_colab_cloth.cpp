#include "colab_cloth.h"


/** Simply copies the coordinates of the nodes in M_NODES
    into NODEPOSEVEC. */
void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node>
			      &m_nodes,
			      std::vector<btVector3> &nodeposvec) {
  nodeposvec.resize(m_nodes.size());
  for(int i =0; i < m_nodes.size(); i++)
    nodeposvec[i] = m_nodes[i].m_x;
}


void GripperKinematicObject::applyTransform(btTransform tm) {
  setWorldTransform(getWorldTransform()*tm);
}

void GripperKinematicObject::translate(btVector3 transvec)
{
    btTransform tm = getWorldTransform();
    tm.setOrigin(tm.getOrigin() + transvec);
    setWorldTransform(tm);
}

void GripperKinematicObject::toggleattach(btSoftBody * psb, double radius) {

    if(bAttached)
    {
        btAlignedObjectArray<btSoftBody::Anchor> newanchors;
        for(int i = 0; i < psb->m_anchors.size(); i++)
        {
            if(psb->m_anchors[i].m_body != children[0]->rigidBody.get() && psb->m_anchors[i].m_body != children[1]->rigidBody.get())
                newanchors.push_back(psb->m_anchors[i]);
        }
        releaseAllAnchors(psb);
        for(int i = 0; i < newanchors.size(); i++)
        {
            psb->m_anchors.push_back(newanchors[i]);
        }
        vattached_node_inds.clear();
    }
    else
    {
#ifdef USE_RADIUS_CONTACT
        if(radius == 0)
            radius = halfextents[0];
        btTransform top_tm;
        children[0]->motionState->getWorldTransform(top_tm);
        btTransform bottom_tm;
        children[1]->motionState->getWorldTransform(bottom_tm);
        int closest_body = -1;
        for(int j = 0; j < psb->m_nodes.size(); j++)
        {
            if((psb->m_nodes[j].m_x - cur_tm.getOrigin()).length() < radius)
            {
                if( (psb->m_nodes[j].m_x - top_tm.getOrigin()).length() < (psb->m_nodes[j].m_x - bottom_tm.getOrigin()).length() )
                    closest_body = 0;
                else
                    closest_body = 1;

                vattached_node_inds.push_back(j);
                appendAnchor(psb, &psb->m_nodes[j], children[closest_body]->rigidBody.get());
                cout << "\tappending anchor, closest ind: "<< j << "\n";

            }
        }
#else
        std::vector<btVector3> nodeposvec;
        nodeArrayToNodePosVector(psb->m_nodes, nodeposvec);

        for(int k = 0; k < 2; k++)
        {
            BoxObject::Ptr part = children[k];

            btRigidBody* rigidBody = part->rigidBody.get();
            btSoftBody::tRContactArray rcontacts;
            getContactPointsWith(psb, rigidBody, rcontacts);
            cout << "got " << rcontacts.size() << " contacts\n";

            //if no contacts, return without toggling bAttached
            if(rcontacts.size() == 0)
                continue;
            for (int i = 0; i < rcontacts.size(); ++i) {
                //const btSoftBody::RContact &c = rcontacts[i];
                btSoftBody::Node *node = rcontacts[i].m_node;
                //btRigidBody* colLink = c.m_cti.m_colObj;
                //if (!colLink) continue;
                const btVector3 &contactPt = node->m_x;
                //if (onInnerSide(contactPt, left)) {
                    int closest_ind = -1;
                    for(int n = 0; n < nodeposvec.size(); n++)
                    {
                        if((contactPt - nodeposvec[n]).length() < 0.0001)
                        {
                            closest_ind = n;
                            break;
                        }
                    }
                    assert(closest_ind!=-1);

                    vattached_node_inds.push_back(closest_ind);
                    appendAnchor(psb, node, rigidBody);
                    cout << "\tappending anchor, closest ind: "<< closest_ind << "\n";
            }
        }
#endif
    }

    bAttached = !bAttached;
}

/** Fills in the rcontacs array with contact information
    etween psb and pco. */
void GripperKinematicObject::
getContactPointsWith(btSoftBody *psb,
		     btCollisionObject *pco,
		     btSoftBody::tRContactArray &rcontacts) {

  /** Custom contact checking adapted from
      btSoftBody.cpp and btSoftBodyInternals.h */
  struct Custom_CollideSDF_RS : btDbvt::ICollide {

    Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_):
      rcontacts(rcontacts_) {} 
    
    void Process(const btDbvtNode* leaf) {
      btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
      DoNode(*node);
    }


    void DoNode(btSoftBody::Node& n) {
      // n.m_im : is the inverse mass of the node.
      const btScalar m=n.m_im>0?dynmargin:stamargin;

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
    
    btSoftBody*             psb;
    btCollisionObject*      m_colObj1;
    btRigidBody*    m_rigidBody;
    btScalar                dynmargin;
    btScalar                stamargin;
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
  bOpen = true;
  children.push_back(top_jaw);
  children.push_back(bottom_jaw);
}

/** Sets the transform of the gripper.
    TM : The transform to be applied to the mid-point
         of the top and the bottom jaw. */
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


void GripperKinematicObject::toggle()
{
    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    if(bOpen)
    {
        //btTransform top_offset = cur_tm.inverse()*top_tm;
        //float close_length = (1+closed_gap)*top_offset.getOrigin()[2] - children[0]->halfExtents[2];
        //float close_length = (apperture/2 - children[0]->halfExtents[2] + closed_gap/2);
        top_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(children[0]->halfExtents[2] + closed_gap/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(children[1]->halfExtents[2] + closed_gap/2));
    }
    else
    {
        top_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(apperture/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(apperture/2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    bOpen = !bOpen;
}


void CustomScene::simulateInNewFork(StepState& innerstate, float sim_time,
				    btTransform& left_gripper1_tm,
				    btTransform& left_gripper2_tm) {

    innerstate.bullet.reset(new BulletInstance);
    innerstate.bullet->setGravity(BulletConfig::gravity);
    innerstate.osg.reset(new OSGInstance);
    innerstate.fork.reset(new Fork(env, innerstate.bullet, innerstate.osg));
    innerstate.cloth = boost::static_pointer_cast<BulletSoftObject>
      (innerstate.fork->forkOf(clothptr));
    innerstate.left_gripper1 =
      boost::static_pointer_cast<GripperKinematicObject>
      (innerstate.fork->forkOf(left_gripper1));
    innerstate.left_gripper2 = 
      boost::static_pointer_cast<GripperKinematicObject>
      (innerstate.fork->forkOf(left_gripper2));

    innerstate.left_gripper1->applyTransform(left_gripper1_tm);
    innerstate.left_gripper2->applyTransform(left_gripper2_tm);

    float time = sim_time;
    while (time > 0) {
        innerstate.fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps,
				   BulletConfig::internalTimeStep);
        time -= BulletConfig::dt;
    }
}


double CustomScene::getDistfromNodeToClosestAttachedNodeInGripper
(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind) {
  double min_dist = 1000000;
  closest_ind = -1;

  for(int i = 0; i < gripper->vattached_node_inds.size(); i += 1) {
    double new_dist = cloth_distance_matrix(gripper->vattached_node_inds[i],
					    input_ind);
    if(new_dist < min_dist) {
      min_dist = new_dist;
      closest_ind = gripper->vattached_node_inds[i];
    }
  }

  return min_dist;
}



Eigen::MatrixXf CustomScene::computeJacobian_approx()
{

    double dropoff_const = 1.0;//0.7;
    int numnodes = clothptr->softBody->m_nodes.size();

    std::vector<btTransform> perts;
    float step_length = 0.2;
    float rot_angle = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
#ifdef DO_ROTATION
    #ifdef USE_QUATERNION
        ///NOT IMPLEMENTED!!!!
        perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    #else
        perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    #endif
#endif

    Eigen::MatrixXf J(numnodes*3,perts.size()*num_auto_grippers);
    GripperKinematicObject::Ptr gripper;

    std::vector<btVector3> rot_line_pnts;
    std::vector<btVector4> plot_cols;


    double rotation_scaling = 50;
    omp_set_num_threads(4);

    for(int g = 0; g < num_auto_grippers; g++) {
      if(g == 0)
	gripper = left_gripper1;
      if(g == 1)
	gripper = left_gripper2;
      
#pragma omp parallel shared(J)
      {
       #pragma omp for
      for(int i = 0; i < perts.size(); i++) {
	Eigen::VectorXf  V_pos(numnodes*3);
	for(int k = 0; k < numnodes; k++) {
	  int closest_ind;
	  double dist = getDistfromNodeToClosestAttachedNodeInGripper(gripper, 
								      k,closest_ind);

	  if(i < 3) { // TRANSLATION
	    btVector3 transvec = ( ((gripper->getWorldTransform()*perts[i]).getOrigin()
				    - gripper->getWorldTransform().getOrigin())
				   * exp(-dist * dropoff_const)/step_length);
	    for(int j = 0; j < 3; j++)
	      V_pos(3*k + j) = transvec[j];

	  } else {// ROTATION

	    /** Get the vector of translation induced at
		closest attached point by the rotation
		about the center of the gripper. */
	    btTransform T0_attached = 
	      btTransform(btQuaternion(0,0,0,1),
			  clothptr->softBody->m_nodes[closest_ind].m_x);
	    btTransform T0_center = gripper->getWorldTransform();
	    btTransform Tcenter_attached = T0_center.inverse()*T0_attached;
	    btTransform T0_newattached =  T0_center*perts[i]*Tcenter_attached;
	    btVector3 transvec = ( (T0_attached.inverse()*T0_newattached).getOrigin()
				   /rot_angle * exp(-dist*dropoff_const));
	    
	    for(int j = 0; j < 3; j++)
	      V_pos(3*k + j) = transvec[j]*rotation_scaling;
	    
	  } 
	}
	// Store the approx values in J
	J.col(perts.size()*g + i) = V_pos;
      }
      }//end omp
    }
    rot_lines->setPoints(rot_line_pnts,plot_cols);
    return J;
}

Eigen::MatrixXf CustomScene::computeJacobian_parallel()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);


    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }



    std::vector<btTransform> perts;
    float step_length = 0.2;
    float rot_angle = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
#ifdef DO_ROTATION
    perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    omp_set_num_threads(7); //need to find a better way to do this
#else
    omp_set_num_threads(4);
#endif

    Eigen::MatrixXf J(numnodes*3,perts.size());
    #pragma omp parallel shared(J)
    {

        //schedule(static, 1)
        #pragma omp for nowait
        for(int i = 0; i < perts.size(); i++)
        {

            btTransform dummy_tm;
            StepState innerstate;
            simulateInNewFork(innerstate, jacobian_sim_time, perts[i],dummy_tm);

            Eigen::VectorXf  V_after(V_before);
            for(int k = 0; k < numnodes; k++)
            {
                for(int j = 0; j < 3; j++)
                    V_after(3*k + j) = innerstate.cloth->softBody->m_nodes[k].m_x[j];
            }
            float divider;
            if(i < 3)
                divider = step_length;
            else
                divider = rot_angle;

            J.col(i) = (V_after - V_before)/divider;
        }
    }

    //cout << J<< endl;
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;


}





Eigen::MatrixXf CustomScene::computeJacobian()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);
    Eigen::VectorXf  V_after(V_before);

    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }

    Eigen::MatrixXf J(numnodes*3,3);

    std::vector<btTransform> perts;
    float step_length = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
    float time;

    for(int i = 0; i < 3 ; i++)
    {
        createFork();
        swapFork(); //now pointers are set to the forked objects

        //apply perturbation
        left_gripper1->applyTransform(perts[i]);

        time = jacobian_sim_time;
        while (time > 0) {
            float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;
            if (syncTime && drawingOn)
                endTime = viewer.getFrameStamp()->getSimulationTime();

            // run pre-step callbacks
            //for (int j = 0; j < prestepCallbackssize(); ++j)
            //    prestepCallbacks[j]();

            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            draw();

            if (syncTime && drawingOn) {
                float timeLeft = BulletConfig::dt - (endTime - startTime);
                idleFor(timeLeft);
                startTime = endTime + timeLeft;
            }


            time -= BulletConfig::dt;

        }

        for(int k = 0; k < numnodes; k++)
        {
            for(int j = 0; j < 3; j++)
                V_after(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
        }

        destroyFork();
        J.col(i) = (V_after - V_before)/step_length;
    }

    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

    loopState.skip_step = bBackupLoopState;
    return J;
}

/** Returns the Moore-Penrose psuedo inverse of the input matrix MAT.
    This psuedo-inverse is the standard in MATLAB.

    see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse */
Eigen::MatrixXf pinv(const Eigen::MatrixXf &mat) {

  if ( mat.rows() < mat.cols() ) {
    cout << "Pseudo-inverse error : number of cols > number of rows." << endl;
    return Eigen::MatrixXf();
  }
  
  // singular value decomposition.
  Eigen::JacobiSVD<Eigen::MatrixXf>
    svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

  /** Build a diagonal matrix with the inverted singular values
      The pseudo inverted singular matrix is formed by replacing
      every nonzero entry by its reciprocal. 
      Sufficiently small values are treated as zero. */
  Eigen::MatrixXf singular_values = svd.singularValues();
  Eigen::MatrixXf inverted_singular_values(svd.matrixV().cols(), 1);

  for (int iRow = 0; iRow < singular_values.rows(); iRow += 1) {
    if ( fabs(singular_values(iRow)) <= 1e-10 ) //Todo: Put epsilon in parameter	
      inverted_singular_values(iRow, 0) = 0.;
    else
      inverted_singular_values(iRow,0) = 1./singular_values(iRow);
  }

  Eigen::MatrixXf mAdjointU = svd.matrixU().adjoint().
    block(0, 0, singular_values.rows(), svd.matrixU().adjoint().cols());

  // Pseudo-Inverse = V * S * U'
  return (svd.matrixV() * inverted_singular_values.asDiagonal()) * mAdjointU;
}


void CustomScene::doJTracking() {

    //this loop is already being executed by someone else, abort
    if(bInTrackingLoop)
        return;
    else
        bInTrackingLoop = true;


    loopState.skip_step = true;
    int numnodes = clothptr->softBody->m_nodes.size();

    float approx_thresh = 5;

    float step_limit = 0.05;
    Eigen::VectorXf V_step(numnodes*3);

    Eigen::VectorXf V_trans;
    btTransform transtm1,transtm2;
    Eigen::MatrixXf J;
    while(bTracking) {

      for(int i = 0; i < numnodes*3;i++)
	V_step(i) = 0;
      
      float error = 0;
      std::vector<btVector3> plotpoints;
      std::vector<btVector4> plotcols;
      float node_change = 0;
      int counter = 0;
      for( map<int,int>::iterator ii=node_mirror_map.begin();
	   ii!=node_mirror_map.end(); ++ii) {
	
	counter += 1;
	btVector3 targpoint =
	  point_reflector->reflect(clothptr->softBody->m_nodes[(*ii).second].m_x);
	
	btVector3 targvec =
	  targpoint - clothptr->softBody->m_nodes[(*ii).first].m_x;
	error = error + targvec.length();
	for(int j = 0; j < 3; j++)
	  V_step(3*(*ii).first + j) = targvec[j];
	
	
	plotpoints.push_back(clothptr->softBody->m_nodes[(*ii).first].m_x);
	plotcols.push_back(btVector4(targvec.length(),0,0,1));
	
	node_change += (clothptr->softBody->m_nodes[(*ii).first].m_x
		       - prev_node_pos[(*ii).first]).length();
	
      }
      nodeArrayToNodePosVector(clothptr->softBody->m_nodes, prev_node_pos);
      
      plot_points->setPoints(plotpoints,plotcols);
      
      cout << "Error: " << error << " ";
      
      cout << "(" << node_change <<")"<<endl;
      
      J = computeJacobian_approx();
      cout<<"Map size: "<<counter<<" | Number of nodes: "<<numnodes<<endl;
      cout<<"J : "<<J.rows()<<"x"<<J.cols()<<endl;
      
      node_change = error;
	
      Eigen::MatrixXf Jpinv= pinv(J.transpose()*J)*J.transpose();
      V_trans = Jpinv*V_step;
      
      cout<<"J inverse : "<<Jpinv.rows()<<"x"<<Jpinv.cols()<<endl;
      cout<<"V step : "<<V_step.rows()<<"x"<<V_step.cols()<<endl;
      
      
      // Limit big motions to STEP_LIMIT
      if(V_trans.norm() > step_limit)
	V_trans = V_trans/V_trans.norm()*step_limit;
      
      // Don't move if the movements are too small.
      if(V_trans.norm() < 0.008) {
	cout<<"Movements are too small. Not moving."<<endl;
	break;  
      }
	  
#ifdef DO_ROTATION
    #ifdef USE_QUATERNION
        ///NOT IMPLEMENTED!
        btVector4 dquat1(V_trans(3),V_trans(4),V_trans(5),V_trans(6));
        btVector4 dquat2(V_trans(10),V_trans(11),V_trans(12),V_trans(13));
        dquat1 = dquat1*1/dquat1.length();
        dquat2 = dquat2*1/dquat2.length();

        transtm1 = btTransform(btQuaternion(dquat1),
                              btVector3(V_trans(0),V_trans(1),V_trans(2)));
        transtm2 = btTransform(btQuaternion(dquat2),
                                  btVector3(V_trans(7),V_trans(8),V_trans(9)));


    #else

        transtm1 = btTransform(btQuaternion(btVector3(0,0,1),V_trans(5))*
                              btQuaternion(btVector3(0,1,0),V_trans(4))*
                              btQuaternion(btVector3(1,0,0),V_trans(3)),
                              btVector3(V_trans(0),V_trans(1),V_trans(2)));
        transtm2 = btTransform(btQuaternion(btVector3(0,0,1),V_trans(11))*
                                  btQuaternion(btVector3(0,1,0),V_trans(10))*
                                  btQuaternion(btVector3(1,0,0),V_trans(9)),
                                  btVector3(V_trans(6),V_trans(7),V_trans(8)));
    #endif
#else
        transtm1 = btTransform(btQuaternion(0,0,0,1), btVector3(V_trans(0),V_trans(1),V_trans(2)));
        if(num_auto_grippers > 1)
            transtm2 = btTransform(btQuaternion(0,0,0,1), btVector3(V_trans(3),V_trans(4),V_trans(5)));
        else
            transtm2 = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
#endif
        //check is it more symmetric than it would have been had you done nothing
        float errors[2];
        if(0 && errors[0] >= errors[1]) {
	  cout << "Error increase, not moving (new error: " <<  error << ")" << endl;
	  break;
        } else {
	  cout << endl;
	  left_gripper1->applyTransform(transtm1);
	  left_gripper2->applyTransform(transtm2);
#ifdef USE_PR2
	  btTransform left1(left_gripper1->getWorldTransform());
	  btTransform TOR_newtrans = left1*TBullet_PR2GripperRight;
	  TOR_newtrans.setOrigin(left1.getOrigin());
	  pr2m.pr2Right->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
	  
	  btTransform left2(left_gripper2->getWorldTransform());
	  TOR_newtrans = left2*TBullet_PR2GripperLeft;
	  TOR_newtrans.setOrigin(left2.getOrigin());
	  pr2m.pr2Left->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
        }
        break;
	
    }

    loopState.skip_step = false;
    bInTrackingLoop = false;
}

//--------------------
void CustomScene::regraspWithOneGripper
(GripperKinematicObject::Ptr gripper_to_attach,
 GripperKinematicObject::Ptr  gripper_to_detach) {

  gripper_to_detach->toggleattach(clothptr->softBody.get());
  gripper_to_detach->toggle();

  gripper_to_attach->toggleattach(clothptr->softBody.get());
  
  float apperture = gripper_to_attach->apperture;
  gripper_to_attach->apperture = 0.1;
  gripper_to_attach->toggle();
  gripper_to_attach->apperture = apperture;
  
  gripper_to_attach->toggleattach(clothptr->softBody.get());
  gripper_to_attach->toggle();
}


class CustomKeyHandler : public osgGA::GUIEventHandler {
  CustomScene &scene;
public:
  CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};


bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,
			      osgGA::GUIActionAdapter & aa) {
  switch (ea.getEventType()) {
  case osgGA::GUIEventAdapter::KEYDOWN:
    switch (ea.getKey()) {

      //'1', '2', 'q', 'w' reservered for PR2!
      
      /** These set of keys: {'3', 'a','4','s','5','e','6','r'}
	  should be used simultaneously with the mouse to move
	  the desired gripper. 
	  
	  The numeric keys cause TRANSLATION.
	  The alphabetical keys cause ROTATION. */
      
      // Robot's side :: right gripper
    case '3':
      scene.inputState.transGrabber0 = true; break;
    case 'a':
      scene.inputState.rotateGrabber0 = true; break;

      // Robot's side :: left gripper
    case '4':
      scene.inputState.transGrabber1 = true; break;
    case 's':
      scene.inputState.rotateGrabber1 = true; break;

      // Human's side :: left gripper
    case '5':
      scene.inputState.transGrabber2 = true; break;
    case 'e':
      scene.inputState.rotateGrabber2 = true; break;

      // Human's side :: right gripper
    case '6':
      scene.inputState.transGrabber3 = true; break;
    case 'r':
      scene.inputState.rotateGrabber3 = true; break;

#ifdef USE_PR2
        case '9':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);

            break;
        case '0':

            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);

            break;
#endif
        case 'c':
        {
            scene.regraspWithOneGripper(scene.left_gripper1,scene.left_gripper2);
            break;
        }

        case 'v':
        {
            scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
            break;
        }

        case 'f':
        {
            scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
            scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
            break;
        }

        case 'g':
        {
            scene.regraspWithOneGripper(scene.right_gripper2,scene.left_gripper2);
            scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,110)));
            break;
        }


        case 'k':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;

        case ',':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;


        case 'l':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;

        case '.':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;


        case 'y':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;


        case 'u':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;


        case 'i':
            scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.robot_mid_point_ind].m_x));
            break;

        case 'o':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.user_mid_point_ind].m_x));
            break;


        case 'b':
            if(scene.right_gripper2->bOpen)
                scene.right_gripper2->state = GripperState_CLOSING;
            else
                scene.right_gripper2->state = GripperState_OPENING;

            break;

        case 'n':
            if(scene.left_gripper2->bOpen)
                scene.left_gripper2->state = GripperState_CLOSING;
            else
                scene.left_gripper2->state = GripperState_OPENING;

            break;

        case 'j':
            {
#ifdef PROFILER
                if(!scene.bTracking)
                    ProfilerStart("profile.txt");
                else
                    ProfilerStop();
#endif

               nodeArrayToNodePosVector(scene.clothptr->softBody->m_nodes, scene.prev_node_pos);
               scene.bTracking = !scene.bTracking;
               if(!scene.bTracking)
                   scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

                break;
            }

//        case 'b':
//            scene.stopLoop();
//            break;
        }
        break;

    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '3':
            scene.inputState.transGrabber0 = false; break;
            break;
        case 'a':
            scene.inputState.rotateGrabber0 = false; break;
        case '4':
            scene.inputState.transGrabber1 = false; break;
        case 's':
            scene.inputState.rotateGrabber1 = false; break;
        case '5':
            scene.inputState.transGrabber2 = false; break;
        case 'e':
            scene.inputState.rotateGrabber2 = false; break;
        case '6':
            scene.inputState.transGrabber3 = false; break;
        case 'r':
            scene.inputState.rotateGrabber3 = false; break;


        }
        break;

    case osgGA::GUIEventAdapter::PUSH:
        scene.inputState.startDragging = true;
        break;

    case osgGA::GUIEventAdapter::DRAG:
        if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG){
            // drag the active manipulator in the plane of view
            if ( (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
                  (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0 ||
                   scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1 ||
                   scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2 ||
                   scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)) {
                if (scene.inputState.startDragging) {
                    scene.inputState.dx = scene.inputState.dy = 0;
                } else {
                    scene.inputState.dx = scene.inputState.lastX - ea.getXnormalized();
                    scene.inputState.dy = ea.getYnormalized() - scene.inputState.lastY;
                }
                scene.inputState.lastX = ea.getXnormalized(); scene.inputState.lastY = ea.getYnormalized();
                scene.inputState.startDragging = false;

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene.manip->getTransformation(osgCenter, osgEye, osgUp);
                btVector3 from(util::toBtVector(osgEye));
                btVector3 to(util::toBtVector(osgCenter));
                btVector3 up(util::toBtVector(osgUp)); up.normalize();

                // compute basis vectors for the plane of view
                // (the plane normal to the ray from the camera to the center of the scene)
                btVector3 normal = (to - from).normalized();
                btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
                btVector3 xVec = normal.cross(yVec);
                btVector3 dragVec = SceneConfig::mouseDragScale*10 * (scene.inputState.dx*xVec + scene.inputState.dy*yVec);
                //printf("dx: %f dy: %f\n",scene.inputState.dx,scene.inputState.dy);

                btTransform origTrans;
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    scene.left_gripper1->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1)
                {
                    scene.left_gripper2->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2)
                {
                    scene.right_gripper1->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)
                {
                    scene.right_gripper2->getWorldTransform(origTrans);
                }

                //printf("origin: %f %f %f\n",origTrans.getOrigin()[0],origTrans.getOrigin()[1],origTrans.getOrigin()[2]);

                btTransform newTrans(origTrans);

                if (scene.inputState.transGrabber0 || scene.inputState.transGrabber1  ||
                        scene.inputState.transGrabber2  || scene.inputState.transGrabber3)
                    // if moving the manip, just set the origin appropriately
                    newTrans.setOrigin(dragVec + origTrans.getOrigin());
                else if (scene.inputState.rotateGrabber0 || scene.inputState.rotateGrabber1 ||
                         scene.inputState.rotateGrabber2 || scene.inputState.rotateGrabber3) {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = normal.cross(dragVec);
                    btScalar angle = dragVec.length();
                    btQuaternion rot(axis, angle);
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small (effectively zero) mouse movements
                    // this is the easiest way to do this:
                    if (rot.length() > 0.99f && rot.length() < 1.01f)
                        newTrans.setRotation(rot * origTrans.getRotation());
                }
                //printf("newtrans: %f %f %f\n",newTrans.getOrigin()[0],newTrans.getOrigin()[1],newTrans.getOrigin()[2]);
                //softbody ->addForce(const btVector3& forceVector,int node)

//                std::vector<btVector3> plot_line;
//                std::vector<btVector4> plot_color;
//                plot_line.push_back(origTrans.getOrigin());
//                plot_line.push_back(origTrans.getOrigin() + 100*(newTrans.getOrigin()- origTrans.getOrigin()));
//                plot_color.push_back(btVector4(1,0,0,1));
//                scene.drag_line->setPoints(plot_line,plot_color);
                //btTransform TBullet_PR2Gripper = btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0));
                //btTransform TOR_newtrans = TBullet_PR2Gripper*newTrans;
                //TOR_newtrans.setOrigin(newTrans.getOrigin());
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    scene.left_gripper1->setWorldTransform(newTrans);
#ifdef USE_PR2
                    btTransform TOR_newtrans = newTrans*TBullet_PR2GripperRight;
                    TOR_newtrans.setOrigin(newTrans.getOrigin());
                    scene.pr2m.pr2Right->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
                }
                else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1)
                {
                    scene.left_gripper2->setWorldTransform(newTrans);
#ifdef USE_PR2
                    btTransform TOR_newtrans = newTrans*TBullet_PR2GripperLeft;
                    TOR_newtrans.setOrigin(newTrans.getOrigin());
                    scene.pr2m.pr2Left->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
                }
                else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2)
                {
                    scene.right_gripper1->setWorldTransform(newTrans);
                }
                else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)
                {
                    scene.right_gripper2->setWorldTransform(newTrans);
                }
                return true;
            }
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center) {
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 10;//2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS      ;//  | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    //pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
    psb->m_cfg.kDP = 0.1;//0.05;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);
    //psb->generateClusters(500);

/*    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::createFork() {
    if(fork)
    {
        destroyFork();
    }
    bullet2.reset(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    //cout << "forked!" << endl;

#ifdef USE_PR2
    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
#endif
    left_gripper1_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(left_gripper1));
    right_gripper1_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(right_gripper1));
    clothptr_fork = boost::static_pointer_cast<BulletSoftObject> (fork->forkOf(clothptr));

}

void CustomScene::destroyFork() {
    if(left_gripper1.get() == left_gripper1_fork.get())
    {
        left_gripper1 = left_gripper1_orig;
        right_gripper1 = right_gripper1_orig;
        clothptr = clothptr_orig;
    }

    unregisterFork(fork);
    osg->root->removeChild(osg2->root.get());
    fork.reset();
    left_gripper1_fork.reset();
    right_gripper1_fork.reset();
    clothptr_fork.reset();
}

void CustomScene::swapFork() {
#ifdef USE_PR2
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);
#endif
    if(left_gripper1.get() == left_gripper1_orig.get())
    {
        left_gripper1 = left_gripper1_fork;
        right_gripper1 = right_gripper1_fork;
        clothptr = clothptr_fork;
    }
    else
    {
        left_gripper1 = left_gripper1_orig;
        right_gripper1 = right_gripper1_orig;
        clothptr = clothptr_orig;
    }



/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}


int getExtremalPoint(std::vector<btVector3> &pnt_vec, btMatrix3x3 projection_matrix, int first_dim_minmax, int second_dim_minmax, int third_dim_minmax)
{
    //first_dim_maxmin = 0 if min, first_dim_maxmin = 1 if max, first_dim_maxmin = -1 don't care
    btVector3 extremal_pnt = btVector3(pow(-1,first_dim_minmax)*1000,pow(-1,second_dim_minmax)*1000,pow(-1,third_dim_minmax)*1000);
    btVector3 projected_pnt;
    int extremal_ind = -1;
    bool bDimOK_1,bDimOK_2,bDimOK_3;
    for(int i = 0; i < pnt_vec.size();i++)
    {
        projected_pnt = projection_matrix * pnt_vec[i];

        bDimOK_1 = bDimOK_2 = bDimOK_3 = false;

        if(first_dim_minmax == -1)
            bDimOK_1 = true;
        if(second_dim_minmax == -1)
            bDimOK_2 = true;
        if(third_dim_minmax == -1)
            bDimOK_3 = true;

        if(projected_pnt[0] >= extremal_pnt[0] && first_dim_minmax == 1)
            bDimOK_1 = true;

        if(projected_pnt[0] <= extremal_pnt[0] && !first_dim_minmax)
            bDimOK_1 = true;

        if(projected_pnt[1] >= extremal_pnt[1] && second_dim_minmax == 1)
            bDimOK_2 = true;

        if(projected_pnt[1] <= extremal_pnt[1] && !second_dim_minmax )
            bDimOK_2 = true;

        if(projected_pnt[2] >= extremal_pnt[2] && third_dim_minmax == 1)
            bDimOK_3 = true;

        if(projected_pnt[2] <= extremal_pnt[2] && !third_dim_minmax)
            bDimOK_3 = true;


        if(bDimOK_1 && bDimOK_2 && bDimOK_3) {
	  extremal_pnt = projected_pnt;
	  extremal_ind = i;
        }
    }
    return extremal_ind;
}

void CustomScene::drawAxes() {
  left_axes1->setup(left_gripper1->getWorldTransform(),1);
  left_axes2->setup(left_gripper2->getWorldTransform(),1);
}

void CustomScene::run() {
  viewer.addEventHandler(new CustomKeyHandler(*this));
  
  addPreStepCallback(boost::bind(&CustomScene::doJTracking, this));
  addPreStepCallback(boost::bind(&CustomScene::drawAxes, this));


  const float dt = BulletConfig::dt;
  const float table_height = .7;

#ifdef USE_TABLE
  const float table_thickness = .05;
  BoxObject::Ptr table(new BoxObject(0, (GeneralConfig::scale
					 *btVector3(.75,.75,table_thickness/2)),
				     btTransform(btQuaternion(0, 0, 0, 1),
						 GeneralConfig::scale
						 * btVector3(1.0,0,table_height-table_thickness/2))));
  table->rigidBody->setFriction(1);
  env->add(table);
#endif

    BulletSoftObject::Ptr cloth(
            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.7, 0, table_height+0.01)));


    btSoftBody* psb = cloth->softBody.get();
    clothptr = clothptr_orig = cloth;
    psb->setTotalMass(0.1);


    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->right_gripper2,psb));
    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->left_gripper2,psb));

#ifdef USE_PR2
    pr2m.pr2->ignoreCollisionWith(psb);
    pr2m.pr2->ignoreCollisionWith(left_gripper1->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper1->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper2->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper2->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper1->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper1->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper2->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper2->getChildren()[1]->rigidBody.get());
#endif


    env->add(cloth);

    //left_mover.reset(new RigidMover(table, table->rigidBody->getCenterOfMassPosition(), env->bullet->dynamicsWorld));

#ifdef USE_PR2
    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(psb);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(psb);
#endif


    //btVector3 pos(0,0,0);
    //grab_left.reset(new Grab(psb, &psb->m_nodes[0], left_grabber->rigidBody.get()));
    //grab_left.reset(new Grab(psb, 0, left_grabber->rigidBody.get()));

    //psb->m_cfg.kAHR = 1;
    //psb->appendAnchor(0,left_grabber->rigidBody.get());
    //psb->appendAnchor(1,left_grabber->rigidBody.get());
    //psb->appendAnchor(2,left_grabber->rigidBody.get());


    int min_x_ind = -1;
    int max_x_ind = -1;
    int min_y_ind = -1;
    int max_y_ind = -1;
    double min_x = 100;
    double max_x = -100;
    double min_y = 100;
    double max_y = -100;

    //std::vector<float> node_x(psb->m_nodes.size());
    //std::vector<float> node_y(psb->m_nodes.size());
    std::vector<btVector3> node_pos(psb->m_nodes.size());
    std::vector<int> corner_ind(4,-1);
    std::vector<btVector3> corner_pnts(4);

    corner_pnts[0] = btVector3(100,100,0);
    corner_pnts[1] = btVector3(100,-100,0);
    corner_pnts[2] = btVector3(-100,100,0);
    corner_pnts[3] = btVector3(-100,-100,0);

    for(int i = 0; i < psb->m_nodes.size();i++)
    {
        //printf("%f\n", psb->m_nodes[i].m_x[0]);
//        double new_x = psb->m_nodes[i].m_x[0];
//        double new_y = psb->m_nodes[i].m_x[1];
        node_pos[i] = psb->m_nodes[i].m_x;

        if(node_pos[i][0] <= corner_pnts[0][0] && node_pos[i][1] <= corner_pnts[0][1])
        {
            corner_ind[0] = i;
            corner_pnts[0] = node_pos[i];
        }

        if(node_pos[i][0] <= corner_pnts[1][0] && node_pos[i][1] >= corner_pnts[1][1])
        {
            corner_ind[1] = i;
            corner_pnts[1] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[2][0] && node_pos[i][1] <= corner_pnts[2][1])
        {
            corner_ind[2] = i;
            corner_pnts[2] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[3][0] && node_pos[i][1] >= corner_pnts[3][1])
        {
            corner_ind[3] = i;
            corner_pnts[3] = node_pos[i];
        }

    }

    max_x = corner_pnts[3][0];
    max_y = corner_pnts[3][1];
    min_x = corner_pnts[0][0];
    min_y = corner_pnts[0][1];

    
    btTransform tm_left1 = btTransform(btQuaternion(0,0,0 ,1),
				       corner_pnts[0]
				       + btVector3(left_gripper1->children[0]
						   ->halfExtents[0],
						   left_gripper1->children[0]
						   ->halfExtents[1], 0) );
    left_gripper1->setWorldTransform(tm_left1);
    //left_gripper1->toggle();


    btTransform tm_right1 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[2] + btVector3(-right_gripper1->children[0]->halfExtents[0],right_gripper1->children[0]->halfExtents[1],0));
    right_gripper1->setWorldTransform(tm_right1);

    btTransform tm_left2 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[1] + btVector3(left_gripper2->children[0]->halfExtents[0],-left_gripper2->children[0]->halfExtents[1],0));
    left_gripper2->setWorldTransform(tm_left2);
    //left_gripper1->toggle();


    btTransform tm_right2 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[3] + btVector3(-right_gripper2->children[0]->halfExtents[0],-right_gripper2->children[0]->halfExtents[1],0));
    right_gripper2->setWorldTransform(tm_right2);

    gripper_node_distance_map.resize(num_auto_grippers);

    for(int i = 0; i < num_auto_grippers; i++)
    {
        gripper_node_distance_map[i].resize(node_pos.size());

        for(int j = 0; j < node_pos.size(); j++)
        {
            gripper_node_distance_map[i][j] = (corner_pnts[i]-node_pos[j]).length();
        }
    }


    cloth_distance_matrix = Eigen::MatrixXf( node_pos.size(), node_pos.size());
    for(int i = 0; i < node_pos.size(); i++)
    {
        for(int j = i; j < node_pos.size(); j++)
        {
            cloth_distance_matrix(i,j) = (node_pos[i]-node_pos[j]).length();
            cloth_distance_matrix(j,i) = cloth_distance_matrix(i,j);
        }
    }
    //cout << cloth_distance_matrix<< endl;

    //mirror about centerline along y direction;
    //centerline defined by 2 points
    float mid_x = (max_x + min_x)/2;
    point_reflector.reset(new PointReflector(mid_x, min_y, max_y));
    //find node that most closely matches reflection of point
    for(int i = 0; i < node_pos.size(); i++)
    {
        if(node_pos[i][0] < mid_x) //look at points in left half
        //if(node_pos[i][0] < mid_x && (abs(node_pos[i][0] - min_x) < 0.01 || abs(node_pos[i][1] - min_y) < 0.01 || abs(node_pos[i][1] - max_y) < 0.01))
        {
            //float reflected_x = node_pos[i][0] + 2*(mid_x - node_pos[i][0]);
            btVector3 new_vec = point_reflector->reflect(node_pos[i]);
            float closest_dist = 100000;
            int closest_ind = -1;
            for(int j = 0; j < node_pos.size(); j++)
            {
                float dist = (node_pos[j]-new_vec).length();//(node_pos[j][0]-reflected_x)*(node_pos[j][0]-reflected_x) + (node_pos[j][1]-node_pos[i][1])*(node_pos[j][1]-node_pos[i][1]);
                if(dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_ind = j;
                }
            }
            node_mirror_map[i] = closest_ind;
        }


    }

    //get boundary points

    btVector3 user_target_mid_point(max_x,(max_y + min_y)/2,corner_pnts[0][2]);
    btVector3 robot_target_mid_point(min_x,(max_y + min_y)/2,corner_pnts[0][2]);
    btVector3 user_mid_point(100,100,100);
    btVector3 robot_mid_point(100,100,100);

    double user_length= 1000;
    double robot_length= 1000;
    for(int i = 0; i < node_pos.size(); i++)
    {
        double this_user_length = (node_pos[i]-user_target_mid_point).length();
        if( this_user_length < user_length)
        {
            user_length = this_user_length;
            user_mid_point = node_pos[i];
            user_mid_point_ind = i;
        }

        double this_robot_length = (node_pos[i]-robot_target_mid_point).length();
        if( this_robot_length < robot_length)
        {
            robot_length = this_robot_length;
            robot_mid_point = node_pos[i];
            robot_mid_point_ind = i;
        }
    }
//    cout << "robot side midpoint: " << robot_mid_point[0] << " " << robot_mid_point[1] << " " << robot_mid_point[2] << endl;
//    cout << "user side midpoint: " << user_mid_point[0] << " " << user_mid_point[1] << " " << user_mid_point[2] << endl;


//    tm_left1 = btTransform(btQuaternion( 0,    0,    0 ,   1), robot_mid_point);
//    left_gripper1->setWorldTransform(tm_left1);

//    tm_right2 = btTransform(btQuaternion( 0,    0,    0 ,   1), user_mid_point);
//    right_gripper2->setWorldTransform(tm_right2);


    //plotting

    std::vector<btVector3> plotpoints;
    std::vector<btVector4> plotcols;
    plotpoints.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    plotpoints.push_back(btVector3(mid_x,max_y,node_pos[0][2]));
    plotcols.push_back(btVector4(1,0,0,1));

//    for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
//    {

//        cout << (*ii).first << ": " << (*ii).second << endl;
//        float r = (float)rand()/(float)RAND_MAX;
//        if(r < 0.5)
//        {
//            plotpoints.push_back(node_pos[(*ii).first]);
//            plotpoints.push_back(node_pos[(*ii).second]);
//            plotcols.push_back(btVector4(0,1,0,1));
//        }

//    }

    PlotLines::Ptr lines;
    lines.reset(new PlotLines(2));
    lines->setPoints(plotpoints,plotcols);
    env->add(lines);

    plot_points.reset(new PlotPoints(5));
    env->add(plot_points);

    rot_lines.reset(new PlotLines(2));
    rot_lines->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
    env->add(rot_lines);



    left_center_point.reset(new PlotPoints(10));

    btTransform left_tm = left_gripper1->getWorldTransform();
    cout << left_tm.getOrigin()[0] << " " << left_tm.getOrigin()[1] << " " << left_tm.getOrigin()[2] << " " <<endl;
    cout << mid_x << " " << min_y << " " << node_pos[0][2] <<endl;
    std::vector<btVector3> poinsfsefts2;
    //points2.push_back(left_tm.getOrigin());
    //points2.push_back(left_tm.getOrigin());
    std::vector<btVector4> plotcols2;
    plotcols2.push_back(btVector4(1,0,0,1));
    //plotcols2.push_back(btVector4(1,0,0,1));

    poinsfsefts2.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    //poinsfsefts2[0] = left_tm.getOrigin();
    //plotcols.push_back(btVector4(1,0,0,1));

    std::vector<btVector3> plotpoints2;
    plotpoints2.push_back( left_tm.getOrigin());
    //plotpoints2.push_back(btVector3(mid_x,max_y,node_pos[0][2]));


    env->add(left_center_point);
    //left_center_point->setPoints(plotpoints2);


    left_axes1.reset(new PlotAxes());
    left_axes1->setup(tm_left1,1);
    env->add(left_axes1);

    left_axes2.reset(new PlotAxes());
    left_axes2->setup(tm_left2,1);
    env->add(left_axes2);

//    drag_line.reset(new PlotLines(2));
//    env->add(drag_line);

    left_gripper1->toggle();
    right_gripper1->toggle();

    left_gripper1->toggleattach(clothptr->softBody.get());
    right_gripper1->toggleattach(clothptr->softBody.get());

    left_gripper2->toggle();
    right_gripper2->toggle();

    left_gripper2->toggleattach(clothptr->softBody.get());
    right_gripper2->toggleattach(clothptr->softBody.get());

//    btMatrix3x3 proj_mat;
//    proj_mat[0][0] =   ; proj_mat[0][1] =  ;  proj_mat[0][2] = ;
//    proj_mat[1][0] =   ; proj_mat[1][1] =  ;  proj_mat[1][2] = ;
//    proj_mat[2][0] =   ; proj_mat[2][1] =  ;  proj_mat[2][2] = ;

//    for(int i = 0; i<cloth_boundary_inds.size();i++)
//    {

//    }



    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    /*
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    */
    //ProfilerStart("profile.txt");
    startFixedTimestepLoop(dt);
    //ProfilerStop();
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    CustomScene().run();
    return 0;
}
