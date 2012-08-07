#include "CustomScene.h"
#include "CustomKeyHandler.h"


void CustomScene::mergeGrippers(boost::shared_ptr<btSoftBody> psb,
				btScalar _close_thresh) {
  cout<<"Called merge grippers."<<endl;
  for(int i = 0; i < grippers.size(); i += 1) {
    for(int j = i+1; j < grippers.size(); j+=1) {
      if (areClose(grippers[i], grippers[j], _close_thresh)) {
	char ch;
	cout<<"Merge? "<<i<<" & "<<j;
	cin >> ch;
	cout <<ch<<endl;
	if (ch == 'y' || ch == 'Y') {
	  GripperKinematicObject::Ptr new_gripper1, new_gripper2;
	  GripperKinematicObject::Ptr old_gripper1(grippers[i]);
	  GripperKinematicObject::Ptr old_gripper2(grippers[j]);

	  new_gripper1.reset(new GripperKinematicObject());
		  
	  btTransform avg_transform = getAverageTransform(old_gripper1, old_gripper2);
	  new_gripper1->setWorldTransform(avg_transform);
	  avg_transform = new_gripper1->getWorldTransform();
	  
	  new_gripper2.reset(new GripperKinematicObject());
	  btTransform down_transform = btTransform(avg_transform.getRotation(),
						   getDownPoint(avg_transform.getOrigin(), psb, 1.0));
	  new_gripper2->setWorldTransform(down_transform);  
	  
	  old_gripper1->setWorldTransform(new_gripper1->getWorldTransform());
	  old_gripper1->toggle();
	  old_gripper1->attach_incremental(psb, 0);

	  old_gripper2->setWorldTransform(new_gripper2->getWorldTransform());
	  old_gripper2->toggle();
	  old_gripper2->attach_incremental(psb, 0);

						  
	}
      }
    }
  }
} 


/** Returns the average of the transformations of the gripper G1 and G2. */
btTransform CustomScene::getAverageTransform(GripperKinematicObject::Ptr &g1,
					     GripperKinematicObject::Ptr &g2) {
  btTransform t1 = g1->getWorldTransform();
  btTransform t2 = g2->getWorldTransform();
  btVector3 avg_translate = t1.getOrigin() + t2.getOrigin();
  avg_translate *= 0.5;
  return btTransform(t1.getRotation(), avg_translate);
}


/** Returns true iff, the || g1 - g2|| <= _THRESH.
    Where the distance is defined as: NORM of the location of origins
    of the two grippers. */
bool CustomScene::areClose(GripperKinematicObject::Ptr &g1,
			   GripperKinematicObject::Ptr &g2, btScalar _thresh) {
  btScalar dist = (g1->getWorldTransform().getOrigin()
		   - g2->getWorldTransform().getOrigin()).length();
  cout<<"\tDist: "<<dist<<" thresh: "<<_thresh<<endl;
  return (dist <= _thresh);
}

/** Returns the coordinates of the last point below (-z) SOURCE_PT
    on the cloth represented by PSB. */
btVector3 CustomScene::getDownPoint(btVector3 & source_pt, boost::shared_ptr<btSoftBody> psb,
				    btScalar radius) {
  int node_idx= -1;
  int min_z = 9999;

  for(int i = 0; i < psb->m_nodes.size(); i += 1) {
    btScalar rad_dist = getXYDistance(source_pt, psb->m_nodes[i].m_x);
    if (rad_dist < radius)
      if (psb->m_nodes[i].m_x.z() < min_z) {
	min_z = psb->m_nodes[i].m_x.z();
	node_idx   = i;
      }
    }
 return psb->m_nodes[node_idx].m_x;
}


/** Returns ||(v1.x, v1.y) - (v2.x, v2.y)||. */
btScalar inline CustomScene::getXYDistance(btVector3 &v1, btVector3 &v2) {
  btVector3 vec(v1.x() - v2.x(), v1.y() - v2.y(), 0.0);
  return vec.length();
}


/** Saves the points in the scenep.lotPoints to a file in PCL format. */
void CustomScene::savePoints(std::vector<btVector3> &points, btScalar scale,
			     std::string _fname) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width    = points.size();
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  scale *= 0.5;

  for (size_t i = 0; i < cloud.points.size(); i+=1) {
    cloud.points[i].x = scale * points[i].x();
    cloud.points[i].y = scale * points[i].y();
    cloud.points[i].z = scale * points[i].z();
  }

  stringstream ss;
  int rnd = rand()/1000;
  rnd /=100;
  ss << _fname <<rnd <<".pcd";
  pcl::io::savePCDFileASCII (ss.str(), cloud);
  cout << "Saved " << cloud.points.size ()
       << " data points to " << ss.str()<<"."<< endl;
}

/** Raycasts from SOURCE to all the nodes of PSB
    and returns a vector of the same size as the nodes of PSB
    depicting whether that node is visible or not. */
std::vector<btVector3> CustomScene::checkNodeVisibility(btVector3 camera_origin,
				      boost::shared_ptr<btSoftBody> psb) {

  plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

  std::vector<btVector3> plotpoints;
  std::vector<btVector4> color;

  plotpoints.push_back(camera_origin);
  color.push_back(btVector4(3,1,0,1));

  for (int i=0; i < psb->m_nodes.size(); i += 1) {
    btVector3 node_pos = psb->m_nodes[i].m_x;
    btSoftBody::sRayCast sol;
    bool b = psb->rayTest (camera_origin, node_pos, sol);

    if (sol.fraction >= 0.99) { // iff fraction -> 1 : means the ray can hit the node.
      plotpoints.push_back(node_pos);
      color.push_back(btVector4(2,0,0,1));
    }
  }
  plot_points->setPoints(plotpoints,color);
  return plotpoints;
}


/** Attaches GRIPPER_TO_ATTACH with the softbody and detaches
    GRIPPER_TO_DETACH from it.*/
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

 
/* Creates a square cloth with side length 2s.
   The four coordinates of the cloth are:
   {(s,s) (-s,s,) (-s,-s) (s,-s)}
   Then, the center of the cloth (initially at (0,0,0)
   is translated to CENTER. */
BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, 
					       const btVector3 &center) {
  const int divs = 45;
  btSoftBody *psb =
    btSoftBodyHelpers::CreatePatch(env->bullet->softBodyWorldInfo,
				   center + btVector3(-s,-s,0),
				   center + btVector3(+2*s,-s,0),
				   center + btVector3(-s,+s,0),
				   center + btVector3(+2*s,+s,0),
				   2*divs, divs,
				   0, true);

  psb->m_cfg.piterations = 10;//2;
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
    | btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::CL_SELF;

  psb->m_cfg.kDF = 1.0;
  psb->getCollisionShape()->setMargin(0.05);
  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
  psb->m_cfg.kDP = 0.1;//0.05;
  psb->generateBendingConstraints(2, pm);
  psb->randomizeConstraints();
  psb->setTotalMass(1, true);
  //psb->generateClusters(0);
  psb->generateClusters(500);
  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

/* Creates a square cloth with side length 2s.
   The four coordinates of the cloth are:
   {(s,s) (-s,s,) (-s,-s) (s,-s)}
   Then, the center of the cloth (initially at (0,0,0)
   is translated to CENTER. */
BulletSoftObject::Ptr CustomScene::createCloth(btScalar _w, btScalar _l, 
					       const btVector3 &center) {
  const int x_res = (int) (_l*70);
  const int y_res = (int) (_w*70);

  btScalar l = GeneralConfig::scale*_l*0.5;
  btScalar w = GeneralConfig::scale*_w*0.5;

  btSoftBody *psb =
    btSoftBodyHelpers::CreatePatch(env->bullet->softBodyWorldInfo,
				   center + btVector3(-l,-w,5),
				   center + btVector3(l,-w,5),
				   center + btVector3(-l,w,5),
				   center + btVector3(l,w,5),
				   x_res, y_res,
				   0, true);
  cout<<"DONE: Cloth"<<endl;
  
  psb->m_cfg.piterations = 10;//2;
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
    | btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::CL_SELF;

  psb->m_cfg.kDF = 1.0;
  psb->getCollisionShape()->setMargin(0.05);
  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
  psb->m_cfg.kDP = 0.1;//0.05;
  psb->generateBendingConstraints(2, pm);
  psb->randomizeConstraints();
  psb->setTotalMass(1, true);
  psb->generateClusters(0);
  //psb->generateClusters(500);
  return BulletSoftObject::Ptr(new BulletSoftObject(psb));  
}

/** Main loop which is responsible for jacobian tracking. */
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
    node_change = error;

    Eigen::MatrixXf Jpinv= pinv(J.transpose()*J)*J.transpose();
    V_trans = Jpinv*V_step;

    // Limit big motions to STEP_LIMIT
    if(V_trans.norm() > step_limit)
      V_trans = V_trans/V_trans.norm()*step_limit;

    // Don't move if the movements are too small.
    if(V_trans.norm() < 0.01) {
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
    transtm1 = btTransform(btQuaternion(0,0,0,1),
			   btVector3(V_trans(0),
				     V_trans(1),
				     V_trans(2)));
    if(num_auto_grippers > 1)
      transtm2 = btTransform(btQuaternion(0,0,0,1),
			     btVector3(V_trans(3),
				       V_trans(4),
				       V_trans(5)));
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
      pr2m.pr2Left->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision,true);
#endif
    }
    break;
  }
  loopState.skip_step = false;
  bInTrackingLoop = false;
}


/** Simulates the scene in a new fork.*/
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


/** Finds the distance from a node corresponding to 
    INPUT_IND on the cloth to the closest node attached to the gripper.*/
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



/** Computes an approximation to the Jacobian of the cloth's
    node positions wrt to the robot grippers, by using an 
    scaling factor which decays exponentially with increasing
    distance from the gripper. */
Eigen::MatrixXf CustomScene::computeJacobian_approx() {
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


/** Very similar to computeJacobian except is parallelized
    (using openMP).*/
Eigen::MatrixXf CustomScene::computeJacobian_parallel() {
  boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

  //printf("starting jacobian computation\n");
  //stopLoop();
  bool bBackupLoopState = loopState.skip_step;
  loopState.skip_step = true;

  int numnodes = clothptr->softBody->m_nodes.size();
  Eigen::VectorXf  V_before(numnodes*3);


  for(int k = 0; k < numnodes; k++) {
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
    for(int i = 0; i < perts.size(); i += 1)  {
      btTransform dummy_tm;
      StepState innerstate;
      simulateInNewFork(innerstate, jacobian_sim_time, perts[i],dummy_tm);

      Eigen::VectorXf  V_after(V_before);
      for(int k = 0; k < numnodes; k+=1) {
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

  boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
  //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

  loopState.skip_step = bBackupLoopState;
  return J;
}


/** Simulates the cloth in bullet for JACOBIAN_SIM_TIME
    and takes the difference in the positions of the node and
    divides by JACOBIAN_SIM_TIME.*/
Eigen::MatrixXf CustomScene::computeJacobian() {
  boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

  //printf("starting jacobian computation\n");
  //stopLoop();
  bool bBackupLoopState = loopState.skip_step;
  loopState.skip_step = true;

  int numnodes = clothptr->softBody->m_nodes.size();
  Eigen::VectorXf  V_before(numnodes*3);
  Eigen::VectorXf  V_after(V_before);

  for(int k = 0; k < numnodes; k+=1) {
    for(int j = 0; j < 3; j+=1)
      V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
  }

  Eigen::MatrixXf J(numnodes*3,3);

  std::vector<btTransform> perts;
  float step_length = 0.2;
  perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
  perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
  perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
  float time;

  for(int i = 0; i < 3 ; i+=1) {
    createFork();
    swapFork(); //now pointers are set to the forked objects

    //apply perturbation
    left_gripper1->applyTransform(perts[i]);

    time = jacobian_sim_time;
    while (time > 0) {
      float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;
      if (syncTime && drawingOn)
	endTime = viewer.getFrameStamp()->getSimulationTime();

      fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps,
		      BulletConfig::internalTimeStep);
      draw();

      if (syncTime && drawingOn) {
	float timeLeft = BulletConfig::dt - (endTime - startTime);
	idleFor(timeLeft);
	startTime = endTime + timeLeft;
      }


      time -= BulletConfig::dt;

    }

    for(int k = 0; k < numnodes; k++) {
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

/** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
void CustomScene::drawAxes() {
  left_axes1->setup(left_gripper1->getWorldTransform(),1);
  left_axes2->setup(left_gripper2->getWorldTransform(),1);
}

/** Swaps the forked objects with the ones in the main loop. */
void CustomScene::swapFork() {
#ifdef USE_PR2
  // swaps the forked robot with the real one
  cout << "Swapping forks." << endl;
  int leftidx = pr2m.pr2Left->index;
  int rightidx = pr2m.pr2Right->index;
  origRobot.swap(tmpRobot);
  pr2m.pr2 = origRobot;
  pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
  pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);
#endif
  if(left_gripper1.get() == left_gripper1_orig.get()) {
    left_gripper1 = left_gripper1_fork;
    right_gripper1 = right_gripper1_fork;
    clothptr = clothptr_fork;
  } else {
    left_gripper1 = left_gripper1_orig;
    right_gripper1 = right_gripper1_orig;
    clothptr = clothptr_orig;
  }
}


void CustomScene::createFork() {
  if(fork)
    destroyFork();

  bullet2.reset(new BulletInstance);
  bullet2->setGravity(BulletConfig::gravity);
  osg2.reset(new OSGInstance);
  osg->root->addChild(osg2->root.get());
  fork.reset(new Fork(env, bullet2, osg2));
  registerFork(fork);

#ifdef USE_PR2
  origRobot = pr2m.pr2;
  EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
  if (!p) {
    cout << "Failed to get forked version of robot!" << endl;
    return;
  }
  tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
  cout << (tmpRobot->getEnvironment() == env.get()) << endl;
  cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
#endif
  left_gripper1_fork =
    boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(left_gripper1));
  right_gripper1_fork =
    boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(right_gripper1));
  clothptr_fork = boost::static_pointer_cast<BulletSoftObject> (fork->forkOf(clothptr));
}


void CustomScene::destroyFork() {
  if(left_gripper1.get() == left_gripper1_fork.get()) {
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

/* Sets up the scene and UI even handlers,
   initializes various structures.*/
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
  /* ORIGINAL CODE*/
  cout<<"SCALE: "<<GeneralConfig::scale<<endl;
  /**BulletSoftObject::Ptr 
    cloth(createCloth(GeneralConfig::scale * 0.25,
    GeneralConfig::scale * btVector3(0.7, 0, table_height+0.01)));**/
  
  BulletSoftObject::Ptr 
    cloth(createCloth(0.46,0.92, btVector3(0.0, 0.0, 0.0)));


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

#ifdef USE_PR2
  leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, 
						"l_gripper_l_finger_tip_link", 
						"l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(psb);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, 
						   "r_gripper_l_finger_tip_link", 
						   "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(psb);
#endif

  int min_x_ind = -1;
  int max_x_ind = -1;
  int min_y_ind = -1;
  int max_y_ind = -1;
  double min_x = 100;
  double max_x = -100;
  double min_y = 100;
  double max_y = -100;

  std::vector<btVector3> node_pos(psb->m_nodes.size());
  std::vector<int> corner_ind(4,-1);
  std::vector<btVector3> corner_pnts(4);

  corner_pnts[0] = btVector3(100,100,0);
  corner_pnts[1] = btVector3(100,-100,0);
  corner_pnts[2] = btVector3(-100,100,0);
  corner_pnts[3] = btVector3(-100,-100,0);

  for(int i = 0; i < psb->m_nodes.size();i+=1) {
    node_pos[i] = psb->m_nodes[i].m_x;

    if(node_pos[i][0] <= corner_pnts[0][0]
       && node_pos[i][1] <= corner_pnts[0][1]) {
      corner_ind[0] = i;
      corner_pnts[0] = node_pos[i];
    }

    if(node_pos[i][0] <= corner_pnts[1][0]
       && node_pos[i][1] >= corner_pnts[1][1]) {
      corner_ind[1] = i;
      corner_pnts[1] = node_pos[i];
    }

    if(node_pos[i][0] >= corner_pnts[2][0]
       && node_pos[i][1] <= corner_pnts[2][1]) {
      corner_ind[2] = i;
      corner_pnts[2] = node_pos[i];
    }

    if(node_pos[i][0] >= corner_pnts[3][0]
       && node_pos[i][1] >= corner_pnts[3][1]) {
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


  btTransform tm_right1 = btTransform(btQuaternion(0,0,0,1),
				      corner_pnts[2] 
				      + btVector3(-right_gripper1->children[0]->halfExtents[0],
						  right_gripper1->children[0]->halfExtents[1],0));
  right_gripper1->setWorldTransform(tm_right1);

  btTransform tm_left2 = btTransform(btQuaternion(0,0,0,1),
				     corner_pnts[1]
				     + btVector3(left_gripper2->children[0]->halfExtents[0],
						 -left_gripper2->children[0]->halfExtents[1],0));
  left_gripper2->setWorldTransform(tm_left2);
  //left_gripper1->toggle();


  btTransform tm_right2 = btTransform(btQuaternion(0,0,0,1),
				      corner_pnts[3]
				      + btVector3(-right_gripper2->children[0]->halfExtents[0],
						  -right_gripper2->children[0]->halfExtents[1],0));
  right_gripper2->setWorldTransform(tm_right2);


  // Calculate node disntaces to the corners where the auto gripper are.
  gripper_node_distance_map.resize(num_auto_grippers);
  for(int i = 0; i < num_auto_grippers; i++) {
    gripper_node_distance_map[i].resize(node_pos.size());
    for(int j = 0; j < node_pos.size(); j++)
      gripper_node_distance_map[i][j] = (corner_pnts[i]-node_pos[j]).length();   
  }

  // Calculate internode distance.
  cloth_distance_matrix = Eigen::MatrixXf( node_pos.size(), node_pos.size());
  for(int i = 0; i < node_pos.size(); i++) {
    for(int j = i; j < node_pos.size(); j++) {
      cloth_distance_matrix(i,j) = (node_pos[i]-node_pos[j]).length();
      cloth_distance_matrix(j,i) = cloth_distance_matrix(i,j);
    }
  }

  /** Mirror about centerline along y direction

               ______|______ max y
              |      |      |
      y HUMAN |   CLO|TH    |  ROBOT
      ^       |      |      |
      |       |______|______|
      --> x   max x  |     min x, min y

      Centerline defined by 2 points. */
  float mid_x = (max_x + min_x)/2;
  point_reflector.reset(new PointReflector(mid_x, min_y, max_y));
  //find node that most closely matches reflection of point
  for(int i = 0; i < node_pos.size(); i++) {
    if(node_pos[i][0] < mid_x) {//look at points in left half
      btVector3 new_vec = point_reflector->reflect(node_pos[i]);
      float closest_dist = 100000;
      int closest_ind = -1;
      for(int j = 0; j < node_pos.size(); j++) {
	float dist = (node_pos[j]-new_vec).length();
	if(dist < closest_dist) {
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
  /**Initialize with large values, as we need to find
     the minimum distance by doing a linear search. */
  double user_length  = 1000; 
  double robot_length = 1000;
  for(int i = 0; i < node_pos.size(); i++) {
    double this_user_length = (node_pos[i]-user_target_mid_point).length();
    if( this_user_length < user_length) {
      user_length = this_user_length;
      user_mid_point = node_pos[i];
      user_mid_point_ind = i;
    }
    double this_robot_length = (node_pos[i]-robot_target_mid_point).length();
    if( this_robot_length < robot_length) {
      robot_length = this_robot_length;
      robot_mid_point = node_pos[i];
      robot_mid_point_ind = i;
    }
  }
  cout << "robot side midpoint: " << robot_mid_point[0] << " "
       << robot_mid_point[1] << " " << robot_mid_point[2] << endl;
  cout << "user side midpoint: " << user_mid_point[0] << " "
       << user_mid_point[1] << " " << user_mid_point[2] << endl;

  //plotting

  std::vector<btVector3> plotpoints;
  std::vector<btVector4> plotcols;
  plotpoints.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
  plotpoints.push_back(btVector3(mid_x,max_y,node_pos[0][2]));
  plotcols.push_back(btVector4(1,0,0,1));

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
  cout << left_tm.getOrigin()[0] << " " << left_tm.getOrigin()[1]
       << " " << left_tm.getOrigin()[2] << " " <<endl;
  cout << mid_x << " " << min_y << " " << node_pos[0][2] <<endl;

  std::vector<btVector3> poinsfsefts2;
  std::vector<btVector4> plotcols2;
  plotcols2.push_back(btVector4(1,0,0,1));

  poinsfsefts2.push_back(btVector3(mid_x,min_y,node_pos[0][2]));

  std::vector<btVector3> plotpoints2;
  plotpoints2.push_back( left_tm.getOrigin());

  env->add(left_center_point);

  left_axes1.reset(new PlotAxes());
  left_axes1->setup(tm_left1,1);
  env->add(left_axes1);

  left_axes2.reset(new PlotAxes());
  left_axes2->setup(tm_left2,1);
  env->add(left_axes2);

  left_gripper1->toggle();
  right_gripper1->toggle();

  left_gripper1->toggleattach(clothptr->softBody.get());
  right_gripper1->toggleattach(clothptr->softBody.get());

  left_gripper2->toggle();
  right_gripper2->toggle();

  left_gripper2->toggleattach(clothptr->softBody.get());
  right_gripper2->toggleattach(clothptr->softBody.get());

  //setSyncTime(true);
  startViewer();
  stepFor(dt, 2);

  //ProfilerStart("profile.txt");
  startFixedTimestepLoop(dt);
  //ProfilerStop();
}


/** Constructor. */
#ifdef USE_PR2
 CustomScene::CustomScene() : pr2m(*this) {
#else
  CustomScene::CustomScene() {
#endif
    bTracking = bInTrackingLoop = false;
      
    inputState.transGrabber0 =  inputState.rotateGrabber0 =
      inputState.transGrabber1 =  inputState.rotateGrabber1 =
      inputState.transGrabber2 =  inputState.rotateGrabber2 =
      inputState.transGrabber3 =  inputState.rotateGrabber3 =
      inputState.startDragging = false;

    jacobian_sim_time = 0.05;


    left_gripper1_orig.reset(new GripperKinematicObject());

    left_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						      btVector3(0,-10,0))); 
    env->add(left_gripper1_orig);


    btVector4 color(0.6,0.6,0.6,1);
    right_gripper1_orig.reset(new GripperKinematicObject(color));
    right_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						       btVector3(0,10,0)));
    env->add(right_gripper1_orig);
 

    left_gripper1 = left_gripper1_orig;
    right_gripper1 = right_gripper1_orig;
      

    left_gripper2.reset(new GripperKinematicObject());
    left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						 btVector3(0,20,0)));
    env->add(left_gripper2);


    right_gripper2.reset(new GripperKinematicObject(color));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
						  btVector3(0,-20,0)));
    env->add(right_gripper2);

    grippers.push_back(left_gripper1);
    grippers.push_back(left_gripper2);
    grippers.push_back(right_gripper1);
    grippers.push_back(right_gripper2);

    num_auto_grippers = 2;      
    fork.reset();
}
