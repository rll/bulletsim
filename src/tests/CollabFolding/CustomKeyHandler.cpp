#include "CustomKeyHandler.h"

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
    case 'm': {
      btVector3 camera_pos(10.0,0.0,20.0);
      scene.checkNodeVisibility(camera_pos, scene.clothptr->softBody);
      break;
    }
    case 'z': {
      btVector3 camera_pos(10.0,0.0,20.0);
      std::vector<btVector3> points =
	scene.checkNodeVisibility(camera_pos, scene.clothptr->softBody);
      scene.savePoints(points, (1.0/(GeneralConfig::scale*0.5)));
      break;
    }
    case 'x': {
      scene.mergeGrippers(scene.clothptr->softBody);
      break;
    }

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

        case 'v': {
	  scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
	  break;
        }

        case 'f': {
	  scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
	  scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
	  break;
        }

        case 'g': {
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

        case 'j': {
#ifdef PROFILER
	  if(!scene.bTracking)
	    ProfilerStart("profile.txt");
	  else
	    ProfilerStop();
#endif
	  nodeArrayToNodePosVector(scene.clothptr->softBody->m_nodes, scene.prev_node_pos);
	  scene.bTracking = !scene.bTracking;
	  if(!scene.bTracking) // clear the points to plot : show no points
	    scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
	  break;
            }
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
                scene.inputState.lastX = ea.getXnormalized();
		scene.inputState.lastY = ea.getYnormalized();
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

		//FIXME: is this necessary with osg?
                btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); 
                btVector3 xVec = normal.cross(yVec);
                btVector3 dragVec = (SceneConfig::mouseDragScale*10
				     * (scene.inputState.dx*xVec + scene.inputState.dy*yVec));

                btTransform origTrans;
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0) {
		  scene.left_gripper1->getWorldTransform(origTrans);
                } else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1) {
		  scene.left_gripper2->getWorldTransform(origTrans);
                } else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2) {
		  scene.right_gripper1->getWorldTransform(origTrans);
                } else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3) {
		  scene.right_gripper2->getWorldTransform(origTrans);
                }

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


		// reflect the change in positions of the grabbers in the simulation.
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0) {
		  scene.left_gripper1->setWorldTransform(newTrans);
                #ifdef USE_PR2
		  btTransform TOR_newtrans = newTrans*TBullet_PR2GripperRight;
		  TOR_newtrans.setOrigin(newTrans.getOrigin());
		  scene.pr2m.pr2Right->moveByIK(TOR_newtrans,
						SceneConfig::enableRobotCollision, true);
                #endif
                } else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1) {
		  scene.left_gripper2->setWorldTransform(newTrans);
                #ifdef USE_PR2
		  btTransform TOR_newtrans = newTrans*TBullet_PR2GripperLeft;
		  TOR_newtrans.setOrigin(newTrans.getOrigin());
		  scene.pr2m.pr2Left->moveByIK(TOR_newtrans,
					       SceneConfig::enableRobotCollision, true);
                #endif
                }
                else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2) {
		  scene.right_gripper1->setWorldTransform(newTrans);
                }
                else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3) {
		  scene.right_gripper2->setWorldTransform(newTrans);
                }
                return true;
            }
        }
        break;
    }
    return false;
}
