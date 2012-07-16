#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "convexdecomp.h"
#include "utils/config.h"
#include "bullet_io.h"
#include "utils/logging.h"
#include <set>

using namespace OpenRAVE;
using namespace std;
using boost::shared_ptr;

btVector3 computeCentroid(const KinBody::Link::TRIMESH& mesh) {
	btVector3 sum(0, 0, 0);
	BOOST_FOREACH(const RaveVector<double>& v, mesh.vertices) {
		sum += util::toBtVector(v);
	}
	//printf("COMPUTECENTROIDOFTRIMESHFUNCTION: %f, %f, %f\n", (sum / mesh.vertices.size()).getX(), (sum / mesh.vertices.size()).getY(), (sum / mesh.vertices.size()).getZ());
	return GeneralConfig::scale * sum / mesh.vertices.size();
}

btTransform centroidTransform(const std::list<KinBody::Link::GEOMPROPERTIES> &geometries){

	btScalar totalMass = 0;
	btTransform relativeTransform;
	relativeTransform.setIdentity();

	//calculate total mass of KinBody
	for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin(); geom != geometries.end(); ++geom) {

		btVector3 boxExtents;
		btScalar sphereRadius;
		btScalar cylinderRadius;
		btScalar cylinderHeight;

		btScalar subshapeMass;
		btTransform subshapeTransform;

		btVector3 meshOffset(0,0,0);

		const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();

		switch (geom->GetType()) {
			case KinBody::Link::GEOMPROPERTIES::GeomBox:
				boxExtents = util::toBtVector(geom->GetBoxExtents());
				subshapeMass = 8.0 * boxExtents.getX()* boxExtents.getY() * boxExtents.getZ();
				break;
			case KinBody::Link::GEOMPROPERTIES::GeomSphere:
				sphereRadius = geom->GetSphereRadius();
				subshapeMass = 4.0/3.0 * 3.14159265358979323 * sphereRadius * sphereRadius * sphereRadius;
				break;
			case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
				cylinderRadius = geom->GetCylinderRadius();
				cylinderHeight = geom->GetCylinderHeight();
				subshapeMass = 3.14159265358979323 * cylinderRadius * cylinderRadius * cylinderHeight;
				break;
			case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
				subshapeMass = 1;
				meshOffset = computeCentroid(mesh);
				printf("subshapeCentroid(MESH) Translation: %f, %f, %f\n", meshOffset.getX(), meshOffset.getY(), meshOffset.getZ());
				break;
			default:
				subshapeMass = 0;
				break;
		}
		subshapeMass *= GeneralConfig::scale * GeneralConfig::scale * GeneralConfig::scale;
		totalMass += subshapeMass;

		subshapeTransform = util::toBtTransform(geom->GetTransform(),GeneralConfig::scale * subshapeMass);
		subshapeTransform.setOrigin(subshapeTransform.getOrigin() + meshOffset * subshapeMass);

		relativeTransform =  relativeTransform * subshapeTransform;


		printf("translation of subshape: %f, %f, %f //mass of subshape: %f\n",
				relativeTransform.getOrigin().getX(), relativeTransform.getOrigin().getY(), relativeTransform.getOrigin().getZ(), subshapeMass);
		//printf("geomGetTransform Rotation: %f, %f, %f, %f\n",
		//		geom->GetTransform().rot.w, geom->GetTransform().rot.x, geom->GetTransform().rot.y, geom->GetTransform().rot.z);

	}
	printf("KinBody totalMass: %f\n", totalMass);
	relativeTransform.setOrigin(relativeTransform.getOrigin() / totalMass);
	return relativeTransform;
}

btScalar computeVolume(const KinBody::Link::TRIMESH& mesh) {
	return btScalar(1);
}

RaveInstance::RaveInstance() {
	isRoot = true;
	RaveInitialize(true);
	env = RaveCreateEnvironment();
	if (GeneralConfig::verbose  <= log4cplus::DEBUG_LOG_LEVEL)
		RaveSetDebugLevel(Level_Debug);
}

RaveInstance::RaveInstance(const RaveInstance &o, int cloneOpts) {
	isRoot = false;
	env = o.env->CloneSelf(cloneOpts);
}

RaveInstance::~RaveInstance() {
    env->Destroy();
	if (isRoot)
		RaveDestroy();
}

void Load(Environment::Ptr env, RaveInstance::Ptr rave, const string& filename,
		bool dynamicRobots) {
	bool success = rave->env->Load(filename);
	if (!success)
		throw runtime_error(
				(boost::format("couldn't load %s!\n") % (filename)).str());

	std::set<string> bodiesAlreadyLoaded;
	BOOST_FOREACH(EnvironmentObject::Ptr obj, env->objects) {
		RaveObject* robj = dynamic_cast<RaveObject*>(obj.get());
		if (robj) bodiesAlreadyLoaded.insert(robj->body->GetName());
	}

	std::vector<boost::shared_ptr<OpenRAVE::KinBody> > bodies;
	rave->env->GetBodies(bodies);
	BOOST_FOREACH(OpenRAVE::KinBodyPtr body, bodies) {
		if (bodiesAlreadyLoaded.find(body->GetName()) == bodiesAlreadyLoaded.end()) {
			if (body->IsRobot()) env->add(RaveRobotObject::Ptr(new RaveRobotObject(rave, boost::dynamic_pointer_cast<RobotBase>(body), btTransform::getIdentity(), CONVEX_HULL, dynamicRobots)));
			else {
				cout << "loading " << body->GetName() << endl;;
				env->add(RaveObject::Ptr(new RaveObject(rave, body, btTransform::getIdentity(), CONVEX_HULL, true)));
			}
		}
	}

}

RaveObject::RaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_,
        const btTransform &initTrans,
		TrimeshMode trimeshMode, bool isDynamic) {
	initRaveObject(rave_, body_, initTrans, trimeshMode, .0005 * METERS, isDynamic);
}

void RaveObject::init() {
	CompoundObject<BulletObject>::init();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	BOOST_FOREACH( map_t::value_type &joint_cnt, jointMap )
{	cout << joint_cnt.first->GetName() << endl;
	getEnvironment()->addConstraint(joint_cnt.second);
}
}

void RaveObject::destroy() {
	CompoundObject<BulletObject>::init();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	map_t mmap;
	BOOST_FOREACH( map_t::value_type &joint_cnt, mmap )
{	getEnvironment()->removeConstraint(joint_cnt.second);
}

}

static BulletObject::Ptr createFromLink(KinBody::LinkPtr link,
        std::vector<boost::shared_ptr<btCollisionShape> >& subshapes,
        std::vector<boost::shared_ptr<btStridingMeshInterface> >& meshes,
        const btTransform &initTransform, TrimeshMode trimeshMode,
        float fmargin, bool isDynamic) {

	btVector3 offset(0, 0, 0);
	btVector3 meshOffset;
	btTransform relativeTransform;


	//specified kinbody translation
	btTransform linkGetTransform = util::toBtTransform(link->GetTransform(), GeneralConfig::scale);
	btTransform childTransform =  initTransform * linkGetTransform;

	printf("===========initTransform Translation: %f, %f, %f===========\n", initTransform.getOrigin().getX(), initTransform.getOrigin().getY(), initTransform.getOrigin().getZ());
	//printf("linkGetTransform: %f, %f, %f\n", initTransform.getOrigin().getX(), initTransform.getOrigin().getY(), initTransform.getOrigin().getZ());
	//printf("linkGetRotation: %f, %f, %f, %f\n", initTransform.getRotation().getW(), initTransform.getRotation().getX(), initTransform.getRotation().getY(), initTransform.getRotation().getZ());

	const std::list<KinBody::Link::GEOMPROPERTIES> &geometries = link->GetGeometries();
	// sometimes the OpenRAVE link might not even have any geometry data associated with it
	// (this is the case with the PR2 model). therefore just add an empty BulletObject
	// pointer so we know to skip it in the future
	if (geometries.empty()) {
		return BulletObject::Ptr();
	}

	// each link is a compound of several btCollisionShapes
	btCompoundShape *compound = new btCompoundShape();
	compound->setMargin(fmargin);

	relativeTransform = centroidTransform(geometries);

	printf("relativeCentroidTranslation: %f, %f, %f\n", relativeTransform.getOrigin().getX(), relativeTransform.getOrigin().getY(), relativeTransform.getOrigin().getZ());
	//printf("relativeCentroidRotation: %f, %f, %f, %f\n", relativeTransform.getRotation().getW(), relativeTransform.getRotation().getX(), relativeTransform.getRotation().getY(), relativeTransform.getRotation().getZ());

	//Put toghether the KinBody
	for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin(); geom != geometries.end(); ++geom) {

		boost::shared_ptr<btCollisionShape> subshape;

		const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();
		btBoxShape * boxShape;

		switch (geom->GetType()) {
		case KinBody::Link::GEOMPROPERTIES::GeomBox:
			boxShape = new btBoxShape(util::toBtVector(GeneralConfig::scale* geom->GetBoxExtents()));
			subshape.reset(boxShape);
			meshOffset.setZero();
			break;
		case KinBody::Link::GEOMPROPERTIES::GeomSphere:
			subshape.reset(new btSphereShape(GeneralConfig::scale * geom->GetSphereRadius()));
			meshOffset.setZero();
			break;
		case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
			// cylinder axis aligned to Y
			subshape.reset(new btCylinderShapeZ(btVector3(GeneralConfig::scale
					* geom->GetCylinderRadius(), GeneralConfig::scale
					* geom->GetCylinderRadius(), GeneralConfig::scale
					* geom->GetCylinderHeight() / 2.)));
			meshOffset.setZero();
			break;
		case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
			meshOffset = computeCentroid(mesh);
			if (mesh.indices.size() < 3)
				break;
			if (trimeshMode == CONVEX_DECOMP) {
				printf("running convex decomposition\n");
				ConvexDecomp decomp(fmargin);
				for (size_t i = 0; i < mesh.vertices.size(); ++i)
					decomp.addPoint(util::toBtVector(mesh.vertices[i]));
				for (size_t i = 0; i < mesh.indices.size(); i += 3)
					decomp.addTriangle(mesh.indices[i], mesh.indices[i + 1],
							mesh.indices[i + 2]);
				subshape = decomp.run(subshapes); // use subshapes to just store smart pointer

			} else {

				//offset = computeCentroid(mesh) * METERS * 0;

				btTriangleMesh* ptrimesh = new btTriangleMesh();
				// for some reason adding indices makes everything crash
				/*
				 printf("-----------\n");
				 for (int z = 0; z < mesh.indices.size(); ++z)
				 printf("%d\n", mesh.indices[z]);
				 printf("-----------\n");*/

				for (size_t i = 0; i < mesh.indices.size(); i += 3){
					ptrimesh->addTriangle(util::toBtVector(GeneralConfig::scale	* mesh.vertices[i]) - relativeTransform.getOrigin(),
							util::toBtVector(GeneralConfig::scale * mesh.vertices[i + 1]) - relativeTransform.getOrigin(),
							util::toBtVector(GeneralConfig::scale * mesh.vertices[i + 2])- relativeTransform.getOrigin());
				}
				// store the trimesh somewhere so it doesn't get deallocated by the smart pointer
				meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));

				if (trimeshMode == CONVEX_HULL) {
					boost::shared_ptr<btConvexShape> pconvexbuilder(new btConvexTriangleMeshShape(ptrimesh));
					pconvexbuilder->setMargin(fmargin);

					//Create a hull shape to approximate Trimesh
					boost::shared_ptr<btShapeHull> hull(new btShapeHull( pconvexbuilder.get()));
					hull->buildHull(fmargin);

					btConvexHullShape *convexShape = new btConvexHullShape();
					for (int i = 0; i < hull->numVertices(); ++i){
						convexShape->addPoint(hull->getVertexPointer()[i]);// + relativeTransform.getOrigin());
					}
					subshape.reset(convexShape);

				} else { // RAW
					subshape.reset(new btBvhTriangleMeshShape(ptrimesh, true));
				}
			}
			break;

		default:
			break;
		}
		if (!subshape) {
			cout << "did not create geom type %d\n", geom->GetType();
			continue;
		}

		// store the subshape somewhere so it doesn't get deallocated by the smart pointer
		subshapes.push_back(subshape);
		subshape->setMargin(fmargin);

		btTransform subshapeTransform = util::toBtTransform(geom->GetTransform(),GeneralConfig::scale);
		subshapeTransform = subshapeTransform * relativeTransform.inverse();
		subshapeTransform.setOrigin(subshapeTransform.getOrigin() + meshOffset);
		printf("subshape new translation: %f, %f, %f\n", subshapeTransform.getOrigin().getX(), subshapeTransform.getOrigin().getY(), subshapeTransform.getOrigin().getZ());


		compound->addChildShape(subshapeTransform, subshape.get());
	}

	Transform link_t = link->GetTransform();
	printf("shape initial translation: %f, %f, %f\n", childTransform.getOrigin().getX(), childTransform.getOrigin().getY(), childTransform.getOrigin().getZ());


	childTransform = childTransform * relativeTransform;
	float mass = isDynamic ? link->GetMass() : 0;

	printf("shape center of mass: %f, %f, %f\n", childTransform.getOrigin().getX(), childTransform.getOrigin().getY(), childTransform.getOrigin().getZ());
	BulletObject::Ptr child(new BulletObject(mass, compound, childTransform, !isDynamic));
	return child;

}

BulletConstraint::Ptr createFromJoint(KinBody::JointPtr joint, std::map<
		KinBody::LinkPtr, BulletObject::Ptr> linkMap) {

	KinBody::LinkPtr joint1 = joint->GetFirstAttached();
	KinBody::LinkPtr joint2 = joint->GetSecondAttached();

	if (!joint1 || !joint2 || !linkMap[joint1] || !linkMap[joint2])
		return BulletConstraint::Ptr();

	cout << joint->GetName() << endl;

	btRigidBody* body0 = linkMap[joint->GetFirstAttached()]->rigidBody.get();
	btRigidBody* body1 = linkMap[joint->GetSecondAttached()]->rigidBody.get();

	Transform t0inv = (joint)->GetFirstAttached()->GetTransform().inverse();
	Transform t1inv = (joint)->GetSecondAttached()->GetTransform().inverse();
	btTypedConstraint* cnt;

	switch ((joint)->GetType()) {
	case KinBody::Joint::JointHinge: {
		btVector3 pivotInA = util::toBtVector(t0inv * (joint)->GetAnchor());
		btVector3 pivotInB = util::toBtVector(t1inv * (joint)->GetAnchor());
		btVector3 axisInA = util::toBtVector(t0inv.rotate((joint)->GetAxis(0)));
		btVector3 axisInB = util::toBtVector(t1inv.rotate((joint)->GetAxis(0)));
		btHingeConstraint* hinge = new btHingeConstraint(*body0, *body1,
				pivotInA, pivotInB, axisInA, axisInB);
		if (!(joint)->IsCircular(0)) {
			vector<dReal> vlower, vupper;
			(joint)->GetLimits(vlower, vupper);
			btScalar orInitialAngle = (joint)->GetValue(0);
			btScalar btInitialAngle = hinge->getHingeAngle();
			btScalar lower_adj, upper_adj;
			btScalar diff = (btInitialAngle + orInitialAngle);
			lower_adj = diff - vupper.at(0);
			upper_adj = diff - vlower.at(0);
			hinge->setLimit(lower_adj, upper_adj);
		}
		cnt = hinge;
		cout << "hinge" << endl;
		break;
	}
	case KinBody::Joint::JointSlider: {
		Transform tslider;
		tslider.rot = quatRotateDirection(Vector(1, 0, 0), (joint)->GetAxis(0));
		btTransform frameInA = util::toBtTransform(t0inv * tslider);
		btTransform frameInB = util::toBtTransform(t1inv * tslider);
		cnt = new btSliderConstraint(*body0, *body1, frameInA, frameInB, true);
		cout << "slider" << endl;
		break;
	}
	case KinBody::Joint::JointUniversal:
		RAVELOG_ERROR ( "universal joint not supported by bullet\n");
		break;
		case KinBody::Joint::JointHinge2:
		RAVELOG_ERROR("hinge2 joint not supported by bullet\n");
		break;
		default:
		cout << boost::format("unknown joint type %d\n")%joint->GetType();
		break;
	}
	return BulletConstraint::Ptr(new BulletConstraint(cnt, true));
}

void RaveObject::initRaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_,
        const btTransform &initTrans_,
		TrimeshMode trimeshMode, float fmargin, bool isDynamic) {
	rave = rave_;
	body = body_;
    initTrans = initTrans_;

	const std::vector<KinBody::LinkPtr> &links = body->GetLinks();
	getChildren().reserve(links.size());
	// iterate through each link in the robot (to be stored in the children vector)
	BOOST_FOREACH(KinBody::LinkPtr link, links) {
		BulletObject::Ptr child = createFromLink(link, subshapes, meshes, initTrans, trimeshMode, fmargin, isDynamic && !link->IsStatic());
		getChildren().push_back(child);

		linkMap[link] = child;
		childPosMap[child] = getChildren().size() - 1;
		if (child) {
			collisionObjMap[child->rigidBody.get()] = link;
		// since the joints are always in contact, we should ignore their collisions
		// when setting joint positions (OpenRAVE should take care of them anyway)
			ignoreCollisionWith(child->rigidBody.get());
	}
}

if (isDynamic) {
	vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(body->GetJoints().size()+body->GetPassiveJoints().size());
	vbodyjoints.insert(vbodyjoints.end(),body->GetJoints().begin(),body->GetJoints().end());
	vbodyjoints.insert(vbodyjoints.end(),body->GetPassiveJoints().begin(),body->GetPassiveJoints().end());
	BOOST_FOREACH(KinBody::JointPtr joint, vbodyjoints) {
		BulletConstraint::Ptr constraint = createFromJoint(joint, linkMap);
		if (constraint) {
			// todo: put this in init:
			// getEnvironment()->bullet->dynamicsWorld->addConstraint(constraint->cnt, bIgnoreCollision);
			jointMap[joint] = constraint;
		}
	}
}
}

bool RaveObject::detectCollisions() {
	getEnvironment()->bullet->dynamicsWorld->updateAabbs();

	BulletInstance::CollisionObjectSet objs;
	for (int i = 0; i < getChildren().size(); ++i) {
		BulletObject::Ptr child = getChildren()[i];
		if (!child)
			continue;
		objs.clear();
		getEnvironment()->bullet->contactTest(child->rigidBody.get(), objs,
				&ignoreCollisionObjs);
		if (!objs.empty()) {
			// collision!
			return true;
		}
	}
	return false;
}

void RaveRobotObject::setDOFValues(const vector<int> &indices, const vector<
		dReal> &vals) {
	// update openrave structure
	{
		EnvironmentMutex::scoped_lock lock(rave->env->GetMutex());
		robot->SetActiveDOFs(indices);
		robot->SetActiveDOFValues(vals);
		rave->env->UpdatePublishedBodies();
	}
	updateBullet();
}

void RaveObject::updateBullet() {
	// update bullet structures
	// we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
	// which are easy to feed into Bullet
	vector<OpenRAVE::Transform> transforms;
	body->GetLinkTransformations(transforms);
	BOOST_ASSERT(transforms.size() == getChildren().size());
	for (int i = 0; i < getChildren().size(); ++i) {
		BulletObject::Ptr c = getChildren()[i];
		if (!c)
			continue;
		c->motionState->setKinematicPos(initTrans * util::toBtTransform(transforms[i],
				GeneralConfig::scale));
	}

}

vector<double> RaveRobotObject::getDOFValues(const vector<int>& indices) {
	robot->SetActiveDOFs(indices);
	vector<double> out;
	robot->GetActiveDOFValues(out);
	return out;
}

EnvironmentObject::Ptr RaveObject::copy(Fork &f) const {
	Ptr o(new RaveObject());

	internalCopy(o, f); // copies all children

    o->initTrans = initTrans;
	o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));

	// now we need to set up mappings in the copied robot
	for (std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i =
			linkMap.begin(); i != linkMap.end(); ++i) {
		const KinBody::LinkPtr raveObj = o->rave->env->GetKinBody(
				i->first->GetParent()->GetName())->GetLink(i->first->GetName());

		const int j = childPosMap.find(i->second)->second;
		const BulletObject::Ptr bulletObj = o->getChildren()[j];

		o->linkMap.insert(std::make_pair(raveObj, bulletObj));
		o->collisionObjMap.insert(std::make_pair(bulletObj->rigidBody.get(),
				raveObj));
	}

	for (std::map<BulletObject::Ptr, int>::const_iterator i =
			childPosMap.begin(); i != childPosMap.end(); ++i) {
		const int j = childPosMap.find(i->first)->second;
		o->childPosMap.insert(std::make_pair(o->getChildren()[j], i->second));
	}

	o->body = o->rave->env->GetKinBody(body->GetName());

	return o;
}

EnvironmentObject::Ptr RaveRobotObject::copy(Fork &f) const {

	Ptr o(new RaveRobotObject());

	/////////////////////// duplicated from RaveObject::copy //////////////////
	internalCopy(o, f); // copies all children

	o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));

	// now we need to set up mappings in the copied robot
	for (std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i =
			linkMap.begin(); i != linkMap.end(); ++i) {
		const KinBody::LinkPtr raveObj = o->rave->env->GetKinBody(
				i->first->GetParent()->GetName())->GetLink(i->first->GetName());

		const int j = childPosMap.find(i->second)->second;
		const BulletObject::Ptr bulletObj = o->getChildren()[j];

		o->linkMap.insert(std::make_pair(raveObj, bulletObj));
		o->collisionObjMap.insert(std::make_pair(bulletObj->rigidBody.get(),
				raveObj));
	}

	for (std::map<BulletObject::Ptr, int>::const_iterator i =
			childPosMap.begin(); i != childPosMap.end(); ++i) {
		const int j = childPosMap.find(i->first)->second;
		o->childPosMap.insert(std::make_pair(o->getChildren()[j], i->second));
	}

	o->body = o->rave->env->GetKinBody(body->GetName());
	/////////////////// end duplicated portion //////////////////


	o->createdManips.reserve(createdManips.size());
	for (int i = 0; i < createdManips.size(); ++i) {
		o->createdManips.push_back(createdManips[i]->copy(o, f));
		o->createdManips[i]->index = i;
	}
	return o;
}

void RaveObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
	Ptr o = boost::static_pointer_cast<RaveObject>(copy);

	for (BulletInstance::CollisionObjectSet::const_iterator i =
			ignoreCollisionObjs.begin(); i != ignoreCollisionObjs.end(); ++i)
		o->ignoreCollisionObjs.insert((btCollisionObject *) f.copyOf(*i));
}

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, RobotBasePtr robot_, const btTransform &initTrans, TrimeshMode trimeshMode, bool isDynamic) {
	robot = robot_;
	initRaveObject(rave_, robot_, initTrans, trimeshMode, .0005 * METERS, isDynamic);
}

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, const std::string &uri, const btTransform &initTrans, TrimeshMode trimeshMode, bool isDynamic) {
	robot = rave_->env->ReadRobotURI(uri);
	initRaveObject(rave_, robot, initTrans, trimeshMode, .0005 * METERS, isDynamic);
	rave->env->AddRobot(robot);
}

RaveRobotObject::Manipulator::Ptr RaveRobotObject::createManipulator(
		const std::string &manipName, bool useFakeGrabber) {
	RaveRobotObject::Manipulator::Ptr m(new Manipulator(this));
	// initialize the ik module
	robot->SetActiveManipulator(manipName);
	m->manip = m->origManip = robot->GetActiveManipulator();
	m->ikmodule = RaveCreateModule(rave->env, "ikfast");
	rave->env->AddModule(m->ikmodule, "");
	// stringstream ssin, ssout;
	// ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
	// if (!m->ikmodule->SendCommand(ssout, ssin)) {
	//   cout << "failed to load iksolver\n";
	//     return Manipulator::Ptr(); // null
	// }

	m->useFakeGrabber = useFakeGrabber;
	if (useFakeGrabber) {
		m->grabber.reset(new GrabberKinematicObject(0.02, 0.05));
		m->updateGrabberPos();
		ignoreCollisionWith(m->grabber->rigidBody.get());
	}

	m->index = createdManips.size();
	createdManips.push_back(m);
	return m;
}

void RaveRobotObject::Manipulator::updateGrabberPos() {
	// set the grabber right on top of the end effector
	if (useFakeGrabber)
		grabber->motionState->setKinematicPos(getTransform());
}

bool RaveRobotObject::Manipulator::solveIKUnscaled(
		const OpenRAVE::Transform &targetTrans, vector<dReal> &vsolution) {

	vsolution.clear();
	// TODO: lock environment?!?!
	// notice: we use origManip, which is the original manipulator (after cloning)
	// this way we don't have to clone the iksolver, which is attached to the manipulator
	if (!origManip->FindIKSolution(IkParameterization(targetTrans), vsolution,
			IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions)) {
		cout << "ik failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RaveRobotObject::Manipulator::solveAllIKUnscaled(
		const OpenRAVE::Transform &targetTrans,
		vector<vector<dReal> > &vsolutions) {

	vsolutions.clear();
	// see comments for solveIKUnscaled
	if (!origManip->FindIKSolutions(IkParameterization(targetTrans),
			vsolutions, true)) {
		std::cout << "ik  failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RaveRobotObject::Manipulator::moveByIKUnscaled(
		const OpenRAVE::Transform &targetTrans, bool checkCollisions,
		bool revertOnCollision) {

	vector<dReal> vsolution;
	if (!solveIKUnscaled(targetTrans, vsolution))
		return false;

	// save old manip pose if we might want to revert
	vector<dReal> oldVals(0);
	if (checkCollisions && revertOnCollision) {
		robot->robot->SetActiveDOFs(manip->GetArmIndices());
		robot->robot->GetActiveDOFValues(oldVals);
	}

	// move the arm
	robot->setDOFValues(manip->GetArmIndices(), vsolution);
	if (checkCollisions && robot->detectCollisions()) {
		if (revertOnCollision)
			robot->setDOFValues(manip->GetArmIndices(), oldVals);
		return false;
	}

	if (useFakeGrabber)
		updateGrabberPos();

	return true;
}

float RaveRobotObject::Manipulator::getGripperAngle() {
	vector<int> inds = manip->GetGripperIndices();
	assert(inds.size() ==1 );
	vector<double> vals = robot->getDOFValues(inds);
	return vals[0];
}

void RaveRobotObject::Manipulator::setGripperAngle(float x) {
	vector<int> inds = manip->GetGripperIndices();
	assert(inds.size() ==1 );
	vector<double> vals;
	vals.push_back(x);
	robot->setDOFValues(inds, vals);
}

vector<double> RaveRobotObject::Manipulator::getDOFValues() {
	return robot->getDOFValues(manip->GetArmIndices());
}

void RaveRobotObject::Manipulator::setDOFValues(const vector<double>& vals) {
	robot->setDOFValues(manip->GetArmIndices(), vals);
}


RaveRobotObject::Manipulator::Ptr RaveRobotObject::Manipulator::copy(
		RaveRobotObject::Ptr newRobot, Fork &f) {
	OpenRAVE::EnvironmentMutex::scoped_lock lock(
			newRobot->rave->env->GetMutex());

	Manipulator::Ptr o(new Manipulator(newRobot.get()));

	o->ikmodule = ikmodule;
	o->origManip = origManip;

	newRobot->robot->SetActiveManipulator(manip->GetName());
	o->manip = newRobot->robot->GetActiveManipulator();

	o->useFakeGrabber = useFakeGrabber;
	if (useFakeGrabber)
		o->grabber = boost::static_pointer_cast<GrabberKinematicObject>(
				grabber->copy(f));

	return o;
}

void RaveRobotObject::destroyManipulator(RaveRobotObject::Manipulator::Ptr m) {
	std::vector<Manipulator::Ptr>::iterator i = std::find(
			createdManips.begin(), createdManips.end(), m);
	if (i == createdManips.end())
		return;
	(*i).reset();
	createdManips.erase(i);
}
