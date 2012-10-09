#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "plotters.h"
#include "config_sqp.h"
#include "simulation/openravesupport.h"
#include <osg/Depth>
using namespace std;
using namespace Eigen;
using namespace util;


#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>

void makeFullyTransparent(BulletObject::Ptr obj) {
  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask( false );
  obj->node->getOrCreateStateSet()->setAttributeAndModes( depth, osg::StateAttribute::ON );
}


void handler(int sig) {
  void *array[50];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 50);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, 2);
  exit(1);
}

struct LocalConfig : Config {
  static int nSteps;
  static int nIter;
  static int startPosture;
  static int endPosture;
  static int plotType;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int>("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<int>("startPosture", &startPosture, "start posture"));
    params.push_back(new Parameter<int>("endPosture", &endPosture, "end posture"));
    params.push_back(new Parameter<int>("plotType", &plotType, "0: grippers, 1: arms"));
  }
};
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
int LocalConfig::startPosture=3;
int LocalConfig::endPosture=1;
int LocalConfig::plotType = 1;

const static double postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched
const static double base_states[][3] = {
		{0.0, 0.0, 0.0}, // 0=origin
		{3, 0.0, 0.0}, //1=moved
};

void removeBodiesFromBullet(vector<BulletObject::Ptr> objs, btDynamicsWorld* world) {
  BOOST_FOREACH(BulletObject::Ptr obj, objs) {
    if (obj && obj->rigidBody)
      world->removeRigidBody(obj->rigidBody.get());
  }
}

int main(int argc, char *argv[]) {

  //signal(SIGABRT, handler);   // install our handler

	GeneralConfig::scale = 1.;
	BulletConfig::friction = 2; // for if you're shooting blocks
	BulletConfig::margin = .01;
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(LocalConfig());
	parser.addGroup(SQPConfig());
	parser.read(argc, argv);

	const float table_height = .65;
	const float table_thickness = .06;

	if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

	Scene scene;
	util::setGlobalEnv(scene.env);
	BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.85, .65, table_thickness / 2),
			btTransform(btQuaternion(0, 0, 0.001, 1).normalized(), GeneralConfig::scale * btVector3(1.1, .1, table_height - table_thickness / 2))));
	scene.env->add(table);
	PR2Manager pr2m(scene);
	RaveRobotObject::Ptr pr2 = pr2m.pr2;
	pr2->setColor(1,1,1,.4);
	table->setColor(0,0,0,.3);
	removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);
	BOOST_FOREACH(BulletObjectPtr obj, pr2->children) if(obj) makeFullyTransparent(obj);
	makeFullyTransparent(table);

	BulletRaveSyncher brs = syncherFromRobotBody(pr2);

	int nJoints = 3;
    VectorXd startJoints = Map<const VectorXd>(base_states[0], nJoints);
    VectorXd endJoints = Map<const VectorXd>(base_states[1], nJoints);

    TIC();
	PlanningProblem prob;
	MatrixXd initTraj = makeTraj(startJoints, endJoints, LocalConfig::nSteps);
	LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),SQPConfig::lengthCoef));
	std::vector<int> dofInds;
	dofInds.push_back(OpenRAVE::DOF_X);
	dofInds.push_back(OpenRAVE::DOF_Y);
	dofInds.push_back(OpenRAVE::DOF_RotationAxis);
	CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs, dofInds, -BulletConfig::linkPadding/2, SQPConfig::collCoef));


	//JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
	prob.initialize(initTraj, true);
    prob.addComponent(lcc);
    prob.addComponent(cc);
    //prob.m_model->feasRelax(GRB_FEASRELAX_LINEAR, true, true, true);


    //prob.addComponent(jb);
	LOG_INFO_FMT("setup time: %.2f", TOC());

	cout << prob.m_currentTraj << endl;
	TrajPlotterPtr plotter(new BasePlotter(pr2, &scene, brs, SQPConfig::plotDecimation));
//  if (LocalConfig::plotType == 0) {
//	  //plotter.reset(new GripperPlotter(rarm, &scene, 1));
//  }
//  else if (LocalConfig::plotType == 1) {
//	  //plotter.reset(new ArmPlotter(rarm, &scene, brs, SQPConfig::plotDecimation));
//  }
//  else throw std::runtime_error("invalid plot type");

  prob.addPlotter(plotter);

scene.startViewer();
  TIC1();
	try {
	  prob.optimize(LocalConfig::nIter);
	}
	catch (GRBException e) {
	  cout << e.getMessage() << endl;
	  handler(0);
	  throw;
	}
	long time = TOC();
	cout << "Optimization time"<<time<<endl;
	cout << prob.m_currentTraj << endl;
	LOG_INFO_FMT("optimization time: %.2f", TOC());
	  scene.idle(true);

}
