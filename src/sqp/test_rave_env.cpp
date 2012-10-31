#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "robots/robot_manager.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "config_sqp.h"
#include <osg/Depth>
#include <json/json.h>
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "planning_problems.h"
#include "kinematics_utils.h"
#include "plotters.h"
#include "utils_openrave.h"
using namespace std;
using namespace Eigen;
using namespace util;
namespace fs = boost::filesystem;

MatrixXd ravePlannerTest(OpenRAVE::EnvironmentBasePtr penv, OpenRAVE::RobotBasePtr probot,
    OpenRAVE::RobotBase::ManipulatorPtr pmanip, vector<double> target, const string plannerName="birrt"){
  PlannerBasePtr planner = RaveCreatePlanner(penv, plannerName);
  probot->SetActiveDOFs(pmanip->GetArmIndices());

  PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
  params->_nMaxIterations = 4000; // max iterations before failure
  params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs
  params->vgoalconfig.resize(probot->GetActiveDOF());
  params->vgoalconfig = target;

  RAVELOG_INFO("starting to plan\n");
  probot->GetActiveDOFValues(params->vinitialconfig);
  if( !planner->InitPlan(probot,params) ) {
      return MatrixXd();
  }

  // create a new output trajectory
  TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv,"");
  if( !planner->PlanPath(ptraj) ) {
      RAVELOG_WARN("plan failed, trying again\n");
      return MatrixXd();
  }

  return raveTrajectoryToEigen(ptraj);
}

Json::Value readJson(fs::path jsonfile) {
  // occasionally it fails, presumably when the json isn't done being written. so repeat 10 times
  std::ifstream infile(jsonfile.string().c_str());
  if (infile.fail()) throw FileOpenError(jsonfile.string());

  Json::Reader reader;
  Json::Value root;
  infile >> root;
  return root;
}

float randf() {return (float)rand()/(float)RAND_MAX;}


struct LocalConfig: Config {
  static string probSpec;
  static string jsonOutputPath;
  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("probSpec", &probSpec, "problem specification"));
    params.push_back(new Parameter<string> ("jsonOutputPath", &jsonOutputPath, "path to output final trajectory as JSON"));
  }
};
string LocalConfig::probSpec = "";
string LocalConfig::jsonOutputPath = "";

int main(int argc, char *argv[]) {

  BulletConfig::linkPadding = .02;
  BulletConfig::margin = .01;
  SQPConfig::padMult = 2;
  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 10.;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

  Scene scene;
  scene.startViewer();

  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");


  Json::Value probInfo = readJson(LocalConfig::probSpec);

  if (probInfo.isMember("env")) Load(scene.env, scene.rave, probInfo["env"].asString(),false);
  else ASSERT_FAIL();

  vector<double> startJoints;
  for (int i=0; i < probInfo["start_joints"].size(); ++i) startJoints.push_back(probInfo["start_joints"][i].asDouble());

  RaveRobotObject::Ptr robot = getRobotByName(scene.env, scene.rave, probInfo["robot"].asString());
  RaveRobotObject::Manipulator::Ptr arm = robot->createManipulator(probInfo["manip"].asString(), false);

  assert(robot);
  assert(arm);

  removeBodiesFromBullet(robot->children, scene.env->bullet->dynamicsWorld);
  BOOST_FOREACH(EnvironmentObjectPtr obj, scene.env->objects) {
    BulletObjectPtr bobj = boost::dynamic_pointer_cast<BulletObject>(obj);
    obj->setColor(randf(),randf(),randf(),1);
//    if (bobj) makeFullyTransparent(bobj);
  }
  robot->setColor(0,1,1, .4);

  vector<double> goal;
  for (int i=0; i < probInfo["goal"].size(); ++i) goal.push_back(probInfo["goal"][i].asDouble());

  arm->setGripperAngle(.5);

  BulletRaveSyncherPtr brs = syncherFromArm(arm);

  TIC();
  PlanningProblem prob;
  prob.addPlotter(ArmPlotterPtr(new ArmPlotter(arm, &scene,  SQPConfig::plotDecimation)));


  if (probInfo["goal_type"] == "joint") {
    int nJoints = 7;
    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    VectorXd endJoints = toVectorXd(goal);
    MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit);
    LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(
        initTraj), SQPConfig::lengthCoef));
    CollisionCostPtr cc(new CollisionCost(robot->robot, scene.env->bullet->dynamicsWorld, brs,
        arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
    JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
    prob.initialize(initTraj, true);
    prob.addComponent(lcc);
    prob.addComponent(cc);
    prob.addComponent(jb);
  }
  else if (probInfo["goal_type"] == "cart") {

    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
        btVector3(goal[4], goal[5], goal[6]));
    util::drawAxes(goalTrans, .15 * METERS, scene.env);
    TIC1();
    planArmToCartTarget(prob, startJoints, goalTrans, arm);
    cout << "total time: " << TOC()<< endl;

    cout << "Trying OpenRAVE planner" << endl;
    TIC1();
    vector<double> ikSoln;
    bool ikSuccess = arm->solveIKUnscaled(util::toRaveTransform(goalTrans), ikSoln);
    if (!ikSuccess) {
      LOG_ERROR("no ik solution for target!");
    }
    MatrixXd raveTraj = ravePlannerTest(scene.rave->env, robot->robot,
        arm->origManip, ikSoln);
    cout << "total time: " << TOC()<< endl;
  }
  else if (probInfo["goal_type"] == "grasp") {

    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
        btVector3(goal[4], goal[5], goal[6]));
    util::drawAxes(goalTrans, .15 * METERS, scene.env);
    TIC1();
    planArmToGrasp(prob, startJoints, goalTrans, arm);
    cout << "total time: " << TOC() << endl;

  }
  if(!LocalConfig::jsonOutputPath.empty()){
    prob.writeTrajToJSON(LocalConfig::jsonOutputPath);
  }


  prob.m_plotters[0].reset();

  BulletConfig::linkPadding = 0;
  //scene.env->remove(pr2);
  RobotManager robotm1(scene);
  interactiveTrajPlot(prob.m_currentTraj, robotm1.botLeft,  &scene);
  scene.idle(true);

}
