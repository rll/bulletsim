#include "simulation/simplescene.h"
#include "robots/pr2.h"

int main(int argc, char* argv[]) {
  SceneConfig::enableIK = true;
  SceneConfig::enableHaptics = true;
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);

  std::string myfile = "../../../home/james-huang/github/bulletsim/data/lab1.env.xml"; //"../../../home/james-huang/github/bulletsim/data/pr2wam_test1.env.xml";

  Scene scene;
  Load(scene.env, scene.rave, myfile);
  scene.startViewer();
  scene.step(0);
  scene.idle(true);
  scene.startLoop();
  
}
