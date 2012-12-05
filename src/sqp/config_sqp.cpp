#include "config_sqp.h"
double SQPConfig::collCoefInit = 20;
int SQPConfig::nStepsInit = 15;
double SQPConfig::lengthCoef = .5;
bool SQPConfig::topCollOnly = false;
int SQPConfig::plotDecimation = 10;
bool SQPConfig::pauseEachIter = false;
double SQPConfig::distPen = .02;
double SQPConfig::distDiscSafe = .018;
double SQPConfig::distContSafe = .01;
double SQPConfig::shapeExpansion = .02;
int SQPConfig::maxIter=50;
double SQPConfig::trShrink = .2;
double SQPConfig::trExpand = 1.2;
double SQPConfig::trThresh = .2;
double SQPConfig::shrinkLimit = .01;
double SQPConfig::doneIterThresh = .01;
double SQPConfig::maxCollCoef=200;
double SQPConfig::maxSteps=100;
bool SQPConfig::enablePlot=true;
int SQPConfig::padMult = 1;
