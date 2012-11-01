#include "utils_openrave.h"
#include <vector>
#include <iostream>

using namespace std;
Eigen::MatrixXd raveTrajectoryToEigen(OpenRAVE::TrajectoryBasePtr trajectory){
  int waypoints = trajectory->GetNumWaypoints();
  int dofs = trajectory->GetConfigurationSpecification().GetDOF();
  std::vector<OpenRAVE::dReal> data;
  trajectory->GetWaypoints(0, waypoints, data);
  Eigen::Map<Eigen::MatrixXd> out(data.data(), waypoints, dofs);
  return out;
}
