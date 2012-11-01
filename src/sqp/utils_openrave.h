#ifndef UTILS_OPENRAVE_H_
#define UTILS_OPENRAVE_H_

#include <Eigen/Dense>
#include <openrave/openrave.h>

Eigen::MatrixXd raveTrajectoryToEigen(OpenRAVE::TrajectoryBasePtr);


#endif /* UTILS_OPENRAVE_H_ */
