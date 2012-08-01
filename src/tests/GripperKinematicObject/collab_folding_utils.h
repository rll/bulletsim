/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta | 27th July 2012. */


#ifndef _COLAB_FOLDING_UTILS_
#define _COLAB_FOLDING_UTILS_

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

/** Simply copies the coordinates of the nodes in M_NODES
    into NODEPOSEVEC. */
void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node>
			      &m_nodes, std::vector<btVector3> &nodeposvec);


/** Reflects an input 3 vector about the MID_X. */
class PointReflector {

 public:
  float mid_x, min_y, max_y;
  typedef boost::shared_ptr<PointReflector> Ptr;

  /** Constructor. _MID_X defines the x-coordinate of the plane of symmetry. 
      The plane of symmetry is parallel to the Y-Z plane. */
  PointReflector(float _mid_x, float _min_y=0, float _max_y=100);

  /** Returns the vector_in reflected about MID_X plane. */
  btVector3 reflect(btVector3& vector_in);
};

/** Returns the Moore-Penrose psuedo inverse of the input matrix MAT.
    This psuedo-inverse is the standard in MATLAB.

    see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse */
Eigen::MatrixXf pinv(const Eigen::MatrixXf &mat);
#endif
