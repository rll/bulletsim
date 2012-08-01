/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta | 27th July 2012. */

#include "collab_folding_utils.h"

/** Simply copies the coordinates of the nodes in M_NODES
    into NODEPOSEVEC. */
void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node>
			      &m_nodes, std::vector<btVector3> &nodeposvec) {
  nodeposvec.resize(m_nodes.size());
  for(int i =0; i < m_nodes.size(); i += 1)
    nodeposvec[i] = m_nodes[i].m_x;
}

/** Constructor. _MID_X defines the x-coordinate of the plane of symmetry. 
    The plane of symmetry is parallel to the Y-Z plane. */
PointReflector::PointReflector(float _mid_x, float _min_y, float _max_y) {
  mid_x = _mid_x;
  min_y = _min_y;
  max_y = _max_y;
}

/** Returns the vector_in reflected about MID_X plane. */
btVector3 PointReflector::reflect(btVector3& vector_in) {
    btVector3 v_out = vector_in;
    v_out[0] = vector_in[0] - 2*(vector_in[0] - mid_x);
    return v_out;
}

/** Returns the Moore-Penrose psuedo inverse of the input matrix MAT.
    This psuedo-inverse is the standard in MATLAB.

    see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse */
Eigen::MatrixXf pinv(const Eigen::MatrixXf &mat) {

  if ( mat.rows() < mat.cols() ) {
    std::cout << "Pseudo-inverse error : number of cols > number of rows." << std::endl;
    return Eigen::MatrixXf();
  }
  
  // singular value decomposition.
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

  /** Build a diagonal matrix with the inverted singular values
      The pseudo inverted singular matrix is formed by replacing
      every nonzero entry by its reciprocal. 
      Sufficiently small values are treated as zero. */
  Eigen::MatrixXf singular_values = svd.singularValues();
  Eigen::MatrixXf inverted_singular_values(svd.matrixV().cols(), 1);

  for (int iRow = 0; iRow < singular_values.rows(); iRow += 1) {
    if ( fabs(singular_values(iRow)) <= 1e-10 ) //Todo: Put epsilon in parameter	
      inverted_singular_values(iRow, 0) = 0.;
    else
      inverted_singular_values(iRow,0) = 1./singular_values(iRow);
  }

  Eigen::MatrixXf mAdjointU = svd.matrixU().adjoint().
    block(0, 0, singular_values.rows(), svd.matrixU().adjoint().cols());

  // Pseudo-Inverse = V * S * U'
  return (svd.matrixV() * inverted_singular_values.asDiagonal()) * mAdjointU;
}
