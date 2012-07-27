/** Author: Dmitry Berenson
    Maintained by: Ankush Gupta | 27th July 2012. */

#include "colab_folding_utils.h"

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
