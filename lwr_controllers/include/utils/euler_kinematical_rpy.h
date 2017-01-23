// Author: Nicola Piga, Giulio Romualdi
// eul_kin_RPY() computes the matrix T(PHI) such that omega = T PHI_dot
// where omega is the angular velocity vector  of the body whose attitude is represented by the angles PHI (RPY)
// omega is projected with the rotation matrix R(PHI)
// returns the matrix T

#ifndef EULER_KIN_RPY_H
#define EULER_KIN_RPY_H

#include <math.h>
using namespace Eigen;

inline void eul_kin_RPY(const double pitch, const double yaw, Eigen::Matrix3d &T)
{	
  T << 
    0, -sin(yaw), cos(pitch) * cos(yaw),
    0, cos(yaw), cos(pitch) * sin(yaw),
    1, 0, -sin(pitch);
}

#endif
