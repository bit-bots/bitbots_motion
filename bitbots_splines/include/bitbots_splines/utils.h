#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_UTILS_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_UTILS_H_

#include <math.h>       /* atan */
#include <Eigen/Geometry>

namespace bitbots_splines {

inline Eigen::Vector3d cartesian2leg(Eigen::Vector3d cartesian){
  // 3d pythagoras
  double length = cartesian.norm();
  // solving triangle
  double alpha = atan(cartesian.x() / cartesian.z());
  double beta = atan(cartesian.y() / cartesian.z());
  Eigen::Vector3d leg(length, alpha, beta);
  return leg;
}

inline Eigen::Vector3d leg2cartesian(Eigen::Vector3d leg){
  // solution by solving:
  // x = tan(alpha)* z
  // y = tan(beta) * z
  // l² = x²+y²+z² = tan(alpha)²*z²+tan(beta)²*z²+z²
  // l²=z²*(tan(alpha)²+tan(beta)²+1)
  // z=root(l²/(tan(alpha)²+tan(beta)²+1))

  double l = leg.x();
  double alpha = leg.y();
  double beta = leg.z();
  double z= -sqrt((l*l)/(tan(alpha)*tan(alpha)+tan(beta)*tan(beta)+1));
  double x= tan(alpha) * z;
  double y= tan(beta) * z;
  Eigen::Vector3d cartesian(x, y, z);
  return cartesian;
}
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_UTILS_H_
