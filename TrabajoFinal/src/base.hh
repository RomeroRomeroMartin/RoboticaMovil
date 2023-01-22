/*
 * base.hh (C) Inaki Rano 2022
 * 
 * Definition of types for the 2D robot simulator
 * 
 * Types defined (from Eigen):
 * - Position
 * - Pose
 * - Velocity
 *
 * Definition of the function
 *
 * - wrap2pi(): Wraps an angular variable to the range (-pi,pi]
 *
 */
#ifndef RS_BASE_HH
#define RS_BASE_HH
#include <Eigen/Dense>
#include <cmath>


namespace rs {
  
  typedef Eigen::Vector2f Position;
  typedef Eigen::Vector3f Pose;
  typedef Eigen::Vector2f Velocity;
    
  inline float wrap2pi(float z)
  {
    while (z <= -M_PI) z += 2*M_PI;
    while (z > M_PI) z -= 2*M_PI;
    
    return z;
  }

}
#endif

