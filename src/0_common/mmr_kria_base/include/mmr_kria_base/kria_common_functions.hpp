#pragma once

// #include <rclcpp/rclcpp.hpp>
#include <cmath>

/*
 * Extern function
 * They could be used in all other packages
 *
 * If you define them here, you must declare as inline
 * Otherwise use different methods: declaration in this file and
 * definition in mmr_common_functions.cpp file
*/

/* normalizeAngle:
 * take an angle in radiants and normalize it
 * between -pi and pi
*/
inline double normalizeAngle(double angle){
  while(angle > M_PI) angle -= (2 * M_PI);
  while(angle < -M_PI) angle += (2 * M_PI);
  return angle;
}

/* euclideanDistance:
 * Compute Euclidean Distance between 2 points:
 * p1(x1,y1), p2(x2,y2)
*/
inline double euclideanDistance(double x1, double x2, double y1, double y2) {
  return sqrt(((x2 - x1)*(x2 - x1)) + ((y2 - y1)*(y2 - y1)));
}