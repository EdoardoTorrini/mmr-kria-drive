#ifndef WAYPOINTS_SPLINE_H
#define WAYPOINTS_SPLINE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "mmr_kria_base/edf_node.hpp"

class WaypointSpline{
private:
  //Variable used:
  visualization_msgs::msg::Marker waypointDiscretizedSpline;
  std::vector<double> curvaturePoints;

public:
  //Constructor
  WaypointSpline(rclcpp::Time timeStamp);
  void GenerateDiscretizedSpline(visualization_msgs::msg::Marker waypoints, unsigned int resolution);
  visualization_msgs::msg::Marker getWaypointDiscretizedSpline();
  std::vector<double> getCurvaturePoints();
  void resetData();
};


#endif // WAYPOINTS_SPLINE_H
