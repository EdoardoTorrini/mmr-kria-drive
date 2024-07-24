#include "pure_pursuit/waypoints_spline.h"

WaypointSpline::WaypointSpline(rclcpp::Time timeStamp){
  //Initializing marker
  waypointDiscretizedSpline.id = 3;
  waypointDiscretizedSpline.header.frame_id = "track";
  waypointDiscretizedSpline.header.stamp = timeStamp;
  waypointDiscretizedSpline.type = visualization_msgs::msg::Marker::LINE_STRIP;
  waypointDiscretizedSpline.action = visualization_msgs::msg::Marker::ADD;
  waypointDiscretizedSpline.scale.x = 0.3;
  waypointDiscretizedSpline.color.a = 1.0;
  waypointDiscretizedSpline.color.r = 0.0;
  waypointDiscretizedSpline.color.g = 1.0;
  waypointDiscretizedSpline.color.b = 0.0;
  //Initialize with identity quaternion (no rotation)
  waypointDiscretizedSpline.pose.orientation.x = 0.0;
  waypointDiscretizedSpline.pose.orientation.y = 0.0;
  waypointDiscretizedSpline.pose.orientation.z = 0.0;
  waypointDiscretizedSpline.pose.orientation.w = 1.0;
}


void WaypointSpline::GenerateDiscretizedSpline(visualization_msgs::msg::Marker waypoints, unsigned int resolution){

  //Before calculate anithing reset old data
  resetData();

  //To round more the spline connect half of the waypoint
  //Only if we can see enought waypoints
  //bool jump2 = (waypoints.points.size() > 3);
  bool jump2 = false;

  double step = 1.0 / resolution;
  double start = 0.0;
  //REmember the last 4 points are fake, stop at the last REAL waypoints
  double finish = static_cast<double>(waypoints.points.size() - 5);

  // Iterates on all the desired points, but dont connect the last point with the first
  //So we need to end at the penultimate that connect the last one and no more.
  for (double t = start; t < finish; t += step){

    //Skip some points use only even numbers
    //This is done to round more the spline
    if( jump2 && (static_cast<int>(t) % 2 != 0 )) continue;

    //Parameter for Spline
    size_t  size = static_cast<size_t>(waypoints.points.size());
    size_t p0, p1, p2, p3;

    if(jump2){
      //If you use even number the jump is 2
      p1 = static_cast<size_t>(t);
      p2 = (p1 + 2) % size;
      p3 = (p2 + 2) % size;
      p0 = p1 >= 2 ? p1 - 2 : p1;
    }
    else{
      //If you use all number the jump is 1
      p1 = static_cast<size_t>(t);
      p2 = (p1 + 1) % size;
      p3 = (p2 + 1) % size;
      p0 = p1 >= 1 ? p1 - 1 : p1;
    }

    // Computing point position with spline parameters
    double myt =  t - static_cast<size_t>(t);
    double tt = myt * myt;
    double ttt = tt * myt;
    double i1 =   -ttt +2*tt -myt;
    double i2 =  3*ttt -5*tt    +2;
    double i3 = -3*ttt +4*tt +myt;
    double i4 =    ttt   -tt;

    //Calculating point and adding to the curvature spline
    geometry_msgs::msg::Point p;
    p.x = (waypoints.points.at(p0).x * i1 + waypoints.points.at(p1).x * i2 + waypoints.points.at(p2).x * i3 + waypoints.points.at(p3).x * i4) / 2;
    p.y = (waypoints.points.at(p0).y * i1 + waypoints.points.at(p1).y * i2 + waypoints.points.at(p2).y * i3 + waypoints.points.at(p3).y * i4) / 2;
    waypointDiscretizedSpline.points.push_back(p);

    // Computing point curvature
    double k1 = -6*t  +4;
    double k2 = 18*t  -10;
    double k3 = -18*t +8;
    double k4 = 6*t   -2;
    double curX = (waypoints.points.at(p0).x * k1 + waypoints.points.at(p1).x * k2 + waypoints.points.at(p2).x * k3 + waypoints.points.at(p3).x * k4) /2;
    double curY = (waypoints.points.at(p0).y * k1 + waypoints.points.at(p1).y * k2 + waypoints.points.at(p2).y * k3 + waypoints.points.at(p3).y * k4) /2;

    //Calculating curvature and adding to the array
    curvaturePoints.push_back(sqrt(pow(curX, 2) + pow(curY, 2)));
  }
}


visualization_msgs::msg::Marker WaypointSpline::getWaypointDiscretizedSpline(){
  return waypointDiscretizedSpline;
}


std::vector<double> WaypointSpline::getCurvaturePoints(){
  return curvaturePoints;
}

void WaypointSpline::resetData(){
  //Reset all data ready for next iteration
  waypointDiscretizedSpline.points.clear();
  curvaturePoints.clear();
}
