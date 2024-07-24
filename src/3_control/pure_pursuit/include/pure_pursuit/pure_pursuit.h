#ifndef FIRST_LAP_PLANNER_H
#define FIRST_LAP_PLANNER_H


#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mmr_kria_base/msg/race_status.hpp"
#include "mmr_kria_base/msg/vehicle_state.hpp"
#include "mmr_kria_base/msg/speed_profile_points.hpp"
#include "mmr_kria_base/msg/target_speed.hpp"
#include "mmr_kria_base/car_configurations.hpp"
#include "mmr_kria_base/kria_common_functions.hpp"
#include "mmr_kria_base/edf_node.hpp"

#include "pure_pursuit/waypoints_spline.h"

using namespace std::chrono_literals;


class PurePursuit: public EDFNode{

private:
  //Variable used:
  //ros::Publisher *carTargetPub,*waypointsSplinePub,*waypointTargetPub,*targetSpeedPub;
  rclcpp::Publisher<mmr_kria_base::msg::TargetSpeed>::SharedPtr targetSpeedPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypointsSplinePub, waypointTargetPub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr carTargetPub;

  rclcpp::Subscription<mmr_kria_base::msg::VehicleState>::SharedPtr driveActualSub;
  // rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr wayPointSub = nh->create_subscription<visualization_msgs::msg::Marker>("planning/borders", 10, std::bind(&PurePursuit::updateWaypoints, &purePursuit, std::placeholders::_1));
  rclcpp::Subscription<mmr_kria_base::msg::SpeedProfilePoints>::SharedPtr racingLineSub;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr centralLineSub;
  rclcpp::Subscription<mmr_kria_base::msg::RaceStatus>::SharedPtr _sub_race_status;

  visualization_msgs::msg::Marker waypointSplineMarkers, waypointTarget;
  std::vector<ackermann_msgs::msg::AckermannDrive> speedProfile;
  mmr_kria_base::msg::VehicleState vehicleActual;
  ackermann_msgs::msg::AckermannDrive carTarget;
  unsigned int targetWaypointId, speedWaypointId;
  double carX,carY,carYaw,carRearX,carRearY, oldCarX, oldCarY;
  double lookForward, speedDistance;
  mmr_kria_base::msg::RaceStatus raceStatus;
  mmr_kria_base::msg::TargetSpeed targetSpeedMsg;
  //ros::WallTime startTime;
  //ros::WallDuration exeTime, wcet;
  WaypointSpline *waypointSpline;
  unsigned int lastWaypoint;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  //Car and speed control Parameter:
  double velocityTarget, accelerationPGain, smootedSpeed, k_smooth;
  float minSpeed, maxSpeed, targetSpeedWeight, distToSwitchWaypoint;
  //Steer control Parameter
  double minLookForward, minLookForwardGain, minSpeedDistance, steerGain,maxSteer;
  //Launch parameters:
  std::string event;
  bool shutdownAfterFirstLap;
  bool allEventWithPurePursuit;
  bool useDynamicTargetSpeed, useRacingLine;
  bool controlSpeed;
  bool stop, isTrackOpen;
  int controlHz, splineWaypointResolution, lapsToDo, posCounter, slowLaps;

  std::vector<float> velocity_coeffs = {0.5051f, 1.063f, -0.02495f, 0.0003832f, -3.251e-6f, 1.494e-8f, -3.511e-11f, 3.32e-14f};

public:
  //Constructor
  PurePursuit();

  //Main methods:
  void loadParameter();
  void updateVehicleActualSpeed(mmr_kria_base::msg::VehicleState::SharedPtr vehicleActual);
  void updateWaypoints(visualization_msgs::msg::Marker::SharedPtr waypoints);
  void racingLineCb(mmr_kria_base::msg::SpeedProfilePoints::SharedPtr racingLine);
  void centralLineCb(visualization_msgs::msg::Marker::SharedPtr centralLine);
  void calculateCarCommands();
  void sendActuation();
  void raceStatusCb(mmr_kria_base::msg::RaceStatus::SharedPtr raceStatusMsg);

  void generateSpline(visualization_msgs::msg::Marker::SharedPtr waypoints);
  float polyeval_targetVelocity(float radius);
  float mengerCurvature(geometry_msgs::msg::Point prev, geometry_msgs::msg::Point curr, geometry_msgs::msg::Point next);

  //Pure Pursuit methods:
  void findTargetWaypoint();
  void findTargetWaypointOpenTrack();
  void calculateSteeringTarget();
  void calculateVelocityTarget();
  void calculateAcceleratorTarget();
  double computeAngleDiff(double angle1,double angle2);
  int getControlHz();
};

#endif // FIRST_LAP_PLANNER_H
