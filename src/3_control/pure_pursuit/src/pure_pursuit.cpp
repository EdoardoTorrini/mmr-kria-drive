#include "pure_pursuit/pure_pursuit.h"

PurePursuit::PurePursuit() : EDFNode("pure_pursuit")
{
  loadParameter();
  this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

  this->declare_parameter<std::string>("event_type", "");
  event = this->get_parameter("event_type").get_value<std::string>();

  if (event.empty()){
    this->declare_parameter<std::string>("common/eventType", "");
    event = this->get_parameter("common/eventType").get_value<std::string>();
  }

  //Setting variables and parameter
  this->carTargetPub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("sim/drive_parameters", 1);
  this->waypointsSplinePub = this->create_publisher<visualization_msgs::msg::Marker>("pure_pursuit/spline_waypoints", 1);
  this->waypointTargetPub = this->create_publisher<visualization_msgs::msg::Marker>("pure_pursuit/waypointTarget", 1);
  this->targetSpeedPub = this->create_publisher<mmr_kria_base::msg::TargetSpeed>("/planning/target_speed", 1);

  this->driveActualSub = this->create_subscription<mmr_kria_base::msg::VehicleState>("/vehicle/actualState", 1, std::bind(&PurePursuit::updateVehicleActualSpeed, this, std::placeholders::_1));
  this->racingLineSub = this->create_subscription<mmr_kria_base::msg::SpeedProfilePoints>("/planning/speedProfilePoints", 10, std::bind(&PurePursuit::racingLineCb, this, std::placeholders::_1));
  this->centralLineSub = this->create_subscription<visualization_msgs::msg::Marker>("/planning/center_line", 5, std::bind(&PurePursuit::centralLineCb, this, std::placeholders::_1));
  this->_sub_race_status = this->create_subscription<mmr_kria_base::msg::RaceStatus>("/planning/race_status", 10, std::bind(&PurePursuit::raceStatusCb, this, std::placeholders::_1));

  tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(this->get_logger(), "initializing data");
  //Initializing messages
  carTarget.jerk = 0.0;
  carTarget.speed = 0.0;
  carTarget.acceleration = 0.0;
  carTarget.steering_angle = 0.0;
  carTarget.steering_angle_velocity = 0.0;
  vehicleActual.actual_speed = 0.0;
  carX = 0;
  carY = 0;
  carYaw = 0;
  carRearX = -carRearWheelToCG;
  carRearY = 0;
  oldCarX = carX;
  oldCarY = carY;
  stop = false;

  posCounter = 0;
  RCLCPP_INFO(this->get_logger(), "racestatus");


  raceStatus.current_lap = 0;
  RCLCPP_INFO(this->get_logger(), "setting up waypoints target");
  waypointTarget.header.frame_id = "track";
  waypointTarget.header.stamp = this->get_clock()->now();
  waypointTarget.ns = "waypointTargetAbsolutePos";
  waypointTarget.id = 0;
  waypointTarget.type = visualization_msgs::msg::Marker::SPHERE;
  waypointTarget.action = visualization_msgs::msg::Marker::ADD;
  waypointTarget.scale.x = 1.0;
  waypointTarget.scale.y = 1.0;
  waypointTarget.scale.z = 0.1;
  waypointTarget.pose.position.x = 0.0;
  waypointTarget.pose.position.y = 0.0;
  waypointTarget.pose.position.z = 0.0;
  waypointTarget.color.r = 1.0;
  waypointTarget.color.g = 0.0;
  waypointTarget.color.b = 0.0;
  waypointTarget.color.a = 1.0;
  //Initialize with identity quaternion (no rotation)
  waypointTarget.pose.orientation.x = 0.0;
  waypointTarget.pose.orientation.y = 0.0;
  waypointTarget.pose.orientation.z = 0.0;
  waypointTarget.pose.orientation.w = 1.0;

  //Initialize waypoint spline
  RCLCPP_INFO(this->get_logger(), "initializing spline");
  waypointSpline = new WaypointSpline(this->get_clock()->now());

  //Start from waypoint 0
  targetWaypointId = 0;
  lastWaypoint = 0;
  speedWaypointId = 0;
  speedDistance = 0.0;
  RCLCPP_INFO(this->get_logger(), "finished constructor");
  //Initilize smooting parameters
  this->smootedSpeed = minSpeed;

}

//PurePursuit main methods:
inline void PurePursuit::loadParameter()
{
  RCLCPP_INFO(this->get_logger(), "loading parameters");
  //staticVelocityTarget
  this->declare_parameter<double>("velocityTarget", 0.0);
  velocityTarget = this->get_parameter("velocityTarget").get_value<double>();

  //minSpeed
  this->declare_parameter<float>("minSpeed", 0.0);
  minSpeed = this->get_parameter("minSpeed").get_value<float>();

  //maxSpeed
  this->declare_parameter<float>("maxSpeed", 0.0);
  maxSpeed = this->get_parameter("maxSpeed").get_value<float>();

  //targetSpeedWeight
  this->declare_parameter<float>("targetSpeedWeight", 0.0);
  targetSpeedWeight = this->get_parameter("targetSpeedWeight").get_value<float>();

  //accelerationPGain
  this->declare_parameter<double>("accelerationPGain", 0.0);
  accelerationPGain = this->get_parameter("accelerationPGain").get_value<double>();

  //minLookForward
  this->declare_parameter<double>("minLookForward", 0.0);
  minLookForward = this->get_parameter("minLookForward").get_value<double>();

  //minLookForwardGain
  this->declare_parameter<double>("minLookForwardGain", 0.0);
  minLookForwardGain = this->get_parameter("minLookForwardGain").get_value<double>();

  //minSpeedDistance
  this->declare_parameter<double>("minSpeedDistance", minLookForward);
  minSpeedDistance = this->get_parameter("minSpeedDistance").get_value<double>();

  if(minSpeedDistance > minLookForward)
  {
    RCLCPP_INFO(this->get_logger(), "minSpeedDistance must be lesser then minlookforward, setted it at minLookForward");
    minSpeedDistance = minLookForward;
  }
  //steerGain
  this->declare_parameter<double>("steerGain", 0.0);
  steerGain = this->get_parameter("steerGain").get_value<double>();

    //shutdownAfterFirstLap
  this->declare_parameter<bool>("shutdownAfterFirstLap", 0.0);
  shutdownAfterFirstLap = this->get_parameter("shutdownAfterFirstLap").get_value<bool>();

    //allEventWithPurePursuit
  this->declare_parameter<bool>("allEventWithPurePursuit", 0.0);
  allEventWithPurePursuit = this->get_parameter("allEventWithPurePursuit").get_value<bool>();

    //useDynamicTargetSpeed
  this->declare_parameter<bool>("useDynamicTargetSpeed", 0.0);
  useDynamicTargetSpeed = this->get_parameter("useDynamicTargetSpeed").get_value<bool>();

  //useRacingLine
  this->declare_parameter<bool>("useRacingLine", 0.0);
  useRacingLine = this->get_parameter("useRacingLine").get_value<bool>();

  //controlSpeed
  this->declare_parameter<bool>("controlSpeed", 0.0);
  controlSpeed = this->get_parameter("controlSpeed").get_value<bool>();

  //controlHz
  this->declare_parameter<int>("controlHz", 0.0);
  controlHz = this->get_parameter("controlHz").get_value<int>();

  //splineWaypointResolution
  this->declare_parameter<int>("splineWaypointResolution", 0.0);
  splineWaypointResolution = this->get_parameter("splineWaypointResolution").get_value<int>();

  //lapsToDo
  this->declare_parameter<int>("lapsToDo", 0.0);
  lapsToDo = this->get_parameter("lapsToDo").get_value<int>();

  //slowLaps
  this->declare_parameter<int>("slowLaps", 0.0);
  slowLaps = this->get_parameter("slowLaps").get_value<int>();

  //isTrackOpen
  this->declare_parameter<bool>("isTrackOpen", 0.0);
  isTrackOpen = this->get_parameter("isTrackOpen").get_value<bool>();

  //distToSwitchWaypoint
  this->declare_parameter<float>("distToSwitchWaypoint", 0.0);
  distToSwitchWaypoint = this->get_parameter("distToSwitchWaypoint").get_value<float>();

    //k_smooth
  this->declare_parameter<double>("k_smooth", 0.25);
  this->k_smooth = this->get_parameter("k_smooth").get_value<double>();

  // EDF parameters
  this->declare_parameter("generic.WCET", 5000000);
	this->declare_parameter("generic.period", 10000000);
	this->declare_parameter("generic.deadline", 10000000);

  this->get_parameter("generic.WCET", this->m_nWCET);
	this->get_parameter("generic.period", this->m_nPeriod);
	this->get_parameter("generic.deadline", this->m_nDeadline);
  
  RCLCPP_INFO(this->get_logger(), "parameters loaded");

  if (event == "acceleration") {
    RCLCPP_INFO(this->get_logger(), "ACCEL EVENT.");
    this->declare_parameter("acceleration/maxSteer", 0.0523599);
    this->get_parameter("acceleration/maxSteer", this->maxSteer);
  }
  else
  {
    this->maxSteer = carMaxSteerAngle;
  }
}

void PurePursuit::raceStatusCb(mmr_kria_base::msg::RaceStatus::SharedPtr raceStatusMsg)
{
  //TODO define operator = for racestatus to racestatus::SharedPtr
  this->raceStatus.current_lap = raceStatusMsg->current_lap;
  this->raceStatus.current_lap_starting_time = raceStatusMsg->current_lap_starting_time;
  this->raceStatus.lap_times = raceStatusMsg->lap_times;
  this->raceStatus.stopping = raceStatusMsg->stopping;
  this->raceStatus.use_brake_line = raceStatusMsg->use_brake_line;
  this->raceStatus.mission_finished = raceStatusMsg->mission_finished;
  this->raceStatus.vehicle_standstill = raceStatusMsg->vehicle_standstill;

  if (allEventWithPurePursuit) {
    if (raceStatusMsg->use_brake_line) {
      if (raceStatusMsg->current_lap >= lapsToDo && raceStatusMsg->stopping){
        stop = true;
      }
    }
    else {
      if (raceStatusMsg->current_lap >= lapsToDo)
      {
        stop = true;
      }
        
    }
  }
}

void PurePursuit::updateVehicleActualSpeed(mmr_kria_base::msg::VehicleState::SharedPtr vehicleActual)
{
  this->vehicleActual = *vehicleActual;
}

void PurePursuit::generateSpline(visualization_msgs::msg::Marker::SharedPtr waypoints)
{
  //If we dont have enought waypoints return
  if (waypoints->points.size() == 0)
    return;

  //Save actual waypoints
  visualization_msgs::msg::Marker myWaypoints;
  myWaypoints.points = waypoints->points;

  //Check if we have at least 2 waypoints
  if (waypoints->points.size() >= 2)
  {
    //Create SPLINE...

    //Update car pose from tf
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_->lookupTransform("track", "vehicle", tf2::TimePointZero);
      carX = transform.transform.translation.x;
      carY = transform.transform.translation.y;
      tf2::Quaternion q;
      tf2::convert(transform.transform.rotation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      carYaw = float(yaw);
      carRearX = carX - (carRearWheelToCG)*cos(carYaw);
      carRearY = carY - (carRearWheelToCG)*sin(carYaw);
    }
    catch (tf2::TransformException ex)
    {
      //rclcpp_ERROR("%s",ex.what());
    }

    //Add to the waypoint array, the Car Position as first "fake Waypoint"
    geometry_msgs::msg::Point carFakeWaypoint;
    carFakeWaypoint.x = carX;
    carFakeWaypoint.y = carY;
    carFakeWaypoint.z = 0;
    myWaypoints.points.insert(myWaypoints.points.begin(), carFakeWaypoint);

    //Add to the waypoint array, 4 "fake Waypoints" to generate a good curvature with spline
    //Why 4???
    //Because if the spline use step2 from the last point and the number of waypoint is even
    //You need at least 4 fake waypoint to calculate the last spline points with the right orientation
    //See: https://stackoverflow.com/questions/6620742/find-3rd-point-in-a-line-using-c
    geometry_msgs::msg::Point lastFakeWaypoint_1, lastFakeWaypoint_2, lastFakeWaypoint_3, lastFakeWaypoint_4;
    lastFakeWaypoint_1.x = (myWaypoints.points.at(myWaypoints.points.size() - 1).x - myWaypoints.points.at(myWaypoints.points.size() - 2).x) * 0.1 + myWaypoints.points.at(myWaypoints.points.size() - 1).x;
    lastFakeWaypoint_1.y = (myWaypoints.points.at(myWaypoints.points.size() - 1).y - myWaypoints.points.at(myWaypoints.points.size() - 2).y) * 0.1 + myWaypoints.points.at(myWaypoints.points.size() - 1).y;
    lastFakeWaypoint_1.z = 0;
    lastFakeWaypoint_2.x = (myWaypoints.points.at(myWaypoints.points.size() - 1).x - myWaypoints.points.at(myWaypoints.points.size() - 2).x) * 0.2 + myWaypoints.points.at(myWaypoints.points.size() - 1).x;
    lastFakeWaypoint_2.y = (myWaypoints.points.at(myWaypoints.points.size() - 1).y - myWaypoints.points.at(myWaypoints.points.size() - 2).y) * 0.2 + myWaypoints.points.at(myWaypoints.points.size() - 1).y;
    lastFakeWaypoint_2.z = 0;
    lastFakeWaypoint_3.x = (myWaypoints.points.at(myWaypoints.points.size() - 1).x - myWaypoints.points.at(myWaypoints.points.size() - 2).x) * 0.3 + myWaypoints.points.at(myWaypoints.points.size() - 1).x;
    lastFakeWaypoint_3.y = (myWaypoints.points.at(myWaypoints.points.size() - 1).y - myWaypoints.points.at(myWaypoints.points.size() - 2).y) * 0.3 + myWaypoints.points.at(myWaypoints.points.size() - 1).y;
    lastFakeWaypoint_3.z = 0;
    lastFakeWaypoint_4.x = (myWaypoints.points.at(myWaypoints.points.size() - 1).x - myWaypoints.points.at(myWaypoints.points.size() - 2).x) * 0.4 + myWaypoints.points.at(myWaypoints.points.size() - 1).x;
    lastFakeWaypoint_4.y = (myWaypoints.points.at(myWaypoints.points.size() - 1).y - myWaypoints.points.at(myWaypoints.points.size() - 2).y) * 0.4 + myWaypoints.points.at(myWaypoints.points.size() - 1).y;
    lastFakeWaypoint_4.z = 0;
    myWaypoints.points.push_back(lastFakeWaypoint_1);
    myWaypoints.points.push_back(lastFakeWaypoint_2);
    myWaypoints.points.push_back(lastFakeWaypoint_3);
    myWaypoints.points.push_back(lastFakeWaypoint_4);

    //Generate Spline passing throw waypoints
    waypointSpline->GenerateDiscretizedSpline(myWaypoints, static_cast<unsigned int>(splineWaypointResolution));

    //Save Spline in local Variable and publish it
    waypointSplineMarkers = waypointSpline->getWaypointDiscretizedSpline();

    //Publish Waypoint Spline to debug
    this->waypointsSplinePub->publish(waypointSplineMarkers);
  }
  else
  {
    // We have only one waypoint, FOLLOW IT!
    //Don't publish it or rviz s'incazza
    waypointSplineMarkers.points = waypoints->points;
  }
}

void PurePursuit::racingLineCb(mmr_kria_base::msg::SpeedProfilePoints::SharedPtr racingLine)
{
  if (!allEventWithPurePursuit) {
    // Kill pure pursuit when global planner send the optimized racing line and leave the control to mpc
    rclcpp::shutdown();
  } else {
    // Use the racing line waypoints to drive with pure pursuit
    useRacingLine = true;
    RCLCPP_INFO(this->get_logger(), "Using racing line provided by global planner");
    waypointSplineMarkers.points.clear();
    for (auto speedPoint : racingLine->points) {
      geometry_msgs::msg::Point p;
      p.x = speedPoint.point.x;
      p.y = speedPoint.point.y;
      waypointSplineMarkers.points.push_back(p);
      speedProfile.push_back(speedPoint.ackerman_point);
    }
  }
}

void PurePursuit::centralLineCb(visualization_msgs::msg::Marker::SharedPtr centralLine)
{
  if (allEventWithPurePursuit && !useRacingLine) {
    // Use the central line waypoints to drive with pure pursuit
    // RCLCPP_INFO(this->get_logger(), "Using central line provided by local planner");
    waypointSplineMarkers.points = centralLine->points;
  }
}

void PurePursuit::updateWaypoints(visualization_msgs::msg::Marker::SharedPtr waypoints)
{
  generateSpline(waypoints);

}

void PurePursuit::calculateCarCommands()
{
  //If we have at least two waypoint run!
  //REMEMBER THE FIRST WAYPOINT IS THE CAR POSITION
  if (waypointSplineMarkers.points.size() == 0)
    return;

  //Update car pose from tf
  geometry_msgs::msg::TransformStamped transform;
  try
  {
      transform = tf_buffer_->lookupTransform("track", "vehicle", tf2::TimePointZero);
      carX = transform.transform.translation.x;
      carY = transform.transform.translation.y;
      tf2::Quaternion q;
      tf2::convert(transform.transform.rotation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      carYaw = float(yaw);
    carRearX = carX - (carRearWheelToCG)*cos(carYaw);
    carRearY = carY - (carRearWheelToCG)*sin(carYaw);
  }
  catch (tf2::TransformException ex)
  {
    RCLCPP_INFO(this->get_logger(),"tf not found ");
    //rclcpp_ERROR("%s",ex.what());
  }
  //rclcpp_INFO_STREAM("yaw = " << carYaw);
  //Check if the car has completed the lap
  if (shutdownAfterFirstLap && raceStatus.current_lap > 1)
    rclcpp::shutdown();

  //Calculate target command with Pure Pursuit and publish it, easy XD
  sendActuation();

  oldCarX = carX;
  oldCarY = carY;
}

inline void PurePursuit::sendActuation()
{
  if (isTrackOpen) {
    findTargetWaypointOpenTrack();
  } else {
    findTargetWaypoint();
  }
  calculateSteeringTarget();
  if (controlSpeed)
  {
    calculateVelocityTarget();
    calculateAcceleratorTarget();
  } else {
    carTarget.speed = 0.0;
  }
  carTargetPub->publish(carTarget);
  targetSpeedMsg.target_speed = velocityTarget;  // target speed in m/s
  targetSpeedPub->publish(targetSpeedMsg);
}

//PurePursuit main methods:
inline void PurePursuit::findTargetWaypoint()
{
  // RCLCPP_INFO(this->nh->get_logger(), "Actual vehicle speed: %lf (m/s)", this->vehicleActual.actual_speed);
  //Calculate look forward based on minimum and waypointsactual speed
  lookForward = minLookForward + minLookForwardGain * static_cast<double>(vehicleActual.actual_speed);
  speedDistance = minSpeedDistance + minLookForwardGain * static_cast<double>(vehicleActual.actual_speed);

  //Setup initial Waypoint at 0
  unsigned int startingWaypoint = 0;
  //If we are in the second lap start searching from lastwaypoint selected
  if (raceStatus.current_lap > slowLaps)
  {
    //Attention if you are at the end of the lap you need to restart looking from id 0
    if (lastWaypoint != (waypointSplineMarkers.points.size() - 1))
      startingWaypoint = lastWaypoint;
    else
      startingWaypoint = 0;
  }

  //Just for safety initialize all at the last waypoint avaiable
  targetWaypointId = waypointSplineMarkers.points.size() - 1;
  
  //Set the right target Waypoint
  for (unsigned int i = startingWaypoint; i < waypointSplineMarkers.points.size(); i++)
  {
    //Discard waypoints behind car
    if (computeAngleDiff(atan2(waypointSplineMarkers.points.at(i).y - carY, waypointSplineMarkers.points.at(i).x - carX), carYaw) > (M_PI / 2))
      continue;

    //Calculate waypoint distance to the car
    double distance = euclideanDistance(waypointSplineMarkers.points.at(startingWaypoint).x, carX, waypointSplineMarkers.points.at(startingWaypoint).y, carY);
    for (unsigned int j = startingWaypoint; j < i; j++)
    {
      distance += euclideanDistance(waypointSplineMarkers.points.at(j).x, waypointSplineMarkers.points.at(j + 1).x, waypointSplineMarkers.points.at(j).y, waypointSplineMarkers.points.at(j + 1).y);
    }
    //first waypoint that is far enough to have a good responce for the speed
    if (distance < speedDistance)
    {
      speedWaypointId = i;
    }

    //the first waypoint far at least lookForward is good
    if (distance > lookForward)
    {
      targetWaypointId = i;
      break;
    }
  }

  //Remember last waypoint id selected
  lastWaypoint = speedWaypointId;

  //Publish our target just to see it
  waypointTarget.pose.position.x = waypointSplineMarkers.points.at(targetWaypointId).x;
  waypointTarget.pose.position.y = waypointSplineMarkers.points.at(targetWaypointId).y;
  waypointTargetPub->publish(waypointTarget);
}

void PurePursuit::findTargetWaypointOpenTrack()
{

  lookForward = minLookForward + minLookForwardGain * static_cast<double>(vehicleActual.actual_speed);
  speedDistance = minSpeedDistance + minLookForwardGain * static_cast<double>(vehicleActual.actual_speed);

  //Calculate distance between waypoint and car
  double currDistance = euclideanDistance(waypointSplineMarkers.points.at(targetWaypointId).x, carX,
                                    waypointSplineMarkers.points.at(targetWaypointId).y, carY);
  if(currDistance < distToSwitchWaypoint)
  {
    targetWaypointId++;
    targetWaypointId = (targetWaypointId > (waypointSplineMarkers.points.size() - 1)) ?
                                            (waypointSplineMarkers.points.size() - 1) : targetWaypointId;

    std::cout << "Target waypoint ID : " << targetWaypointId << " / " << waypointSplineMarkers.points.size()-1 << std::endl;
  }
  speedWaypointId = targetWaypointId;

  //Publish our target just to see it
  waypointTarget.pose.position.x = waypointSplineMarkers.points.at(targetWaypointId).x;
  waypointTarget.pose.position.y = waypointSplineMarkers.points.at(targetWaypointId).y;
  waypointTargetPub->publish(waypointTarget);
}

inline void PurePursuit::calculateSteeringTarget()
{
  //Calculate delta between target direction and car Rotation
  double targetX = waypointSplineMarkers.points.at(targetWaypointId).x;
  double targetY = waypointSplineMarkers.points.at(targetWaypointId).y;
  double SteerTarget = normalizeAngle(atan2(targetY - carRearY, targetX - carRearX) - carYaw);

  //Calculate steer target rotation
  double wheelRotation = atan2(2 * carWheelbase * sin(SteerTarget) / (lookForward * steerGain), 1);

  //Cut off with respect to real steer car capability
  if (wheelRotation > this->maxSteer)
    wheelRotation = this->maxSteer;
  else if (wheelRotation < -this->maxSteer)
    wheelRotation = -this->maxSteer;
  carTarget.steering_angle = static_cast<float>(wheelRotation);
  
}

float PurePursuit::mengerCurvature(geometry_msgs::msg::Point prev, geometry_msgs::msg::Point curr, geometry_msgs::msg::Point next)
{
  // k = fabs(2*((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1))) / sqrt( ((x2-x1)**2+(y2-y1)**2) * ((x3-x2)**2+(y3-y2)**2) * ((x1-x3)**2+(y1-y3)**2) )
  float k = abs(2*((curr.x-prev.x)*(next.y-prev.y)-(curr.y-prev.y)*(next.x-prev.x))) / 
                  (sqrt((pow(curr.x-prev.x, 2)+pow(curr.y-prev.y, 2)) *
                        (pow(next.x-curr.x, 2)+pow(next.y-curr.y, 2)) *
                        (pow(prev.x-next.x, 2)+pow(prev.y-next.y, 2))) + 
                  1e-6);  // add an epsilon to avoid nan results

  return k;
}

float PurePursuit::polyeval_targetVelocity(float radius)
{
  float result = 0.0f;
  for (int i = 0; i < velocity_coeffs.size(); i++)
  {
      result += velocity_coeffs[i] * pow(radius, i);
  }
  return result;
}

inline void PurePursuit::calculateVelocityTarget()
{
  //if(lapCounter > 10) carTargetCommand.carVelocityTarget = 0;
  //if(carTargetCommand.carSteerTarget > 0.5 || carTargetCommand.carSteerTarget < -0.5)
  //  carTargetCommand.carVelocityTarget = staticVelocityTarget/1.5;
  //carTarget.speed = staticVelocityTarget;

  if (useDynamicTargetSpeed && raceStatus.current_lap > slowLaps) {
    auto nWaypoints = waypointSplineMarkers.points.size();
    auto prev = waypointSplineMarkers.points.at((speedWaypointId-1) % nWaypoints);
    auto curr = waypointSplineMarkers.points.at(speedWaypointId % nWaypoints);
    auto next = waypointSplineMarkers.points.at((speedWaypointId+1) % nWaypoints);

    float radius = 1 / mengerCurvature(prev, curr, next);
    radius = (radius > 100) ? 100 : radius;
    velocityTarget = polyeval_targetVelocity(radius) * targetSpeedWeight;
    this->smootedSpeed = smootedSpeed * (1 - this->k_smooth) + velocityTarget * this->k_smooth;
    velocityTarget =  this->smootedSpeed;
    velocityTarget = (velocityTarget > maxSpeed) ? maxSpeed : velocityTarget;
    velocityTarget = (velocityTarget < minSpeed) ? minSpeed : velocityTarget;

    if (isnan(velocityTarget)) {
      velocityTarget = minSpeed;
    }

    //std::cout << "Velocity Target : " << staticVelocityTarget << std::endl;
  }
}

inline void PurePursuit::calculateAcceleratorTarget()
{
  // *************************** CHECK STOPPING CONDITION ***************************
  if (stop) {
    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!! STOPPING THE CAR !!!!!!!!!!!!!!!!!!!! , laps done: %d %d", raceStatus.current_lap, stop);
    carTarget.speed = -1;

    float deltaX = abs(oldCarX - carX);
    float deltaY = abs(oldCarY - carY);

    if (deltaX < 0.0001f && deltaY < 0.0001f) {
      posCounter++;
    }
    else {
      posCounter = 0;
    }

    // If the vehicle remains in the same position for ten times, the mission is completed
    if (posCounter > 10) {
      rclcpp::shutdown();
    }
  } else {
    //PID control on accelleration -- not really a PID but only a Proportional for now
    carTarget.speed = static_cast<float>(((velocityTarget - static_cast<double>(vehicleActual.actual_speed)) / velocityTarget) * accelerationPGain);
    if (carTarget.speed > 1.0)
      carTarget.speed = 1.0;  // Prima era 0.0 WHY???
    else if (carTarget.speed < -1.0)
      carTarget.speed = -1.0;
  }
}

inline double PurePursuit::computeAngleDiff(double angle1, double angle2)
{
  double difference = angle2 - angle1;
  while (difference < -M_PI)
    difference += (2 * M_PI);
  while (difference > M_PI)
    difference -= (2 * M_PI);
  return fabs(difference);
}

int PurePursuit::getControlHz()
{
  return controlHz;
}
