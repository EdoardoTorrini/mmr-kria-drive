#pragma once

#include <math.h>

//_________________ REAL CAR CONFIGURATIONS _________________

//Car dimensions:
#define carWidth 1.431              //m
#define carLenght 2.960             //m
#define carWheelbase 1.541          //m
#define carFrontWheelToFromCG 0.815 //m //FROM CAR CG TO FRONT WHEEL
//#define carRearWheelToCG 0.726    //m //FROM CAR CG TO REAR WHEEL
//#define carRearWheelToCG 0.62     //m //FROM ZED CAMERA TO REAR WHEEL
#define carRearWheelToCG 0.75       //m //FROM XSENS IMU TO REAR WHEEL
#define noseToCG 1.73               //m Distance from CG to the nose of the veicle


//Other Car Specs:
//#define carMaxAccelleration       //m/s^2
//#define carMaxTorque              //Nm
#define carWeight 260               //Kg
#define carMinSpeed 2.8             //m/s ( 2.8 * 3.6 = 10.08km/h)
#define wheelExternalDiameter 0.430 //m

//Steering:
#define carMaxSteerAngle 0.3979351 //radiants == 22.8 degree measured on wheels
// #define carMaxSteerAngle 0.70 //radiants == 40 for sim
     
//#define carMaxSteerAngle 0.3490658      //radiants == 20 degree measured on wheels
// #define carMaxSteerAngle 0.3752458      //radiants == 21.5 degree measured on wheels
// #define carMaxSteerAngle 0.279253	//radiants == 16 degree measured on wheels
//#define carMaxSteerAngle 0.5236       //radiants ~= 31 degree measured on wheels
#define carMaxCurvatureAngle (tan(carMaxSteerAngle)/carWheelbase)

//Sensor positioning:
//Lidar:
//#define lidarXoffsetFromCG 1.555 //longitudinal distance between CAR CG and lidar
//#define lidarXoffsetFromCG 1.73    //longitudinal distance between ZED AS CG and lidar
#define lidarXoffsetFromCG 0    //longitudinal distance between ZED AS CG and lidar
#define lidarYoffsetFromCG 0        //lidar is in the middle of the two front tires
#define lidarZoffsetFromCG 0.15     //distance from ground

//Camera:
//#define cameraXoffsetFromCG 0
//#define cameraYoffsetFromCG 0
//#define cameraZoffsetFromCG 0

//Cone radius at lidar height:
#define coneRadius 0.04  //m used to compensate lidar ray travel to the center of the cone

//___________________________________________________________