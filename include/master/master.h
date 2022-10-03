#ifndef MASTER_H_
#define MASTER_H_

#include "ros/ros.h"
#include "master/Vision.h"

//---Subscriber---
//================
ros::Subscriber sub_pc2bs;
ros::Subscriber sub_vision_data;
//---Publisher---


//--Prototypes
//============
void CllbckVisionData(const master::VisionConstPtr &msg);
#endif