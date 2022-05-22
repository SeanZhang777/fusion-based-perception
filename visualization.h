#pragma once
#include "fusion/object.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "input_data_type.h"

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::FusionObjectList& data);
sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::LiDARObjectList& data);

void PublishLidarObjects(
  const ros::Publisher &publisher, 
  const kit::perception::fusion::LiDARObjectListPtr &lidar_obj_list);
void PublishCameraObjects(
  const ros::Publisher &publisher, 
  const kit::perception::fusion::CameraObjectListPtr &camera_obj_list);
void PublishFusionObjects(
  const ros::Publisher &publisher, 
  const kit::perception::fusion::FusionObjectListPtr &fusion_obj_list);
