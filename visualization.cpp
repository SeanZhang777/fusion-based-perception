#include "visualization.h"

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::FusionObjectList& data)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.points.resize(data.objs.size());
    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "rgb";
    cloud.channels[0].values.resize(data.objs.size());

    //generate some fake data for our point cloud
    for (unsigned int i = 0; i < data.objs.size(); ++i)
    {
        cloud.points[i].x = data.objs[i]->x;
        cloud.points[i].y = data.objs[i]->y;
        cloud.points[i].z = data.objs[i]->z;
        cloud.channels[0].values[i] = 255;
    }
    return cloud;
}

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::LiDARObjectList& data)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.points.resize(data.objs.size());
    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "rgb";
    cloud.channels[0].values.resize(data.objs.size());

    //generate some fake data for our point cloud
    for (unsigned int i = 0; i < data.objs.size(); ++i)
    {
        cloud.points[i].x = data.objs[i]->x;
        cloud.points[i].y = data.objs[i]->y;
        cloud.points[i].z = data.objs[i]->z;
        cloud.channels[0].values[i] = 255;
    }
    return cloud;
}

void PublishLidarObjects(
    const ros::Publisher &publisher, 
    const kit::perception::fusion::LiDARObjectListPtr &lidar_obj_list) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(lidar_obj_list->time_ns);

  //clear all markers before
  visualization_msgs::MarkerArray empty_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.id = 0;
  clear_marker.action = clear_marker.DELETEALL; 
  clear_marker.lifetime = ros::Duration();
  empty_markers.markers.push_back(clear_marker);
  publisher.publish(empty_markers);
  
  size_t num_objs = lidar_obj_list->objs.size();
  if (num_objs <= 0) {
    return;
  }

  visualization_msgs::MarkerArray object_markers;
  for (size_t i = 0; i < num_objs; ++i) {
    auto &objs = lidar_obj_list->objs;
    Eigen::Vector3d center(objs.at(i)->x, objs.at(i)->y, objs.at(i)->z);

    //object size
    const double& length = objs.at(i)->length;
    const double& width = objs.at(i)->width;
    const double& height = objs.at(i)->height;
    const double yaw = objs.at(i)->theta;
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
    Eigen::Vector3d bottom_quad[8];
    double half_l = length / 2;
    double half_w = width / 2;
    double h = height;
    //A(-half_l, -half_w)
    bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
    //B(-half_l, half_w)
    bottom_quad[1] = center + ldir * -half_l + odir * half_w;
    //C(half_l, half_w)
    bottom_quad[2] = center + ldir * half_l + odir * half_w;
    //D(half_l, -half_w)
    bottom_quad[3] = center + ldir * half_l + odir * -half_w;
    //top 4 vertices
    bottom_quad[4] = bottom_quad[0]; bottom_quad[4](2) += h;
    bottom_quad[5] = bottom_quad[1]; bottom_quad[5](2) += h;
    bottom_quad[6] = bottom_quad[2]; bottom_quad[6](2) += h;
    bottom_quad[7] = bottom_quad[3]; bottom_quad[7](2) += h;

    Eigen::MatrixXf OBB(8, 3);
    OBB <<  bottom_quad[0](0),bottom_quad[0](1),bottom_quad[0](2),
            bottom_quad[1](0),bottom_quad[1](1),bottom_quad[1](2),
            bottom_quad[2](0),bottom_quad[2](1),bottom_quad[2](2),
            bottom_quad[3](0),bottom_quad[3](1),bottom_quad[3](2),
            bottom_quad[4](0),bottom_quad[4](1),bottom_quad[4](2),
            bottom_quad[5](0),bottom_quad[5](1),bottom_quad[5](2),
            bottom_quad[6](0),bottom_quad[6](1),bottom_quad[6](2),
            bottom_quad[7](0),bottom_quad[7](1),bottom_quad[7](2);

    visualization_msgs::Marker box, text;
    text.action = visualization_msgs::Marker::ADD;
    box.header = text.header = header;
    box.ns = text.ns = "objects";
    box.id = i;
    text.id = i + num_objs;
    box.type = visualization_msgs::Marker::LINE_LIST;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    geometry_msgs::Point p[24];
    //Ground side
    //A->B
    p[0].x = OBB(0,0); p[0].y = OBB(0,1); p[0].z = OBB(0,2);
    p[1].x = OBB(1,0); p[1].y = OBB(1,1); p[1].z = OBB(1,2);
    //B->C
    p[2].x = OBB(1,0); p[2].y = OBB(1,1); p[2].z = OBB(1,2);
    p[3].x = OBB(2,0); p[3].y = OBB(2,1); p[3].z = OBB(2,2);
    //C->D
    p[4].x = OBB(2,0); p[4].y = OBB(2,1); p[4].z = OBB(2,2);
    p[5].x = OBB(3,0); p[5].y = OBB(3,1); p[5].z = OBB(3,2);
    //D->A
    p[6].x = OBB(3,0); p[6].y = OBB(3,1); p[6].z = OBB(3,2);
    p[7].x = OBB(0,0); p[7].y = OBB(0,1); p[7].z = OBB(0,2);

    //Top side
    //E->F
    p[8].x = OBB(4,0); p[8].y = OBB(4,1); p[8].z = OBB(4,2);
    p[9].x = OBB(5,0); p[9].y = OBB(5,1); p[9].z = OBB(5,2);
    //F->G
    p[10].x= OBB(5,0); p[10].y= OBB(5,1); p[10].z= OBB(5,2);
    p[11].x= OBB(6,0); p[11].y= OBB(6,1); p[11].z= OBB(6,2);
    //G->H
    p[12].x= OBB(6,0); p[12].y= OBB(6,1); p[12].z= OBB(6,2);
    p[13].x= OBB(7,0); p[13].y= OBB(7,1); p[13].z= OBB(7,2);
    //H->E
    p[14].x= OBB(7,0); p[14].y= OBB(7,1); p[14].z= OBB(7,2);
    p[15].x= OBB(4,0); p[15].y= OBB(4,1); p[15].z= OBB(4,2);

    //Around side
    //A->E
    p[16].x= OBB(0,0); p[16].y= OBB(0,1); p[16].z= OBB(0,2);
    p[17].x= OBB(4,0); p[17].y= OBB(4,1); p[17].z= OBB(4,2);
    //B->F
    p[18].x= OBB(1,0); p[18].y= OBB(1,1); p[18].z= OBB(1,2);
    p[19].x= OBB(5,0); p[19].y= OBB(5,1); p[19].z= OBB(5,2);
    //C->G
    p[20].x= OBB(2,0); p[20].y= OBB(2,1); p[20].z= OBB(2,2);
    p[21].x= OBB(6,0); p[21].y= OBB(6,1); p[21].z= OBB(6,2);
    //D->H
    p[22].x= OBB(3,0); p[22].y= OBB(3,1); p[22].z= OBB(3,2);
    p[23].x= OBB(7,0); p[23].y= OBB(7,1); p[23].z= OBB(7,2);

    for (size_t pi = 0u; pi < 24; ++pi) {
        box.points.push_back(p[pi]);
    }
    box.scale.x = 0.1;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 0;
    color.g = 1;
    color.b = 0;
    box.color = color;
    object_markers.markers.push_back(box);

    //text
    geometry_msgs::Pose pose;
    pose.position.x = center(0);
    pose.position.y = center(1);
    pose.position.z = center(2) + height * 1.5;

    std::ostringstream str;
    str << "id: " << objs.at(i)->id << std::endl;
    text.pose = pose;
    text.text = str.str();
    text.color.a = 1;
    text.color.r = 0.0;
    text.color.g = 1.0;
    text.color.b = 0.0;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;
    object_markers.markers.push_back(text);

    // velocity arrow
    visualization_msgs::Marker vel_arrow;
    vel_arrow.action = visualization_msgs::Marker::ADD;
    vel_arrow.header = header;
    vel_arrow.ns = "objects";
    vel_arrow.id = i + num_objs * 2;
    vel_arrow.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point start_point, end_point;
    start_point.x = center(0);
    start_point.y = center(1);
    start_point.z = center(2);
    Eigen::Vector3d velocity(objs.at(i)->velo_x, objs.at(i)->velo_y, objs.at(i)->velo_z);
    Eigen::Vector3d end = center + velocity / 3.0;
    end_point.x = end[0];
    end_point.y = end[1];
    end_point.z = end[2];
    vel_arrow.points.push_back(start_point);
    vel_arrow.points.push_back(end_point);
    vel_arrow.scale.x = 0.1;
    vel_arrow.scale.y = 0.2;
    vel_arrow.scale.z = length * 0.1;
    vel_arrow.color.a = 1.0;
    vel_arrow.color.r = 0.0;
    vel_arrow.color.g = 1.0;
    vel_arrow.color.b = 0.0;
    object_markers.markers.push_back(vel_arrow);
  }
  publisher.publish(object_markers);
}


void PublishCameraObjects(
  const ros::Publisher &publisher, 
  const kit::perception::fusion::CameraObjectListPtr &camera_obj_list) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(camera_obj_list->time_ns);

  //clear all markers before
  visualization_msgs::MarkerArray empty_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.id = 0;
  clear_marker.action = clear_marker.DELETEALL; 
  clear_marker.lifetime = ros::Duration();
  empty_markers.markers.push_back(clear_marker);
  publisher.publish(empty_markers);
  
  size_t num_objs = camera_obj_list->objs.size();
  if (num_objs <= 0) {
    return;
  }

  visualization_msgs::MarkerArray object_markers;
  for (size_t i = 0; i < num_objs; ++i) {
    auto &objs = camera_obj_list->objs;
    Eigen::Vector3d center(objs.at(i)->x, objs.at(i)->y, objs.at(i)->z);

    //object size
    const double& length = objs.at(i)->length;
    const double& width = objs.at(i)->width;
    const double& height = objs.at(i)->height;
    const double yaw = objs.at(i)->theta;
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
    Eigen::Vector3d bottom_quad[8];
    double half_l = length / 2;
    double half_w = width / 2;
    double h = height;
    //A(-half_l, -half_w)
    bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
    //B(-half_l, half_w)
    bottom_quad[1] = center + ldir * -half_l + odir * half_w;
    //C(half_l, half_w)
    bottom_quad[2] = center + ldir * half_l + odir * half_w;
    //D(half_l, -half_w)
    bottom_quad[3] = center + ldir * half_l + odir * -half_w;
    //top 4 vertices
    bottom_quad[4] = bottom_quad[0]; bottom_quad[4](2) += h;
    bottom_quad[5] = bottom_quad[1]; bottom_quad[5](2) += h;
    bottom_quad[6] = bottom_quad[2]; bottom_quad[6](2) += h;
    bottom_quad[7] = bottom_quad[3]; bottom_quad[7](2) += h;

    Eigen::MatrixXf OBB(8, 3);
    OBB <<  bottom_quad[0](0),bottom_quad[0](1),bottom_quad[0](2),
            bottom_quad[1](0),bottom_quad[1](1),bottom_quad[1](2),
            bottom_quad[2](0),bottom_quad[2](1),bottom_quad[2](2),
            bottom_quad[3](0),bottom_quad[3](1),bottom_quad[3](2),
            bottom_quad[4](0),bottom_quad[4](1),bottom_quad[4](2),
            bottom_quad[5](0),bottom_quad[5](1),bottom_quad[5](2),
            bottom_quad[6](0),bottom_quad[6](1),bottom_quad[6](2),
            bottom_quad[7](0),bottom_quad[7](1),bottom_quad[7](2);

    visualization_msgs::Marker box, text;
    text.action = visualization_msgs::Marker::ADD;
    box.header = text.header = header;
    box.ns = text.ns = "objects";
    box.id = i;
    text.id = i + num_objs;
    box.type = visualization_msgs::Marker::LINE_LIST;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    geometry_msgs::Point p[24];
    //Ground side
    //A->B
    p[0].x = OBB(0,0); p[0].y = OBB(0,1); p[0].z = OBB(0,2);
    p[1].x = OBB(1,0); p[1].y = OBB(1,1); p[1].z = OBB(1,2);
    //B->C
    p[2].x = OBB(1,0); p[2].y = OBB(1,1); p[2].z = OBB(1,2);
    p[3].x = OBB(2,0); p[3].y = OBB(2,1); p[3].z = OBB(2,2);
    //C->D
    p[4].x = OBB(2,0); p[4].y = OBB(2,1); p[4].z = OBB(2,2);
    p[5].x = OBB(3,0); p[5].y = OBB(3,1); p[5].z = OBB(3,2);
    //D->A
    p[6].x = OBB(3,0); p[6].y = OBB(3,1); p[6].z = OBB(3,2);
    p[7].x = OBB(0,0); p[7].y = OBB(0,1); p[7].z = OBB(0,2);

    //Top side
    //E->F
    p[8].x = OBB(4,0); p[8].y = OBB(4,1); p[8].z = OBB(4,2);
    p[9].x = OBB(5,0); p[9].y = OBB(5,1); p[9].z = OBB(5,2);
    //F->G
    p[10].x= OBB(5,0); p[10].y= OBB(5,1); p[10].z= OBB(5,2);
    p[11].x= OBB(6,0); p[11].y= OBB(6,1); p[11].z= OBB(6,2);
    //G->H
    p[12].x= OBB(6,0); p[12].y= OBB(6,1); p[12].z= OBB(6,2);
    p[13].x= OBB(7,0); p[13].y= OBB(7,1); p[13].z= OBB(7,2);
    //H->E
    p[14].x= OBB(7,0); p[14].y= OBB(7,1); p[14].z= OBB(7,2);
    p[15].x= OBB(4,0); p[15].y= OBB(4,1); p[15].z= OBB(4,2);

    //Around side
    //A->E
    p[16].x= OBB(0,0); p[16].y= OBB(0,1); p[16].z= OBB(0,2);
    p[17].x= OBB(4,0); p[17].y= OBB(4,1); p[17].z= OBB(4,2);
    //B->F
    p[18].x= OBB(1,0); p[18].y= OBB(1,1); p[18].z= OBB(1,2);
    p[19].x= OBB(5,0); p[19].y= OBB(5,1); p[19].z= OBB(5,2);
    //C->G
    p[20].x= OBB(2,0); p[20].y= OBB(2,1); p[20].z= OBB(2,2);
    p[21].x= OBB(6,0); p[21].y= OBB(6,1); p[21].z= OBB(6,2);
    //D->H
    p[22].x= OBB(3,0); p[22].y= OBB(3,1); p[22].z= OBB(3,2);
    p[23].x= OBB(7,0); p[23].y= OBB(7,1); p[23].z= OBB(7,2);

    for (size_t pi = 0u; pi < 24; ++pi) {
        box.points.push_back(p[pi]);
    }
    box.scale.x = 0.1;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    box.color = color;
    object_markers.markers.push_back(box);

    //text
    geometry_msgs::Pose pose;
    pose.position.x = center(0);
    pose.position.y = center(1);
    pose.position.z = center(2) + height * 1.5;

    std::ostringstream str;
    str << "id: " << objs.at(i)->id << std::endl;
    text.pose = pose;
    text.text = str.str();
    text.color.a = 1;
    text.color.r = 1.0;
    text.color.g = 0.0;
    text.color.b = 0.0;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;
    object_markers.markers.push_back(text);

    // velocity arrow
    visualization_msgs::Marker vel_arrow;
    vel_arrow.action = visualization_msgs::Marker::ADD;
    vel_arrow.header = header;
    vel_arrow.ns = "objects";
    vel_arrow.id = i + num_objs * 2;
    vel_arrow.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point start_point, end_point;
    start_point.x = center(0);
    start_point.y = center(1);
    start_point.z = center(2);
    Eigen::Vector3d velocity(objs.at(i)->velo_x, objs.at(i)->velo_y, objs.at(i)->velo_z);
    Eigen::Vector3d end = center + velocity / 3.0;
    end_point.x = end[0];
    end_point.y = end[1];
    end_point.z = end[2];
    vel_arrow.points.push_back(start_point);
    vel_arrow.points.push_back(end_point);
    vel_arrow.scale.x = 0.1;
    vel_arrow.scale.y = 0.2;
    vel_arrow.scale.z = length * 0.1;
    vel_arrow.color.a = 1.0;
    vel_arrow.color.r = 1.0;
    vel_arrow.color.g = 0.0;
    vel_arrow.color.b = 0.0;
    object_markers.markers.push_back(vel_arrow);
  }
  publisher.publish(object_markers);
}

void PublishFusionObjects(
  const ros::Publisher &publisher, 
  const kit::perception::fusion::FusionObjectListPtr &fusion_obj_list) {
std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(fusion_obj_list->time_ns);

  //clear all markers before
  visualization_msgs::MarkerArray empty_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.id = 0;
  clear_marker.action = clear_marker.DELETEALL; 
  clear_marker.lifetime = ros::Duration();
  empty_markers.markers.push_back(clear_marker);
  publisher.publish(empty_markers);
  
  size_t num_objs = fusion_obj_list->objs.size();
  if (num_objs <= 0) {
    return;
  }

  visualization_msgs::MarkerArray object_markers;
  for (size_t i = 0; i < num_objs; ++i) {
    auto &objs = fusion_obj_list->objs;
    Eigen::Vector3d center(objs.at(i)->x, objs.at(i)->y, objs.at(i)->z);

    //object size
    const double& length = objs.at(i)->length;
    const double& width = objs.at(i)->width;
    const double& height = objs.at(i)->height;
    const double yaw = objs.at(i)->theta;
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
    Eigen::Vector3d bottom_quad[8];
    double half_l = length / 2;
    double half_w = width / 2;
    double h = height;
    //A(-half_l, -half_w)
    bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
    //B(-half_l, half_w)
    bottom_quad[1] = center + ldir * -half_l + odir * half_w;
    //C(half_l, half_w)
    bottom_quad[2] = center + ldir * half_l + odir * half_w;
    //D(half_l, -half_w)
    bottom_quad[3] = center + ldir * half_l + odir * -half_w;
    //top 4 vertices
    bottom_quad[4] = bottom_quad[0]; bottom_quad[4](2) += h;
    bottom_quad[5] = bottom_quad[1]; bottom_quad[5](2) += h;
    bottom_quad[6] = bottom_quad[2]; bottom_quad[6](2) += h;
    bottom_quad[7] = bottom_quad[3]; bottom_quad[7](2) += h;

    Eigen::MatrixXf OBB(8, 3);
    OBB <<  bottom_quad[0](0),bottom_quad[0](1),bottom_quad[0](2),
            bottom_quad[1](0),bottom_quad[1](1),bottom_quad[1](2),
            bottom_quad[2](0),bottom_quad[2](1),bottom_quad[2](2),
            bottom_quad[3](0),bottom_quad[3](1),bottom_quad[3](2),
            bottom_quad[4](0),bottom_quad[4](1),bottom_quad[4](2),
            bottom_quad[5](0),bottom_quad[5](1),bottom_quad[5](2),
            bottom_quad[6](0),bottom_quad[6](1),bottom_quad[6](2),
            bottom_quad[7](0),bottom_quad[7](1),bottom_quad[7](2);

    visualization_msgs::Marker box, text;
    text.action = visualization_msgs::Marker::ADD;
    box.header = text.header = header;
    box.ns = text.ns = "objects";
    box.id = i;
    text.id = i + num_objs;
    box.type = visualization_msgs::Marker::LINE_LIST;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    geometry_msgs::Point p[24];
    //Ground side
    //A->B
    p[0].x = OBB(0,0); p[0].y = OBB(0,1); p[0].z = OBB(0,2);
    p[1].x = OBB(1,0); p[1].y = OBB(1,1); p[1].z = OBB(1,2);
    //B->C
    p[2].x = OBB(1,0); p[2].y = OBB(1,1); p[2].z = OBB(1,2);
    p[3].x = OBB(2,0); p[3].y = OBB(2,1); p[3].z = OBB(2,2);
    //C->D
    p[4].x = OBB(2,0); p[4].y = OBB(2,1); p[4].z = OBB(2,2);
    p[5].x = OBB(3,0); p[5].y = OBB(3,1); p[5].z = OBB(3,2);
    //D->A
    p[6].x = OBB(3,0); p[6].y = OBB(3,1); p[6].z = OBB(3,2);
    p[7].x = OBB(0,0); p[7].y = OBB(0,1); p[7].z = OBB(0,2);

    //Top side
    //E->F
    p[8].x = OBB(4,0); p[8].y = OBB(4,1); p[8].z = OBB(4,2);
    p[9].x = OBB(5,0); p[9].y = OBB(5,1); p[9].z = OBB(5,2);
    //F->G
    p[10].x= OBB(5,0); p[10].y= OBB(5,1); p[10].z= OBB(5,2);
    p[11].x= OBB(6,0); p[11].y= OBB(6,1); p[11].z= OBB(6,2);
    //G->H
    p[12].x= OBB(6,0); p[12].y= OBB(6,1); p[12].z= OBB(6,2);
    p[13].x= OBB(7,0); p[13].y= OBB(7,1); p[13].z= OBB(7,2);
    //H->E
    p[14].x= OBB(7,0); p[14].y= OBB(7,1); p[14].z= OBB(7,2);
    p[15].x= OBB(4,0); p[15].y= OBB(4,1); p[15].z= OBB(4,2);

    //Around side
    //A->E
    p[16].x= OBB(0,0); p[16].y= OBB(0,1); p[16].z= OBB(0,2);
    p[17].x= OBB(4,0); p[17].y= OBB(4,1); p[17].z= OBB(4,2);
    //B->F
    p[18].x= OBB(1,0); p[18].y= OBB(1,1); p[18].z= OBB(1,2);
    p[19].x= OBB(5,0); p[19].y= OBB(5,1); p[19].z= OBB(5,2);
    //C->G
    p[20].x= OBB(2,0); p[20].y= OBB(2,1); p[20].z= OBB(2,2);
    p[21].x= OBB(6,0); p[21].y= OBB(6,1); p[21].z= OBB(6,2);
    //D->H
    p[22].x= OBB(3,0); p[22].y= OBB(3,1); p[22].z= OBB(3,2);
    p[23].x= OBB(7,0); p[23].y= OBB(7,1); p[23].z= OBB(7,2);

    for (size_t pi = 0u; pi < 24; ++pi) {
        box.points.push_back(p[pi]);
    }
    box.scale.x = 0.1;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    box.color = color;
    object_markers.markers.push_back(box);

    //text
    geometry_msgs::Pose pose;
    pose.position.x = center(0);
    pose.position.y = center(1);
    pose.position.z = center(2) + height * 1.5;

    std::ostringstream str;
    str << "id: " << objs.at(i)->id << std::endl;
    text.pose = pose;
    text.text = str.str();
    text.color.a = 1;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;
    object_markers.markers.push_back(text);

    // velocity arrow
    visualization_msgs::Marker vel_arrow;
    vel_arrow.action = visualization_msgs::Marker::ADD;
    vel_arrow.header = header;
    vel_arrow.ns = "objects";
    vel_arrow.id = i + num_objs * 2;
    vel_arrow.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point start_point, end_point;
    start_point.x = center(0);
    start_point.y = center(1);
    start_point.z = center(2);
    Eigen::Vector3d velocity(objs.at(i)->velo_x, objs.at(i)->velo_y, objs.at(i)->velo_z);
    Eigen::Vector3d end = center + velocity / 3.0;
    end_point.x = end[0];
    end_point.y = end[1];
    end_point.z = end[2];
    vel_arrow.points.push_back(start_point);
    vel_arrow.points.push_back(end_point);
    vel_arrow.scale.x = 0.1;
    vel_arrow.scale.y = 0.2;
    vel_arrow.scale.z = length * 0.1;
    vel_arrow.color.a = 1.0;
    vel_arrow.color.r = 1.0;
    vel_arrow.color.g = 1.0;
    vel_arrow.color.b = 1.0;
    object_markers.markers.push_back(vel_arrow);
  }
  publisher.publish(object_markers);
}