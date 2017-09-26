//
// Created by wenchi on 17-8-9.
//

#ifndef PROJECT_GPSTOPCD_H
#define PROJECT_GPSTOPCD_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include "ros/package.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define PI 3.1415926

typedef Eigen::Vector3f pose;


class gpstopcd{
public:
    gpstopcd(ros::NodeHandle nh,
            ros::NodeHandle private_nh);
    ~gpstopcd(){}


    ros::Subscriber sub_gps;

    ros::Publisher path_pub;

    void gps_callback(const sensor_msgs::NavSatFix& gps_msg);


    Eigen::Vector3d WGS84toECEF(Eigen::Vector3d gps);

    Eigen::Vector3d XYZ;

    nav_msgs::Path path;

    std::string fileName;//车辆轨迹保存为pcd文件路径
    pcl::PointCloud<pcl::PointXYZI> trajectory;//车辆轨迹保存pcd

    double ori_lat;
    double ori_lon;
    double ori_ati;

    int gps_frames;

    std::string gpspcd_name;
};

#endif //PROJECT_GPSTOPCD_H
