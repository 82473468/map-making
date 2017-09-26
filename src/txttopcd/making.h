//
// Created by wenchi on 17-7-13.
//

#ifndef PROJECT_MAKING_H
#define PROJECT_MAKING_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "ros/package.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

class Making {
public:
    Making(ros::NodeHandle nh,
    ros::NodeHandle pri_nh);
    ~Making(){}

private:
    ros::Publisher pcd_pub;
    ros::Publisher raw_pub;

    void readTxt(std::string txt_path,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);


    void labelbyDis(pcl::PointCloud<pcl::PointXYZI> cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,float distance_threshold);


    void deletePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_2,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,float distance);



};


#endif //PROJECT_MAKING_H
