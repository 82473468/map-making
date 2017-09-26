//
// Created by wenchi on 17-8-21.
//

#ifndef PROJECT_MAP_CREATE_H
#define PROJECT_MAP_CREATE_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "ros/package.h"
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

class MapCreate {
public:
    MapCreate(ros::NodeHandle nh,
    ros::NodeHandle pri_nh);
    ~MapCreate(){}

private:
    ros::Publisher pcd_pub;
    ros::Publisher gps_pub;

    bool background_filter;

};


#endif //PROJECT_MAP_CREATE_H
