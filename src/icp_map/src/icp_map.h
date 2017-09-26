//
// Created by wenchi on 17-9-14.
//

#ifndef PROJECT_ICP_MAP_H
#define PROJECT_ICP_MAP_H
#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include <dirent.h>
#include <opencv/cv.h>


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

class locMap
{

  public:
  locMap(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~locMap() {}

  void readTxt(std::string txt_path,std::vector<cv::Point2f>& loc_point_vec);
  
  ros::Publisher g_wholemap_pub;
  ros::Publisher gps_pub;
};

class icp_map
{

};


#endif //PROJECT_ICP_MAP_H
