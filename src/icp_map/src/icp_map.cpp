//
// Created by wenchi on 17-9-14.
//

#include "icp_map.h"

typedef  PointMatcher<float> PM;
typedef  PM::DataPoints DP;


locMap::locMap(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

  //利用高度z存储对应信息，1.02为杆状物地图，1.03为odom轨迹，1.04为gps原点，1.05为icp匹配点
  g_wholemap_pub = nh.advertise<sensor_msgs::PointCloud2>("/rs_map", 1, true);
  gps_pub = nh.advertise<sensor_msgs::PointCloud2>("/rs_gps", 1,true);
  std::vector<cv::Point2f> loc_point_vec;
  std::string map_str;
  private_nh.getParam("map_str",map_str);
  std::string filter_str;
  private_nh.getParam("filter_str",filter_str);
  std::string txt_str;
  private_nh.getParam("txt_str",txt_str);
  readTxt(txt_str,loc_point_vec);
  loc_point_vec.pop_back();
  DP data(DP::load(map_str));
  DP all_map = data.createSimilarEmpty();
  PM::DataPointsFilters locmap_filter_;
  std::ifstream locmap_ifs(filter_str.c_str());
  locmap_filter_  = PM::DataPointsFilters(locmap_ifs);
  locmap_filter_.apply(data);

  DP background_map = data;

  int new_ptcount=0;
  for(int i=0;i<background_map.features.cols();++i)
  {
    if(background_map.features(2,i)<-1&&background_map.features(2,i)>-1.1)
      background_map.features(2,i)=-1;
    all_map.setColFrom(new_ptcount, background_map, i);
    new_ptcount++;
  }
  std::cout<<"the size of background is "<<new_ptcount<<std::endl;
  DP data3 = data.createSimilarEmpty();
  std::string odomvtk_name;
  std::string polesvtk_name;
  std::string gpsvtk_name;
  std::string mapvtk_name;

  private_nh.getParam("odomvtk_name", odomvtk_name);
  private_nh.getParam("polesvtk_name", polesvtk_name);
  private_nh.getParam("gpsvtk_name", gpsvtk_name);
  private_nh.getParam("mapvtk_name", mapvtk_name);

  float latitude=0;
  float longitude=0;
  private_nh.getParam("latitude", latitude);
  private_nh.getParam("longitude", longitude);
  latitude=latitude;
  longitude=longitude;
  std::cout.precision(10);
  std::cout<<"the latitude and longitude is "<<latitude<<","<<longitude<<std::endl;
  std::cout<<"the poles begein "<<std::endl;
  DP poles_vtk(DP::load(polesvtk_name));
  int trees=0;
  for(int i=0;i<poles_vtk.features.cols();++i)
  {
    poles_vtk.features(2,i)=-1.02;
    all_map.setColFrom(new_ptcount, poles_vtk, i);
    new_ptcount++;
    trees++;
  }
  std::cout<<"the trees is "<<trees<<std::endl;
  std::cout<<"the odom begein "<<std::endl;
  DP odom_vtk(DP::load(odomvtk_name));
  for(int i=0;i<odom_vtk.features.cols();++i)
  {
    odom_vtk.features(2,i)=-1.03+odom_vtk.features(2,i)/1000.;
    all_map.setColFrom(new_ptcount, odom_vtk, i);
    new_ptcount++;
  }
  std::cout<<"the size of all is "<<new_ptcount<<std::endl;
  float theta=0;
  float x_offset = 0;
  float y_offset = 0;
  private_nh.getParam("theta", theta);
  theta = (theta/180.)*M_PI;
  private_nh.getParam("x_offset", x_offset);
  private_nh.getParam("y_offset", y_offset);
  std::cout<<"the theta is "<<theta<<std::endl;
  PM::TransformationParameters Trans;
  Trans = PM::TransformationParameters::Identity(4,4);
  Trans(0, 0) = cosf(theta);
  Trans(0, 1) = -sinf(theta);
  Trans(1, 0) = sinf(theta);
  Trans(1, 1) = cosf(theta);
  Trans(0,3) = x_offset;
  Trans(1,3) = y_offset;
  all_map.features = Trans * all_map.features;

  for(int i=0;i<odom_vtk.features.cols();++i)
  {
    if(i<loc_point_vec.size())
    {
      std::cout<<loc_point_vec[i]<<std::endl;
      odom_vtk.features(0,i)=loc_point_vec[i].x;
      odom_vtk.features(1,i)=loc_point_vec[i].y;
      odom_vtk.features(2,i)=-1.05;
      odom_vtk.features(3, i) = 1;
      all_map.setColFrom(new_ptcount, odom_vtk, i);
      new_ptcount++;
    }
    if (i == odom_vtk.features.cols() - 1)
    {
      odom_vtk.features(0, i) = latitude;
      odom_vtk.features(1, i) = longitude;
      odom_vtk.features(2, i) = -1.04;
      odom_vtk.features(3, i) = 1;
      std::cout<<"the map gps is "<<odom_vtk.features(0,i)<<","<<odom_vtk.features(1,i)<<std::endl;

      all_map.setColFrom(new_ptcount, odom_vtk, i);
      new_ptcount++;
    }
  }

  all_map.conservativeResize(new_ptcount);
  std::cout<<"the size of map is "<<all_map.features.cols()<<std::endl;

  sensor_msgs::PointCloud2 g_wholemap_show;
  g_wholemap_show = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(all_map, "/odom", ros::Time::now());
  g_wholemap_pub.publish(g_wholemap_show);
  all_map.save(mapvtk_name,true);


  int tree_points=0;
  for(int i=0;i<all_map.features.cols();++i)
  {
    if(all_map.features(2,i)<-1.015&&all_map.features(2,i)>-1.025)
    {
      tree_points++;
//      std::cout<<"find one tree!"<<std::endl;
    }
  }
  std::cout<<"the size of tree is "<<tree_points<<std::endl;

  DP gps_vtk(DP::load(gpsvtk_name));
  sensor_msgs::PointCloud2 gps_show;
  gps_show = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(gps_vtk, "/odom", ros::Time::now());
  gps_pub.publish(gps_show);

}


void locMap::readTxt(std::string txt_path,std::vector<cv::Point2f>& loc_point_vec){
  std::ifstream filename;
  filename.open(txt_path.c_str());
  std::string line;
  std::cout<<"the txt path is "<<txt_path<<std::endl;
  loc_point_vec.clear();
  while(!filename.eof()){
    getline(filename,line);
    std::istringstream iss(line);
    float data[2];
    std::string sub_line;
    int count=0;
    while(iss>>sub_line){
      data[count++]=atof(sub_line.c_str());
    }
    cv::Point2f temp_point;
    temp_point.x = data[0];
    temp_point.y = data[1];
    loc_point_vec.push_back(temp_point);
  }
}



int main(int argc, char** argv)
{


  ros::init(argc, argv, "demo");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  locMap poles(nh, private_nh);         //extract features for localization

  ros::spin();

  return 0;
}