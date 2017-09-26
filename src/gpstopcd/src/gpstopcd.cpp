//
// Created by wenchi on 17-8-9.
//

#include "gpstopcd.h"
gpstopcd::gpstopcd(ros::NodeHandle nh, ros::NodeHandle private_nh) {

    sub_gps = nh.subscribe("gps/fix", 10, &gpstopcd::gps_callback, (gpstopcd*) this);

    path_pub = nh.advertise<nav_msgs::Path>("/gps_path",1,true);

    gps_frames=0;

    path.header.frame_id="odom";

    std::string pkg_Path = ros::package::getPath("gpstopcd");
    std::string folder_path = pkg_Path + "/data/";
    if (!fopen(folder_path.c_str(), "wb"))
    {
        mkdir(folder_path.c_str(), S_IRWXU);
    }
    private_nh.getParam("gpspcd_name", gpspcd_name);
    fileName = folder_path + gpspcd_name;
    std::cout<<"the path is "<<fileName<<std::endl;

}



void gpstopcd::gps_callback(const sensor_msgs::NavSatFix& gps_msg){
    //××××××××处理GPS数据

    if(gps_frames==0) {
        ori_lat = gps_msg.latitude;
        ori_lon = gps_msg.longitude;
        ori_ati = 7;
    }
//    ori_lat = 39.9680502;
//    ori_lon = 116.3045885;
//    ori_ati = 7;
    gps_frames++;
    if(gps_frames>100)
        gps_frames=1;
    double latitude_4=gps_msg.latitude;
    double longitude_6=gps_msg.longitude;

    std::cout.precision(10);
    std::cout<<"the latitude is "<<latitude_4<<", the longitude is "<<longitude_6<<std::endl;

    Eigen::Vector3d gps;
    gps<<longitude_6,latitude_4,10;
    Eigen::Vector3d ret;
    ret=WGS84toECEF(gps);

    double rad_lon=ori_lon/180*PI;
    double rad_lat=ori_lat/180*PI;
    double sin_lon=sin(rad_lon);
    double cos_lon=cos(rad_lon);
    double sin_lat=sin(rad_lat);
    double cos_lat=cos(rad_lat);

    Eigen::Matrix3d rot;

    rot<<-sin_lon,cos_lon,0,
            -sin_lat*cos_lon,-sin_lat*sin_lon,cos_lat,
            cos_lat*cos_lon,cos_lat*sin_lon,0;

    Eigen::Vector3d z0;
    Eigen::Vector3d wgs0;
    wgs0<<ori_lon,ori_lat,ori_ati;
    z0=WGS84toECEF(wgs0);
    Eigen::Vector3d l;
    l=ret-z0;
    XYZ=rot*l;

    geometry_msgs::PoseStamped odom;

    odom.header.stamp = gps_msg.header.stamp;
    odom.header.frame_id = "odom";
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

    //set the position
    odom.pose.position.x = XYZ[0];
    odom.pose.position.y = XYZ[1];
    odom.pose.position.z = 0.0;
    odom.pose.orientation = odom_quat;
    path.poses.push_back(odom);
    path_pub.publish(path);

    pcl::PointXYZI tmp_point;

    tmp_point.x = XYZ[0];

    tmp_point.y = XYZ[1];

    tmp_point.z = 0;

    trajectory.push_back(tmp_point);

    trajectory.height = 1;

    trajectory.width = trajectory.size();

    pcl::io::savePCDFileBinary(fileName, trajectory);
}


Eigen::Vector3d gpstopcd::WGS84toECEF(Eigen::Vector3d gps){
    double SEMI_MAJOR_AXIS = 6378137.0;
    double RECIPROCAL_OF_FLATTENING = 298.257223563;
    double SEMI_MINOR_AXIS = 6356752.3142;
    double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
    double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

    double lon=gps[0];
    double lat=gps[1];
    double ati=gps[2];

    double rad_lon=lon/180*PI;
    double rad_lat=lat/180*PI;

    double sin_lon=sin(rad_lon);
    double cos_lon=cos(rad_lon);
    double sin_lat=sin(rad_lat);
    double cos_lat=cos(rad_lat);

    double chi=sqrt(1.0-FIRST_ECCENTRICITY_SQUARED*sin_lat*sin_lat);
    double N=SEMI_MAJOR_AXIS/chi+ati;

    Eigen::Vector3d ret;
    ret<<N*cos_lat*cos_lon,N*cos_lat*sin_lon,(SEMI_MAJOR_AXIS*(1.0-FIRST_ECCENTRICITY_SQUARED)/chi+ati)*sin_lat;

    return ret;
}




int main(int argc, char** argv)
{

    ros::init(argc, argv, "test");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    gpstopcd loc(nh,private_nh);

    ros::spin();

    return 0;

}
