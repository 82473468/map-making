//
// Created by wenchi on 17-8-21.
//

#include "map_create.h"


void publishCloud(const ros::Publisher* in_publisher,pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud)
{
    sensor_msgs::PointCloud2 output_poles;
    pcl::toROSMsg(*poles_cloud,output_poles);
    output_poles.header.frame_id="odom";
    in_publisher->publish(output_poles);
}



MapCreate::MapCreate(ros::NodeHandle nh, ros::NodeHandle pri_nh)
{
    std::string pkg_Path = ros::package::getPath("making3dmap");
    std::string folder_path = pkg_Path + "/data/";
    if (!fopen(folder_path.c_str(), "wb"))
    {
        mkdir(folder_path.c_str(), S_IRWXU);
    }
    std::string backgroudpcd_name;
    std::string odompcd_name;
    std::string polespcd_name;
    std::string gpspcd_name;
    std::string mappcd_name;

    int filter_label = 0;
    background_filter = false;

    pri_nh.getParam("backgroudpcd_name", backgroudpcd_name);
    pri_nh.getParam("odompcd_name", odompcd_name);
    pri_nh.getParam("polespcd_name", polespcd_name);
    pri_nh.getParam("gpspcd_name", gpspcd_name);
    pri_nh.getParam("mappcd_name", mappcd_name);
    pri_nh.getParam("filter_label", filter_label);

    if(filter_label ==0)
        background_filter = false;
    else
    {
        background_filter = true;
    }


    std::string pcd_path = folder_path + backgroudpcd_name;
    std::string pcd_path2 = folder_path + polespcd_name;
    std::string pcd_path3 = folder_path + odompcd_name;
    std::string gps_path = folder_path + gpspcd_name;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_val(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vix(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr gps_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_val(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);


    float theta=0;
    float x_offset = 0;
    float y_offset = 0;
    pri_nh.getParam("theta", theta);
    theta = (theta/180.)*M_PI;
    pri_nh.getParam("x_offset", x_offset);
    pri_nh.getParam("y_offset", y_offset);
    std::cout<<"the theta is "<<theta<<std::endl;
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    transform_1(0, 0) = cosf(theta);
    transform_1(0, 1) = -sinf(theta);
    transform_1(1, 0) = sinf(theta);
    transform_1(1, 1) = cosf(theta);
    transform_1(0, 3) = x_offset;
    transform_1(1, 3) = y_offset;


    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud_raw) == -1)
    {
        PCL_ERROR("read background file failure!");
    }

    for (int i = 0; i < cloud_raw->size(); ++i)
    {
        if ((pow(cloud_raw->points[i].x, 2) + pow(cloud_raw->points[i].y, 2)) < 0.1)
            continue;
        cloud_val->points.push_back(cloud_raw->points[i]);
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(cloud_val);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(8);
    outrem.filter(*cloud_trans);


    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_trans);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.filter(*cloud_vix);


    if(!background_filter)
        *cloud_vix= *cloud_val;


    for (int i = 0; i < cloud_vix->size(); ++i)
    {
        if ((cloud_vix->points[i].intensity > 100 && cloud_vix->points[i].intensity < 104))
            cloud_vix->points[i].intensity = 100;
        cloud->points.push_back(cloud_vix->points[i]);
    }
    std::cout << "the size of cloud is " << cloud->size() << std::endl;


    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path3, *cloud_height_val) == -1)
    {
        PCL_ERROR("read odom file failure!");
    }
    std::cout << "the size of odom is " << cloud_height_val->size() << std::endl;


    for (int i = 0; i < cloud_height_val->size(); ++i)
    {
        pcl::PointXYZI temp_point2;
        temp_point2.x = cloud_height_val->points[i].x;
        temp_point2.y = cloud_height_val->points[i].y;
        temp_point2.z = cloud_height_val->points[i].z;
        temp_point2.intensity = 102;
        cloud->points.push_back(temp_point2);
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path2, *cloud_2) == -1)
    {
        PCL_ERROR("read map file failure!");
    }
    std::cout << "the size of map is " << cloud_2->size() << std::endl;


    for (int i = 0; i < cloud_2->size(); ++i)
    {
        pcl::PointXYZI temp_point3;
        temp_point3.x = cloud_2->points[i].x;
        temp_point3.y = cloud_2->points[i].y;
        temp_point3.z = cloud_2->points[i].z;
        temp_point3.intensity = 101;
        cloud->points.push_back(temp_point3);
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZI>);


    pcl::transformPointCloud(*cloud, *cloud_result, transform_1);


    float latitude=0;
    float longitude=0;
    pri_nh.getParam("latitude", latitude);
    pri_nh.getParam("longitude", longitude);


    pcl::PointXYZI temp_point;
    temp_point.x = latitude;
    temp_point.y = longitude;
    temp_point.z = 0;
    temp_point.intensity = 103;
    std::cout<<"the gps is "<<latitude<<","<<longitude<<std::endl;
    cloud_result->points.push_back(temp_point);


    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("map_pcd",1,true);


    publishCloud(&pcd_pub,cloud_result);


    gps_pub = nh.advertise<sensor_msgs::PointCloud2>("gps_pcd",1,true);


    if (pcl::io::loadPCDFile<pcl::PointXYZI>(gps_path, *gps_cloud) == -1)
    {
        PCL_ERROR("read map file failure!");
    }


    publishCloud(&gps_pub,gps_cloud);


    cloud_result->width = 1;
    cloud_result->height = cloud_result->size();


    std::cout << "the map is done!" << std::endl;
    pcl::io::savePCDFileBinary(folder_path+mappcd_name, *cloud_result);


}





int main(int argc, char** argv) {

    ros::init(argc, argv, "mapping_node");

    ros::NodeHandle nh;

    ros::NodeHandle pri_nh("~");

    MapCreate map(nh,pri_nh);

    ros::spin();

    return 0;

}