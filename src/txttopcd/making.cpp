//
// Created by wenchi on 17-7-13.
//

#include "making.h"

struct label_point
{
    pcl::PointXYZI point;
    int label;
};


void publishCloud(const ros::Publisher* in_publisher,pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud)
{
    sensor_msgs::PointCloud2 output_poles;
    pcl::toROSMsg(*poles_cloud,output_poles);
    output_poles.header.frame_id="odom";
    in_publisher->publish(output_poles);
}


Making::Making(ros::NodeHandle nh, ros::NodeHandle pri_nh)
{
    std::string pkg_Path = ros::package::getPath("txttopcd");
    std::string folder_path = pkg_Path + "/data/";
    if (!fopen(folder_path.c_str(), "wb")) {
        mkdir(folder_path.c_str(), S_IRWXU);
    }
    std::string data_name;
    std::string pcd_name;
    std::string rawpcd_name;
    int txtorpcd=0;

    pri_nh.getParam("txtorpcd",txtorpcd);
    pri_nh.getParam("pcd_name",pcd_name);
    pri_nh.getParam("rawpcd_name",rawpcd_name);
    pri_nh.getParam("data_name", data_name);
    std::string data_path = folder_path + data_name;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_use(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if(txtorpcd==0)
    {
      readTxt(data_path, cloud_raw);
      std::cout << "the size of cloud is " << cloud_raw->size() << std::endl;
    }
    else
    {
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(data_path, *cloud_raw) == -1)
      {
        PCL_ERROR("read map file failure!");
      }
    }
    cloud_raw->width = 1;
    cloud_raw->height = cloud_raw->size();
    pcl::io::savePCDFileASCII(folder_path + rawpcd_name, *cloud_raw);
    raw_pub = nh.advertise<sensor_msgs::PointCloud2>("raw_pcd",1,true);
    publishCloud(&raw_pub,cloud_raw);
    for (int i = 0; i < cloud_raw->points.size(); ++i) {
        if(powf(cloud_raw->points[i].x,2)+powf(cloud_raw->points[i].y,2) < 0.1)
            continue;
        cloud_use->points.push_back(cloud_raw->points[i]);
    }
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(cloud_use);
    outrem.setRadiusSearch(0.4);
    outrem.setMinNeighborsInRadius(3);
    outrem.filter(*cloud);

    for (int i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].z = 0;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
    labelbyDis(*cloud, out_cloud_2, 0.6);
    std::cout<<"the size of out_cloud is "<<out_cloud_2->size()<<std::endl;
    deletePoints(out_cloud_2, out_cloud_3, 1);
    std::cout<<"the size of out_cloud_3 is "<<out_cloud_3->size()<<std::endl;
//    out_cloud = out_cloud_3;
    out_cloud = out_cloud_2;


    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("map_pcd",1,true);

    publishCloud(&pcd_pub,out_cloud);

    std::cout << "the size of out_cloud is " << out_cloud->size() << std::endl;

    out_cloud->width = 1;
    out_cloud->height = out_cloud->size();
    pcl::io::savePCDFileASCII(folder_path + pcd_name, *out_cloud);
}

void Making::deletePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_2,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,float distance){
    std::vector<float> vec_points;
    for(int i=0;i<out_cloud_2->size();++i) {
        int count=0;
        for (int j = 0; j < out_cloud_2->size(); ++j) {
            if (sqrtf(powf(out_cloud_2->points[i].x - out_cloud_2->points[j].x, 2) +
                      powf(out_cloud_2->points[i].y - out_cloud_2->points[j].y, 2)) < distance) {
                count++;
            }
        }
        if (count == 1) {
            pcl::PointXYZI tmp_point;
            tmp_point.x=out_cloud_2->points[i].x;
            tmp_point.y=out_cloud_2->points[i].y;
            tmp_point.z=out_cloud_2->points[i].z;
            out_cloud->points.push_back(tmp_point);
        }
    }

    for(int i=0;i<out_cloud_2->size();++i) {
        if (std::find(vec_points.begin(), vec_points.end(), out_cloud_2->points[i].x) == vec_points.end()) {
            for (int j = 0; j < out_cloud_2->size(); ++j) {
                if(out_cloud_2->points[i].x==out_cloud_2->points[j].x) continue;
                if (std::find(vec_points.begin(), vec_points.end(), out_cloud_2->points[j].x) == vec_points.end()) {
                    if (sqrtf(powf(out_cloud_2->points[i].x - out_cloud_2->points[j].x, 2) +
                              powf(out_cloud_2->points[i].y - out_cloud_2->points[j].y, 2)) < distance) {
                        vec_points.push_back(out_cloud_2->points[j].x);
                        if (out_cloud_2->points[i].intensity > out_cloud_2->points[j].intensity) {
                            if (std::find(vec_points.begin(), vec_points.end(), out_cloud_2->points[i].x) == vec_points.end()) {
                                pcl::PointXYZI tmp_point;
                                tmp_point.x = out_cloud_2->points[i].x;
                                tmp_point.y = out_cloud_2->points[i].y;
                                tmp_point.z = out_cloud_2->points[i].z;
                                out_cloud->points.push_back(tmp_point);
                                vec_points.push_back(out_cloud_2->points[i].x);
                            }
                        } else {
                            if (std::find(vec_points.begin(), vec_points.end(), out_cloud_2->points[i].x) ==
                                vec_points.end()) {
                                pcl::PointXYZI tmp_point;
                                tmp_point.x = out_cloud_2->points[j].x;
                                tmp_point.y = out_cloud_2->points[j].y;
                                tmp_point.z = out_cloud_2->points[i].z;
                                out_cloud->points.push_back(tmp_point);
                                vec_points.push_back(out_cloud_2->points[j].x);
                            }
                        }
                    }
                }
            }
        }
    }
}




void Making::labelbyDis(pcl::PointCloud<pcl::PointXYZI> cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,float distance_threshold) {
    int label = 1;
    std::vector<label_point> cloud_label;
    for(int i=0;i<cloud.points.size();++i){
        label_point tmp_point;
        tmp_point.label=1;
        tmp_point.point=cloud.points[i];
        cloud_label.push_back(tmp_point);
    }
//    std::cout<<"the size of cloud label is "<<cloud_label.size()<<std::endl;
    std::vector<label_point>::iterator it_cloud=cloud_label.begin();
    for (; it_cloud!= cloud_label.end(); ++it_cloud) {
        if ((*it_cloud).label == 1) {
            std::stack<label_point> neighborPixels;
            neighborPixels.push((*it_cloud));// pixel position: <i,j>
            ++label;
            std::cout << "the part " << label << "is done!" << std::endl;
            while (!neighborPixels.empty()) {
                label_point poppoint = neighborPixels.top();
                poppoint.label = label;
                neighborPixels.pop();
                std::vector<label_point>::iterator it_cloud_2=cloud_label.begin();
                for (; it_cloud_2!=cloud_label.end(); ++it_cloud_2) {
                    if ((*it_cloud_2).label != 1) continue;
                    float distance = sqrt(
                            powf(poppoint.point.x - (*it_cloud_2).point.x, 2) + powf(poppoint.point.y - (*it_cloud_2).point.y, 2));
                    if (distance > 0.01 && distance < distance_threshold) {
                        (*it_cloud_2).label = label;
                        neighborPixels.push((*it_cloud_2));
                    }
                }
            }
        }
    }

    std::vector< std::vector<label_point> > vec_points;
    vec_points.resize(label + 1);
    std::vector<label_point>::iterator it_cloud_3=cloud_label.begin();
    for (; it_cloud_3!= cloud_label.end(); ++it_cloud_3) {
        vec_points[(*it_cloud_3).label].push_back((*it_cloud_3));
    }
    std::vector< std::vector<label_point> >::iterator it_vec=vec_points.begin();
    for (; it_vec!=vec_points.end(); ++it_vec) {
        if ((*it_vec).size() < 6) continue;
        std::vector<label_point>::iterator it_cloud_4=(*it_vec).begin();
        float x = 0;
        float y = 0;
        float z = 0;
        for (; it_cloud_4!=(*it_vec).end(); ++it_cloud_4) {
                x += (*it_cloud_4).point.x;
                y += (*it_cloud_4).point.y;
                z += (*it_cloud_4).point.z;
            }
            x /= (*it_vec).size();
            y /= (*it_vec).size();
            z /= (*it_vec).size();
            pcl::PointXYZI tmp_point;
            tmp_point.x=x;
            tmp_point.y=y;
            tmp_point.z=z;
            tmp_point.intensity=(*it_vec).size();
            out_cloud->points.push_back(tmp_point);
    }
}


void Making::readTxt(std::string txt_path,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    std::ifstream filename;
    filename.open(txt_path.c_str());
    std::string line;
    cloud->clear();
    while(!filename.eof()){
        getline(filename,line);
        std::istringstream iss(line);
        float data[4];
        std::string sub_line;
        int count=0;
        while(iss>>sub_line){
            data[count++]=atof(sub_line.c_str());
        }
        pcl::PointXYZI tmp_point;
        tmp_point.x=data[1];
        tmp_point.y=data[2];
        tmp_point.z=data[3];
        cloud->push_back(tmp_point);
    }
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "mapping_node");

    ros::NodeHandle nh;

    ros::NodeHandle pri_nh("~");

    Making map(nh,pri_nh);

    ros::spin();

    return 0;

}