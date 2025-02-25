/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-16 11:26:29
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-01-16 15:22:43
 * @FilePath: /OnlineLTSlam/src/TOOL/files_to_ros/src/load_pcd_to_view.cpp
 * @Description: 
 * @Logs: 
 */

#include "files_to_ros/load_pcd_to_view.h"

pcl::PointCloud<PointType>::Ptr pcdToViewPtr(new pcl::PointCloud<PointType>);

void initParam(ros::NodeHandle& nh)
{
    nh.param("/load_pcd_to_view/base_path", base_path, std::string("/home/jixuanlee/datasetBXN/"));
    nh.param("/load_pcd_to_view/pcd_name", pcd_name, std::string("merge-1st-6000.pcd"));
    nh.param("/load_pcd_to_view/topicScanOdom", topicPcdOnce, std::string("/BXN/pcd_to_view"));

    pcd_name = base_path + pcd_name;

    pubPcdOnce = nh.advertise<sensor_msgs::PointCloud2>(topicPcdOnce, 1, true);

}

void pubPointCloud(const pcl::PointCloud<PointType>::Ptr& data, std::string frame, ros::Publisher& pubObj)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*data,msg);
    msg.header.frame_id = frame;
    pubObj.publish(msg);
    std::cout << "[lptv] Pub pointcloud topic with points: " << msg.data.size() << std::endl;
}

void loadPcdFile()
{
    if (pcl::io::loadPCDFile<PointType>(pcd_name, *pcdToViewPtr) == 0) 
    {
        std::cout << "[lptv] Load PCD file and pub: " << pcdToViewPtr->size() <<" points" << std::endl;
        pubPointCloud(pcdToViewPtr, "map", pubPcdOnce);
    }
    else
    {
        std::cerr << "[lptv] Failed to load PCD file: " << pcd_name << std::endl;
    }
}

void run(ros::NodeHandle& nh)
{
    initParam(nh);
    loadPcdFile();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "load_pcd_to_view");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> load_pcd_to_view Started.\033[0m");
   
    run(nh);

    return 0;
}