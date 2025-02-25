/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-16 11:28:41
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-01-16 11:43:12
 * @FilePath: /OnlineLTSlam/src/TOOL/files_to_ros/include/files_to_ros/load_pcd_to_view.h
 * @Description: 
 * @Logs: 
 */
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZI PointType;

extern pcl::PointCloud<PointType>::Ptr pcdToViewPtr;

std::string base_path;
std::string pcd_name;

ros::Publisher pubPcdOnce;
std::string topicPcdOnce;