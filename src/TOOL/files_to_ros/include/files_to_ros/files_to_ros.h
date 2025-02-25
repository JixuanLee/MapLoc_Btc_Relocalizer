/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-13 17:35:17
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-01-16 11:26:44
 * @FilePath: /OnlineLTSlam/src/TOOL/files_to_ros/include/files_to_ros/files_to_ros.h
 * @Description: Extract the pose and pointcloud info of keyframes from the files which we have got, 
 *               and output ROS format pointcloud, odometer, low-frequency global pointcloud map, 
 *               and cumulative path for each keyframe.
 * @Logs: 
 */


#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <filesystem>
#include <signal.h>
#include <omp.h>
#include <tf/tf.h>
#include <unordered_set>
#include <unordered_map>

#define PCL_ADD_POINT4D_DOUBLE \
    double x; \
    double y; \
    double z; \
    double w;

struct PointXYZIRPYT
{
    // PCL_ADD_POINT4D     
    PCL_ADD_POINT4D_DOUBLE
    PCL_ADD_INTENSITY;  
    double roll;         
    double pitch;
    double yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (double, x, x) (double, y, y)
                                   (double, z, z) (float, intensity, intensity)
                                   (double, roll, roll) (double, pitch, pitch) (double, yaw, yaw)
                                   (double, time, time))

// struct DoubleHash
// {
//     std::hash<double> hasher;
//     double tolerance = 1e-6;
//     size_t operator()(double value) const{
//         return hasher(value);
//     }
// };
// struct DoubleEqual
// {
//     double tolerance = 1e-6;
//     bool operator()(double a, double b) const{
//         return std::abs(a - b) < tolerance;
//     }
// };

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointXYZI PointType;
namespace fs = boost::filesystem;

pcl::VoxelGrid<PointType> downSizeFilterGlobalPCD;
float leafSize;

std::string base_path;
std::string file_pcds;
std::string path_poses; 
std::string path_save_global_pcd;

std::unordered_map<std::string, std::string> pcd_files_map;

PointTypePose initPose;
extern std::vector<std::pair<double, PointTypePose>> pathVec;
extern pcl::PointCloud<PointType>::Ptr globalPointCloudFramePtr;
extern nav_msgs::Path globalPath;

ros::Publisher pubOriPcd;
ros::Publisher pubGlobalPcd;
ros::Publisher pubGlobalPath;
ros::Publisher pubScanOdom;

std::string topicOriPcd;
std::string topicGlobalPcd;
std::string topicGlobalPath;
std::string topicScanOdom;

bool isRunning;
int numberOfCores;
int skipHz;
bool doPubWithFrequency;
float pubFrequency;
