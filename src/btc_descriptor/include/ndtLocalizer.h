/*
 * @Author: Jixuan Lee
 * @Date: 2024-12-15 14:08:20
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-20 14:40:02
 * @FilePath: /OnlineLTSlam/src/btc_descriptor/include/ndtLocalizer.h
 * @Description: A C++ class for NDT localization
 * @Logs: 
 *      1.本插件API来自OnlineLTSlam项目，ljx于2024-12开发完成，现移植至BTC模块使用，以进行BTC/NDT算法重定位精度与耗时验证。
 *     
 */

#pragma once

#include <ros/ros.h>
#include <chrono>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace OnlineLTSlam
{
        
typedef pcl::PointXYZI PointType;

class ndtLocalizer
{
private:
        std::string map_frame_;
        std::string scan_frame_;

        pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
        double trans_epsilon_; // 收敛阈值 
        double step_size_; // 步长
        double resolution_; // 分辩率: 太大会导致精度不高，太小导致内存过高，并且只有两幅点云相差不大的情况才能匹配
        int max_iterations_; // 最大迭代次数
        double converged_param_transform_probability_;

        pcl::PassThrough<PointType> pass;
        pcl::RadiusOutlierRemoval<PointType> sor;
        pcl::VoxelGrid<PointType> vg;

        pcl::PointCloud<PointType>::Ptr mapPointCloudPtr; // 地图点云，map系
        pcl::PointCloud<PointType>::Ptr scanPointCloudPtr; // 当前扫描点云，lidar系
        Eigen::Matrix4f initMat;
        Eigen::Matrix4f preTrans, deltaTrans;

        ros::NodeHandle nh_;
        ros::Publisher pubNdtPose_;
        std::string topicNdtPose_;

        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::TransformBroadcaster tf2_broadcaster_;


public:
        ndtLocalizer();
        ndtLocalizer(ros::NodeHandle& nh);
        ~ndtLocalizer();

        void getNdtParam( std::string map_frame, std::string scan_frame, double trans_epsilon, double step_size, double resolution, int max_iterations, double converged_param_transform_probability);

        void scanPointsFilter();

        // 单次匹配以实现NDT定位
        bool ndtAlignOnce(geometry_msgs::PoseStamped& result_pose,
                const pcl::PointCloud<PointType>::Ptr & scan_pc_ptr, const ros::Time& scan_time,
                const pcl::PointCloud<PointType>::Ptr & map_pc_ptr, Eigen::Matrix4f init_mat = Eigen::Matrix4f::Identity(),
                bool doScanPointsFilter = true);

        // 循环匹配以实现NDT连续自定位
        bool ndtAlignAlways(geometry_msgs::PoseStamped& result_pose, 
                const pcl::PointCloud<PointType>::Ptr & scan_pc_ptr, const ros::Time& scan_time, 
                const pcl::PointCloud<PointType>::Ptr & map_pc_ptr = nullptr, Eigen::Matrix4f init_mat = Eigen::Matrix4f::Identity(), 
                bool has_inited = true);

};



} //namespace OnlineLTSlam