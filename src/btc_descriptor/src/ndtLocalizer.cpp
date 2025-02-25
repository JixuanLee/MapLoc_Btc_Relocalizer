/*
 * @Author: Jixuan Lee
 * @Date: 2024-12-15 14:09:11
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-20 17:16:11
 * @FilePath: /OnlineLTSlam/src/btc_descriptor/src/ndtLocalizer.cpp
 * @Description: A C++ class for NDT localization
 * @Logs: 
 *      1.本插件API来自OnlineLTSlam项目，ljx于2024-12开发完成，现移植至BTC模块使用，以进行BTC/NDT算法重定位精度与耗时验证。
 */

#include "include/ndtLocalizer.h"

namespace OnlineLTSlam
{

ndtLocalizer::ndtLocalizer()
:tf2_listener_(tf2_buffer_)
{}

ndtLocalizer::ndtLocalizer(ros::NodeHandle& nh)
: nh_(nh), tf2_listener_(tf2_buffer_),
mapPointCloudPtr(new pcl::PointCloud<PointType>()), scanPointCloudPtr(new pcl::PointCloud<PointType>())
{}

ndtLocalizer::~ndtLocalizer(){}

void ndtLocalizer::getNdtParam(std::string map_frame, std::string scan_frame, double trans_epsilon, double step_size, double resolution, int max_iterations, double converged_param_transform_probability)
{
        map_frame_ = map_frame;
        scan_frame_ = scan_frame;
        trans_epsilon_ = trans_epsilon;
        step_size_ = step_size;
        resolution_ = resolution;
        max_iterations_ = max_iterations;
        converged_param_transform_probability_ = converged_param_transform_probability;

        ndt_.setTransformationEpsilon(trans_epsilon_);
        ndt_.setStepSize(step_size_);
        ndt_.setResolution(resolution_);
        ndt_.setMaximumIterations(max_iterations_);
        
        ROS_INFO("[NDT] trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d, converged_param_transform_probability: %lf", 
                trans_epsilon_, step_size_, resolution_, max_iterations_, converged_param_transform_probability_);

        topicNdtPose_ = "/onlineLTSlam/ndt_pose";
        pubNdtPose_ = nh_.advertise<geometry_msgs::PoseStamped>(topicNdtPose_, 10);
}

void ndtLocalizer::scanPointsFilter()
{
        // usualy, we dont need to downsample the target points (map points). Actually, if you downsample it, the align may cost more time. 
        // But downsample the scan points is useful.
        int numScanBef = scanPointCloudPtr->size();
        // XYZ直通滤波
        pass.setInputCloud(scanPointCloudPtr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-25, 25);
        pass.filter(*scanPointCloudPtr);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(-25, 25);
        pass.filter(*scanPointCloudPtr);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(-10, 15);
        pass.filter(*scanPointCloudPtr);

        // // 去除离散点
        sor.setInputCloud(scanPointCloudPtr);
        sor.setRadiusSearch(0.15);
        sor.setMinNeighborsInRadius(5);
        sor.filter(*scanPointCloudPtr);

        // 降采样
        vg.setInputCloud(scanPointCloudPtr);
        // vg.setLeafSize(0.3, 0.3, 0.3); // for Once-16lines
        vg.setLeafSize(0.08, 0.08, 0.08); // for Always-16lines
        vg.filter(*scanPointCloudPtr);

        int numScanAft = scanPointCloudPtr->size();
        ROS_INFO("[NDT] ScanPoints has been filtered %.2f%, will use the last %d scan points.", float(numScanBef - numScanAft)/float(numScanBef)*100.0, numScanAft);
}

bool ndtLocalizer::ndtAlignOnce(geometry_msgs::PoseStamped& result_pose,
        const pcl::PointCloud<PointType>::Ptr & scan_pc_ptr, const ros::Time& scan_time,
        const pcl::PointCloud<PointType>::Ptr & map_pc_ptr, Eigen::Matrix4f init_mat,
        bool doScanPointsFilter)
{
        if (map_pc_ptr == nullptr || map_pc_ptr->empty())
        {
                ROS_ERROR("[NDT] No map info!");
                return false;
        }
        else if (scan_pc_ptr->empty())
        {
                ROS_ERROR("[NDT] No scan info!");
                return false;
        }

        mapPointCloudPtr->clear();
        scanPointCloudPtr->clear();
        pcl::copyPointCloud(*map_pc_ptr, *mapPointCloudPtr);
        pcl::copyPointCloud(*scan_pc_ptr, *scanPointCloudPtr);

        if (doScanPointsFilter)
                scanPointsFilter();

        initMat = init_mat;
        ROS_INFO("[NDT] MapPoints: %d, ScanPoints: %d", int(mapPointCloudPtr->size()), int(scanPointCloudPtr->size()));

        ndt_.setInputTarget(mapPointCloudPtr);
        ndt_.setInputSource(scanPointCloudPtr);

        if (ndt_.getInputTarget() == nullptr) 
        {
                ROS_WARN_STREAM_THROTTLE(1, "[NDT] NDT's Target Points is NULL!");
                return false;
        }
        else if (ndt_.getInputSource()  == nullptr)
        {
                ROS_WARN_STREAM_THROTTLE(1, "[NDT] NDT's Source Points is NULL!");
                return false;
        }

        pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
        const auto align_start_time = std::chrono::system_clock::now();
        ndt_.align(*output_cloud, initMat); // 参数1保存了source点云变换后的结果，也就是与target点云配准后的点云；参数2是初始猜想矩阵。
        const auto align_end_time = std::chrono::system_clock::now();
        const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;
        
        const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
        Eigen::Affine3d result_pose_affine;
        result_pose_affine.matrix() = result_pose_matrix.cast<double>();
        const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

        result_pose.pose = result_pose_msg;
        result_pose.header.stamp = scan_time;
        result_pose.header.frame_id = map_frame_;
        
        // pub the ndt output Topic
        pubNdtPose_.publish(result_pose);

        // broadcast the ndt output TF 
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = scan_time;
        tfs.header.frame_id = map_frame_;
        tfs.child_frame_id = scan_frame_;
        tfs.transform.translation.x = result_pose.pose.position.x;
        tfs.transform.translation.y = result_pose.pose.position.y;
        tfs.transform.translation.z = result_pose.pose.position.z;
        tfs.transform.rotation = result_pose.pose.orientation;
        tf2_broadcaster_.sendTransform(tfs); 

        const float transform_probability = ndt_.getTransformationProbability();
        const int iteration_num = ndt_.getFinalNumIteration();
        static size_t skipping_publish_num = 0;
        
        ROS_INFO("------------------------------------------------");
        std::cout << "[NDT] Align cost time: " << align_time << "ms" << std::endl;
        std::cout << "[NDT] Align result score: " << transform_probability 
                                <<" ( Min score ): "<<converged_param_transform_probability_<< std::endl;
        std::cout << "[NDT] Align iterate times: " << iteration_num << std::endl;
        std::cout << "[NDT] Has failed times: " << skipping_publish_num << std::endl;


        // 实际迭代过多、转换概率过低，即判定“不收敛”
        // 实测，定位效果较好时，迭代次数一般在10以下，基本上都收敛，transform_probability基本上在4以上
        if (iteration_num >= ndt_.getMaximumIterations() + 2 ||
                transform_probability < converged_param_transform_probability_) 
        {
                ++skipping_publish_num;
                ROS_ERROR("[NDT] The align is Bad!");
                return false;
        } 
        else 
        {
                skipping_publish_num = 0;
                ROS_INFO("[NDT] The align is Good!");
                return true;
        }
}


bool ndtLocalizer::ndtAlignAlways(geometry_msgs::PoseStamped& result_pose, 
        const pcl::PointCloud<PointType>::Ptr & scan_pc_ptr, const ros::Time& scan_time, 
        const pcl::PointCloud<PointType>::Ptr & map_pc_ptr, Eigen::Matrix4f init_mat, 
        bool has_inited)
{
        // 第一次调用时，所有参数都要有，而且最后参数为false。之后调用只需要提供3个参数即可。
        if (has_inited == false)
        {
                if (map_pc_ptr == nullptr || map_pc_ptr->empty())
                {
                        ROS_ERROR("[NDT] No map info!");
                        return false;
                }
                else if (scan_pc_ptr->empty())
                {
                        ROS_ERROR("[NDT] No scan info!");
                        return false;
                }
                
                mapPointCloudPtr->clear();
                scanPointCloudPtr->clear();
                pcl::copyPointCloud(*map_pc_ptr, *mapPointCloudPtr);
                pcl::copyPointCloud(*scan_pc_ptr, *scanPointCloudPtr);
                initMat = init_mat;
                preTrans = initMat;

                scanPointsFilter();

                ROS_INFO("[NDT] MapPoints: %d, ScanPoints: %d", int(mapPointCloudPtr->size()), int(scanPointCloudPtr->size()));
                ndt_.setInputTarget(mapPointCloudPtr);
                ndt_.setInputSource(scanPointCloudPtr);
        }
        else
        {
                if (scan_pc_ptr->empty())
                {
                        ROS_ERROR("[NDT] No scan info!");
                        return false;
                }
                scanPointCloudPtr->clear();
                pcl::copyPointCloud(*scan_pc_ptr, *scanPointCloudPtr);
                initMat = preTrans * deltaTrans;

                scanPointsFilter();
                
                ROS_INFO("[NDT] MapPoints: %d, ScanPoints: %d", int(mapPointCloudPtr->size()), int(scanPointCloudPtr->size()));
                ndt_.setInputSource(scanPointCloudPtr);
        }
        if (ndt_.getInputTarget() == nullptr) 
        {
                ROS_WARN_STREAM_THROTTLE(1, "[NDT] NDT's Target Points is NULL!");
                return false;
        }
        else if (ndt_.getInputSource()  == nullptr)
        {
                ROS_WARN_STREAM_THROTTLE(1, "[NDT] NDT's Source Points is NULL!");
                return false;
        }

        pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
        const auto align_start_time = std::chrono::system_clock::now();
        ndt_.align(*output_cloud, initMat); // 参数1保存了source点云变换后的结果，也就是与target点云配准后的点云；参数2是初始猜想矩阵。
        const auto align_end_time = std::chrono::system_clock::now();
        const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;
        const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
        Eigen::Affine3d result_pose_affine;
        result_pose_affine.matrix() = result_pose_matrix.cast<double>();
        const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

        result_pose.pose = result_pose_msg;
        result_pose.header.stamp = scan_time;
        result_pose.header.frame_id = map_frame_;

        deltaTrans = preTrans.inverse() * result_pose_matrix; // updata
        preTrans = result_pose_matrix;

        // pub the ndt output Topic
        pubNdtPose_.publish(result_pose);
        // broadcast the ndt output TF 
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = scan_time;
        tfs.header.frame_id = map_frame_;
        tfs.child_frame_id = scan_frame_;
        tfs.transform.translation.x = result_pose.pose.position.x;
        tfs.transform.translation.y = result_pose.pose.position.y;
        tfs.transform.translation.z = result_pose.pose.position.z;
        tfs.transform.rotation = result_pose.pose.orientation;
        tf2_broadcaster_.sendTransform(tfs); 

        const float transform_probability = ndt_.getTransformationProbability();
        const int iteration_num = ndt_.getFinalNumIteration();
        static size_t skipping_publish_num = 0;
        
        ROS_INFO("------------------------------------------------");
        std::cout << "[NDT] Align cost time: " << align_time << "ms" << std::endl;
        std::cout << "[NDT] Align result score: " << transform_probability 
                                <<" ( Min score ): "<<converged_param_transform_probability_<< std::endl;
        std::cout << "[NDT] Align iterate times: " << iteration_num << std::endl;
        std::cout << "[NDT] Has failed times: " << skipping_publish_num << std::endl;

        // 实际迭代过多、转换概率过低，即判定“不收敛”
        // 实测，定位效果较好时，迭代次数一般在10以下，基本上都收敛，transform_probability基本上在4以上
        if (iteration_num >= ndt_.getMaximumIterations() + 2 ||
                transform_probability < converged_param_transform_probability_) 
        {
                ++skipping_publish_num;
                ROS_ERROR("[NDT] The align is Bad!");
                return false;
        } 
        else 
        {
                skipping_publish_num = 0;
                ROS_INFO("[NDT] The align is Good!");
                return true;
        }
}

}