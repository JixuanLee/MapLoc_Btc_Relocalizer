/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-17 17:15:17
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-26 12:30:44
 * @FilePath: /OnlineLTSlam/src/btc_descriptor/example/place_recognition_and_relocation.h
 * @Description: 
 * @Logs: 
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/filesystem.hpp>
#include <signal.h>
#include <chrono>
#include <execution>
#include <omp.h>

#include "include/btc.h"
#include "include/utils.h"
#include "include/ndtLocalizer.h"
#include <calcul_pcd_overlap/calculPcdOverlap.h>

/**
 * @brief ljx 自定义时间打印类 
 * TODO: 自定义timer尺寸
 */
class Timer {
  public:
    Timer(){
      std::fill(std::begin(started_), std::end(started_), false);
    };
    void start(const int id) noexcept {
      checkid(id);
      start_[id] = Clock::now();
      started_[id] = true;
    }
    double elapsed(const int id) const {
      checkid(id);
      if (!started_[id]) 
          throw std::logic_error("Timer not started!");
      return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_[id]).count() /1000.0; // 获取经过时间（毫秒）
    }
    void print(const int id, const std::string title = ""){
      checkid(id);
      std::cout<<title.c_str()<<std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_[id]).count() /1000.0<<"ms."<<std::endl;
    }
  private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    TimePoint start_[20];
    bool started_[20];
    void checkid(const int id) const {
      if (id<0 || id>20) { 
        throw std::logic_error("TimerID should be in 0-19!(now we just support 20 timers, the update is coming soon)"); 
      }
    }
};

typedef pcl::PointXYZI PointType;

typedef struct PosesDiff {
    Eigen::Vector3d t_diff_xyz;
    double t_diff_value;

    Eigen::Matrix3d rot_diff_mat;
    Eigen::Vector3d rot_diff_rpy_deg;
    double rot_diff_value_deg;

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose_diff;
  } PosesDiff;

Timer T;
  
bool isRunning;

std::string setting_path;
std::string pcds_dir;
std::string pose_file;
std::string result_file;
double cloud_overlap_thr;
bool read_bin;

ros::Publisher pubOdomAftMapped;
ros::Publisher pubCureentCloud;
ros::Publisher pubCurrentBinary;
// ros::Publisher pubPath;
// ros::Publisher pubCurrentPose;
// ros::Publisher pubMatchedPose;
ros::Publisher pubMatchedCloud; // 发布粗匹配BTC匹配到，且点云很重和的，old帧的点云（绿色），好的回环点  or  发布粗匹配BTC匹配到，且点云不重和的，过去帧的点云（红色），坏的回环点
ros::Publisher pubMatchedBinary; // 本帧回环匹配到的old帧的所有三角形描述符顶点位置
ros::Publisher pubLoopStatus; // 发布位姿的连线
ros::Publisher pubBTC; // 发布回环粗匹配触发后的新老帧的三角形
ros::Publisher pubCurrentPlane; // 发布当前帧的平面

std_msgs::ColorRGBA color_tp;
std_msgs::ColorRGBA color_fp;
std_msgs::ColorRGBA color_path;

double scale_tp;
double scale_fp;
double scale_path;

const static std::string RED_COLOR = "\033[31m";
const static std::string GREEN_COLOR = "\033[32m";
const static std::string RESET_COLOR = "\033[0m";

std::vector<pcl::PointCloud<PointType>::Ptr> ori_clouds; // 所有的原始点云（lidar系）

std::chrono::_V2::system_clock::time_point beginTimeOfTheFrameWhoseBtcLoopIsNiceAndDoOpti;
std::chrono::_V2::system_clock::time_point endTimeOfTheFrameWhoseBtcLoopIsNiceAndDoOpti;

ConfigSetting config_setting;

std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
std::vector<double> time_list;

extern std::unique_ptr<BtcDescManager> btc_manager;

pcl::PCDReader reader;

pcl::RadiusOutlierRemoval<PointType> inilerFilter;
pcl::VoxelGrid<PointType> downSampleFilter;

