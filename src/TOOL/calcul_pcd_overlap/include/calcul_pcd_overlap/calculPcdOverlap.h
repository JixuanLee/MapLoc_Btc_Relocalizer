/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-21 18:09:22
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-24 12:19:00
 * @FilePath: /OnlineLTSlam/src/TOOL/calcul_pcd_overlap/include/calcul_pcd_overlap/calculPcdOverlap.h
 * @Description: 
 * @Logs: 
 */

#include <ros/ros.h>
#include <iostream>
#include <unordered_map>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "glog/logging.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#define HASH_P 116101
#define MAX_N 10000000000

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

class VOXEL_LOC{
    public:
      int64_t x,y,z;
  
    VOXEL_LOC(int64_t vx=0, int64_t vy =0, int64_t vz = 0  )
      :x(vx), y(vy), z(vz){};
    bool operator==(const VOXEL_LOC &other) const{
      return (x == other.x && y == other.y && z == other.z);
    }
};

// 向标准库std提供自定义数据类型转为哈希值的方法struct hash<VOXEL_LOC>
namespace std {
  template <>
  struct hash<VOXEL_LOC> {
    int64_t operator()(const VOXEL_LOC &s) const {
      using std::hash;
      using std::size_t;
      return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    }
  };
}
  
class VOXEL_INFO{
    public:
      double mean_x, mean_y, mean_z;
      int num;
      double length;
  
    VOXEL_INFO( int count =0 , double length = 0.5)
      :num(count), length(length){};
};

class calculPcdOverlap
{

private:
  std::unordered_map<VOXEL_LOC, std::shared_ptr<VOXEL_INFO> > voxel_map;
  double voxel_length;
  Eigen::Vector3d origin;
  pcl::PCDReader reader;

public:
    // calculPcdOverlap() = default;
    calculPcdOverlap(double length);
    ~calculPcdOverlap();

    bool loadOdomPcd(const std::string file_path, CloudPtr& pcd);
    bool loadLidar2OdomPoses(const std::string& filename, Eigen::Matrix4d& matrix1, Eigen::Matrix4d& matrix2, Eigen::Matrix4d& matrix3);
    bool make_voxel_map(const CloudPtr& scan_odom, const Eigen::Matrix4d& pose, double range);
    double calculate_overlap_rate(const CloudPtr& scan_odom , const Eigen::Matrix4d& pose);
};


