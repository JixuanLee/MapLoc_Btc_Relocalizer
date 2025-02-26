/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-25 16:05:08
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-25 16:30:51
 * @FilePath: /OnlineLTSlam/src/TOOL/calcul_pcd_overlap/src/compareBtcWithNdt.cpp
 * @Description: 一个测试BTC与NDT重定位对点云拼接重合精度的小example
 * @Logs: 
 */

 #include "calcul_pcd_overlap/calculPcdOverlap.h"

void run(calculPcdOverlap& cpo, const std::string file_path_1, const std::string file_path_2, const std::string file_path_3,
    const std::string file_poses)
  {
    CloudPtr pcd1(new PointCloudType());
    CloudPtr pcd2(new PointCloudType());
    CloudPtr pcd3(new PointCloudType());
    Eigen::Matrix4d pcd1ori, pcd2ori, pcd2opti;
  
    if (!cpo.loadOdomPcd(file_path_1, pcd1) ||
      !cpo.loadOdomPcd(file_path_2, pcd2) ||
      !cpo.loadOdomPcd(file_path_3, pcd3) ||
      !cpo.loadLidar2OdomPoses(file_poses, pcd1ori, pcd2ori, pcd2opti))
    {
      std::cout<<"[CPO] Load failed."<<std::endl;
      return;
    }
    std::cout<<"[CPO] Load succeed."<<std::endl;
    
    static double rangeRoi = 200;
    double cpoRateOri = -1;
    double cpoRateOpti = -1;
    // 输入的点云地图：来自loop的submap(odom系)，输入的位姿变化：loop帧的lidar2odom-ori，输入的距离范围：自定
    cpo.make_voxel_map(pcd1, pcd1ori, rangeRoi);
    // 输入的点云：来自curr的单帧(odom系，其是由lidar2odom-ori变换到的)，输入的位姿：单位阵(暂时没用)
    cpoRateOri = cpo.calculate_overlap_rate(pcd2, Eigen::Matrix4d::Identity());
    // 输入的点云：来自curr的单帧(odom系，其是由lidar2odom-opti变换到的)，输入的位姿：单位阵(暂时没用)
    cpoRateOpti = cpo.calculate_overlap_rate(pcd3, Eigen::Matrix4d::Identity());
  
    std::cout<<"[CPO] The rate of pcds(ori-ori):"<<cpoRateOri<<" ,and the pcds(ori-opti):"<<cpoRateOpti<<std::endl;
  
  }
  
  int main(int argc, char *argv[])
  {
    ros::init(argc, argv, "calcul_pcd_overlap");
  
    ros::NodeHandle nh;
  
    std::string base_path, file_pcd_1, file_pcd_2, file_pcd_3, file_poses;
    const int mode = 0; // 0:btc, 1:ndt
  
    if (mode == 0){
      // btc
      base_path = "/home/jixuanlee/pcdsOF523/";
      file_pcd_1 = base_path + "loop_points_odom.pcd"; // loop-odomFrame-by-ori-pose
      file_pcd_2 = base_path + "curr_points_odom.pcd"; // curr-odomFrame-by-ori-pose
      file_pcd_3 = base_path + "curr_points_opti.pcd"; // curr-odomFrame-by-opti-pose
      file_poses = base_path + "btc-poses-for-compare.txt";
    }
    else if(mode == 1){
      // ndt
      base_path = "/home/jixuanlee/pcdsOF523-ndt/";
      file_pcd_1 = base_path + "ndt-loop_points_odom.pcd"; // loop-odomFrame-by-ori-pose
      file_pcd_2 = base_path + "ndt-curr_points_odom.pcd"; // curr-odomFrame-by-ori-pose
      file_pcd_3 = base_path + "ndt-curr_points_opti.pcd"; // curr-odomFrame-by-opti-pose
      file_poses = base_path + "ndt-poses-for-compare.txt";
    }
  
    double voxel_length = 0.5;
    calculPcdOverlap cpo(voxel_length);
    
    run(cpo, file_pcd_1, file_pcd_2, file_pcd_3, file_poses);
  
    return 0;
  }