/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-21 18:09:39
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-25 12:27:47
 * @FilePath: /OnlineLTSlam/src/TOOL/calcul_pcd_overlap/src/calculPcdOverlap.cpp
 * @Description: 
 * @Logs: 
 */

 #include "calcul_pcd_overlap/calculPcdOverlap.h"

calculPcdOverlap::calculPcdOverlap(double length)
{
  voxel_length = length;
}

calculPcdOverlap::~calculPcdOverlap()
{
}

bool calculPcdOverlap::loadOdomPcd(const std::string file_path, CloudPtr& pcd)
{
  if (reader.read(file_path, *pcd) == -1)
  {
    std::cout<<"Couldn't read file " << file_path.c_str();
    return false;
  }
  return true;
}

bool calculPcdOverlap::loadLidar2OdomPoses(const std::string& filename, Eigen::Matrix4d& matrix1, Eigen::Matrix4d& matrix2, Eigen::Matrix4d& matrix3)
{
  std::ifstream inFile(filename);
  if (!inFile.is_open()) {
    std::cerr << "Error: Unable to open file " << filename << std::endl;
    return false;
  }

  std::string line;
  std::getline(inFile, line);  // 跳过第一行（解释性字符串）

  // 读取第1个矩阵
  if (!std::getline(inFile, line)) {
    std::cerr << "Error: Unable to read matrix1 from file." << std::endl;
    return false;
  }

  std::istringstream matrix1Stream(line);
  matrix1.setZero();  // 初始化为零矩阵
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      matrix1Stream >> matrix1(i, j);
    }
  }

  // 读取第2个矩阵
  if (!std::getline(inFile, line)) {
    std::cerr << "Error: Unable to read matrix2 from file." << std::endl;
    return false;
  }

  std::istringstream matrix2Stream(line);
  matrix2.setZero();  // 初始化为零矩阵
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      matrix2Stream >> matrix2(i, j);
    }
  }

  // 读取第3个矩阵
  if (!std::getline(inFile, line)) {
    std::cerr << "Error: Unable to read matrix3 from file." << std::endl;
    return false;
  }

  std::istringstream matrix3Stream(line);
  matrix3.setZero();  // 初始化为零矩阵
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      matrix3Stream >> matrix3(i, j);
    }
  }

  inFile.close();
  std::cout << "Matrices loaded from " << filename << std::endl;
  return true;
}


bool calculPcdOverlap::make_voxel_map(const CloudPtr& scan_odom, const Eigen::Matrix4d& pose, double range){
    if( scan_odom->points.empty()){
      std::cout<<"  make_voxel_map filed : no points in scan cloud "<<std::endl;
      return false;
    }
  
    origin = Eigen::Vector3d(pose(0,3)-200, pose(1,3)-200, pose(2,3)-10 );
    for( auto point : scan_odom->points)
    {
      float loc_xyz[3];
      loc_xyz[0] = point.x - origin.x();
      loc_xyz[1] = point.y - origin.y();
      loc_xyz[2] = point.z - origin.z();
  
      double dis =std::sqrt( std::pow( (point.x - pose(0,3)), 2) + std::pow( (point.y - pose(1,3)), 2) );
      if( dis > range){
        continue;
      }
  
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = loc_xyz[j] / voxel_length;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                         (int64_t)loc_xyz[2]);
  
      auto iter = voxel_map.find(position);
      if (iter != voxel_map.end()) {
  
        voxel_map[position]->num++ ;
      } else {
  
        voxel_map[position] = std::make_shared<VOXEL_INFO>(0, voxel_length) ;
        voxel_map[position]->num++ ;
      }
  
    }
    return true;
    
}

double calculPcdOverlap::calculate_overlap_rate(const CloudPtr& scan_odom , const Eigen::Matrix4d& pose){
  
    CloudPtr transed_cloud(new PointCloudType);
    pcl::transformPointCloud(*scan_odom, *transed_cloud, pose.cast<float>());
  
    double overlapnum =0;
    for( auto point: transed_cloud->points){
      float loc_xyz[3];
      loc_xyz[0] = point.x - origin.x();
      loc_xyz[1] = point.y - origin.y();
      loc_xyz[2] = point.z - origin.z();
  
  
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = loc_xyz[j] / voxel_length;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
  
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                         (int64_t)loc_xyz[2]);
  
      auto iter = voxel_map.find(position);
      if (iter != voxel_map.end() && voxel_map[position]->num++ > 5) {
        overlapnum ++ ;
      }
    }
    return overlapnum/scan_odom->points.size();
}
  
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
  cpo.make_voxel_map(pcd1, pcd1ori, rangeRoi);
  cpoRateOri = cpo.calculate_overlap_rate(pcd2, Eigen::Matrix4d::Identity());
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
