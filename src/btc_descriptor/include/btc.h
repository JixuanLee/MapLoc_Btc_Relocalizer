#ifndef BTC_H
#define BTC_H
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/publisher.h>
#include <ros/ros.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000

typedef struct ConfigSetting {
  /* for submap process*/
  double cloud_ds_size_ = 0.25;

  /* for binary descriptor*/
  int useful_corner_num_ = 30;
  float plane_merge_normal_thre_;
  float plane_merge_dis_thre_;
  float plane_detection_thre_ = 0.01;
  float voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  int proj_plane_num_ = 1;
  float proj_image_resolution_ = 0.5;
  float proj_image_high_inc_ = 0.5;
  float proj_dis_min_ = 0;
  float proj_dis_max_ = 5;
  float summary_min_thre_ = 10;
  int line_filter_enable_ = 0;

  /* for triangle descriptor */
  float descriptor_near_num_ = 10;
  float descriptor_min_len_ = 1;
  float descriptor_max_len_ = 10;
  float non_max_suppression_radius_ = 3.0;
  float std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 20;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  float rough_dis_threshold_ = 0.03;
  float similarity_threshold_ = 0.7;
  float icp_threshold_ = 0.5;
  float normal_threshold_ = 0.1;
  float dis_threshold_ = 0.3;

  /* extrinsic for lidar to vehicle*/
  Eigen::Matrix3d rot_lidar_to_vehicle_;
  Eigen::Vector3d t_lidar_to_vehicle_;

  /* for gt file style*/
  int gt_file_style_ = 0;

} ConfigSetting;

typedef struct BinaryDescriptor {
  std::vector<bool> occupy_array_;// 该网格高程被占据区间的分布，其size固定，为(预设max-预设min)/预设分辨率
  unsigned char summary_; // 该网格高程被占据区间的总数
  Eigen::Vector3d location_; // 该描述符的 实际位置
} BinaryDescriptor;

// Binary Triangle Descriptor
typedef struct BTC {
  Eigen::Vector3d triangle_; // 存放缩放后（源码为放大）的三角形边长（依次为短中长边）
  Eigen::Vector3d angle_;
  Eigen::Vector3d center_; // A B C三个顶点的平均值
  unsigned short frame_number_; // 第几帧点云生成的BTC，有时候就是submap_id
  BinaryDescriptor binary_A_; // A为短中边的顶点，B为短长边的顶点，C为中长边的顶点（的二进制描述符）
  BinaryDescriptor binary_B_;
  BinaryDescriptor binary_C_;
} BTC;

/**
 * @brief 自定义平面类
 */
typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float d_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

/**
 * @brief 本帧 匹配的 胜选帧及其匹配细节
 */
typedef struct BTCMatchList {
  std::vector<std::pair<BTC, BTC>> match_list_;// 本帧与胜选帧之间的所有BTC匹配对
  std::pair<int, int> match_id_; // 1：本帧id； 2：胜选帧id
  int match_frame_; // 胜选帧id
  double mean_dis_;
} BTCMatchList;

/**
 * @brief 【优化】BTC粗匹配&点云重合 都过关的 场景识别帧列表
 */
typedef struct PlaceRecognitionList {
  std::vector<std::pair<BTC, BTC>> match_list_;// 本帧与最优loop帧之间的所有成功的BTC匹配对
  std::pair<int, int> match_id_; // 1：本帧id； 2：最优loop帧id
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform_; // 粗匹配时估计的粗略帧间变换
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> opti_transform_; // 精匹配后的精确帧间变换
} PlaceRecognitionList;

/**
 * @brief 体素的累积点属性类
 */
struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

/**
 * @brief 体素位置类
 */
class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <>
struct hash<VOXEL_LOC> {
  int64 operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
}  // namespace std

class BTC_LOC {
 public:
  int64_t x, y, z, a, b, c;

  BTC_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
          int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const BTC_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

namespace std {
template <>
struct hash<BTC_LOC> {
  int64 operator()(const BTC_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
}  // namespace std

/**
 * @brief 自定义八叉树类
 */
class OctoTree {
 public:
  ConfigSetting config_setting_; // 把this的参数传入该类使用
  std::vector<Eigen::Vector3d> voxel_points_; // 存储该体素下包含的点们的实际坐标
  std::shared_ptr<Plane> plane_ptr_;
  int layer_;
  int octo_state_;  // 0 is end of tree, 1 is not
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> project_normal;
  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3];  // x, y, z
  float quater_length_;
  bool init_octo_;

  // for plot
  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];

  OctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    // for plot
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_.reset(new Plane);
  }

  /**
   * @brief octo初始化程序（主要是plane初始化）
   */
  void init_plane();
  /**
   * @brief octo初始化入口
   */
  void init_octo_tree();
};

/**
 * @brief 对输入点云进行体素化处理，体素特征为平均特征。相当于把一堆点云进行空间划分，每个空间集成出1个点
 * @param pl_feat 输入点云，也是输出的体素特征
 * @param voxel_size 体素尺寸
 */
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

/**
 * @brief 基于cv库实现yaml参数加载
 * @param config_file 
 * @param config_setting 
 */
void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting);

/**
 * @brief 二进制描述符的相似性评估
 * @param b1 
 * @param b2 
 * @return double 
 */
double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2);
/**
 * @brief 根据summary数量对二进制描述符，从大到小排序
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 */
bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b);
/**
 * @brief 根据points_size数量对平面，从大到小排序
 * @param plane1 
 * @param plane2 
 * @return true 
 * @return false 
 */
bool plane_greater_sort(std::shared_ptr<Plane> plane1,
                        std::shared_ptr<Plane> plane2);

void publish_std(const std::vector<std::pair<BTC, BTC>> &match_std_list,
                 const Eigen::Matrix4d &transform1,
                 const Eigen::Matrix4d &transform2,
                 const ros::Publisher &std_publisher);

void publish_std_list(const std::vector<BTC> &btc_list,
                      const ros::Publisher &std_publisher);

void publish_binary(const std::vector<BinaryDescriptor> &binary_list,
                    const Eigen::Vector3d &text_color,
                    const std::string &text_ns,
                    const ros::Publisher &text_publisher);

double calc_triangle_dis(
    const std::vector<std::pair<BTC, BTC>> &match_std_list);

double calc_binary_similaity(
    const std::vector<std::pair<BTC, BTC>> &match_std_list);

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q);

void pubPlane(const ros::Publisher &plane_pub, const std::string plane_ns, const int plane_id, const double during_time, 
              const std::vector<std::pair<pcl::PointXYZINormal,std::pair<Eigen::Vector3d,std::pair<float, float>>>> normal_ps );

struct PlaneSolver 
{
  // 构造函数：初始化当前点和目标点的位置及法向量
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_),        // 当前点的位置
        curr_normal(curr_normal_),      // 当前点的法向量
        target_point(target_point_),    // 目标点的位置
        target_normal(target_normal_){}; // 目标点的法向量

  // 重载 () 运算符：定义残差计算函数
  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const 
  {
    // 将四元数参数转换为 Eigen::Quaternion
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    // 将平移参数转换为 Eigen::Vector3
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    // 将当前点的位置转换为模板类型 T
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    // 计算curr点根据q和t变换后的位置
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    // 将tar点的位置和法向量转换为模板类型 T
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));

    // 计算变换后的curr点到tar平面的距离作为残差
    residual[0] = norm.dot(point_w - point_target);

    // 【优化】
    // 计算法向量差异作为第二个残差项
    // 将当前点的法向量转换为模板类型 T
    Eigen::Matrix<T, 3, 1> curr_norm(T(curr_normal.x()), T(curr_normal.y()),
                                     T(curr_normal.z()));
    // 变换当前点的法向量到目标坐标系
    Eigen::Matrix<T, 3, 1> curr_norm_transformed = q_w_curr.toRotationMatrix() * curr_norm;

    // 归一化 curr_norm_transformed 和 norm
    T current_length = curr_norm_transformed.norm();
    if (current_length > T(1e-6)) { // 防止除以很小的数
        curr_norm_transformed /= current_length;
    }

    T target_length = norm.norm();
    if (target_length > T(1e-6)) {
        norm /= target_length;
    }

    // 计算法向量之间的夹角（替换为法向量差值）
    // residual[1] = (curr_norm_transformed - norm).norm();
    // 或者使用点积来表示夹角
    T dot_product = curr_norm_transformed.dot(norm);
    residual[1] = T(1) - dot_product; // 1 - cos(theta) 用于正则化

    return true;
  }

  // 静态函数：创建 Ceres 代价函数
  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                      const Eigen::Vector3d curr_normal_,
                                      Eigen::Vector3d target_point_,
                                      Eigen::Vector3d target_normal_) {
    // 使用 Ceres 的自动微分创建代价函数，返回一个指针
    // return (
    //     new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver( // 1：残差维度；4：第一个待优化参数的维度；3：第二个待优化参数的维度
    //         curr_point_, curr_normal_, target_point_, target_normal_)));
    // LJX-[优化]：可以考虑提高残差维度，构建法向量到法向量作为二维残差项（当前的点到面，可以把2个平面在距离上拉重合，但是似乎没有考虑大规模夹角的情况？）
    return (
      new ceres::AutoDiffCostFunction<PlaneSolver, 2, 4, 3>(new PlaneSolver( // 2：残差维度；4：第一个待优化参数的维度；3：第二个待优化参数的维度
          curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  // 成员变量：存储当前点和目标点的位置及法向量
  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};

class BtcDescManager {
 public:
  BtcDescManager() = default;

  ConfigSetting config_setting_;

  BtcDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) 
      {
        nice_place_recognition_vec.reserve(100000); // 预分配内存【优化】
      };

  // if print debug info
  bool print_debug_info_ = 0;

  // hash table, save all descriptors
  std::unordered_map<BTC_LOC, std::vector<BTC>> data_base_;

  // save all binary descriptors of key frame
  std::vector<std::vector<BinaryDescriptor>> history_binary_list_;

  // odom下的，拼接、预处理的点云 不建议使用，很消耗内存，容易内存爆炸
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_ori_cloud_vec_;

  // odom下的 为了检测点云重合
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;
  
  // save all planes of all key frames, required 保存每一帧全部的：原始的未合并的、每个体素的小平面（若有）
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;
  
  // 【改进】存储每一帧的原始的all平面
  std::vector<std::vector<std::shared_ptr<Plane>>> plane_ori_vec;

  // 【改进】存储每一帧的二次合并后的all平面
  std::vector<std::vector<std::shared_ptr<Plane>>> plane_merged_vec;

  // 【改进】存储所有的 粗匹配&点云重合 都OK的 场景识别成功信息 的地址
  std::vector<std::shared_ptr<PlaceRecognitionList>> nice_place_recognition_vec;

  /**
   * @brief 【优化】将某一帧的Plane格式平面全部转为pcl点格式
   * @param planes_ptr 输入：某一帧的所有平面的Plane格式的集合
   * @param points_ptr 输出：这一帧的所有平面的pcl点格式的集合（1个点云）
   */
  void planesToPoints(const std::vector<std::shared_ptr<Plane>>& planes_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& points_ptr);

  /**
   * @brief 【优化】检查精匹配的结果是否合规
   * @param transform 
   * @return true 
   * @return false 
   */
  bool checkPlaneIcpResult(const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform);

  // generate BtcDescs from a point cloud
  void GenerateBtcDescs(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                        const int frame_id, std::vector<BTC> &btcs_vec);

  /**
   * @brief 
   * @param btcs_vec 输入的BTC描述符列表
   * @param loop_result 返回的匹配结果（最佳候选帧ID和匹配分数）
   * @param loop_transform 返回的最佳变换（平移和旋转）
   * @param loop_std_pair 返回的本帧-最优回环帧 之间的成功的BTC匹配对们
   */
  void SearchLoop(const std::vector<BTC> &btcs_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<BTC, BTC>> &loop_std_pair);

  // add descriptors to database
  void AddBtcDescs(const std::vector<BTC> &btcs_vec);

  // Geometrical optimization by plane-to-plane icp
  bool PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform_opti);


 private:
  /*Following are sub-processing functions*/

  // voxelization and plane detection
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  // acquire planes from voxel_map
  void get_plane(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
                 std::vector<std::shared_ptr<Plane>> &planes);

  void get_project_plane(
      std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
      std::vector<std::shared_ptr<Plane>> &project_plane_list);

  void merge_plane(std::vector<std::shared_ptr<Plane>> &origin_list,
                   std::vector<std::shared_ptr<Plane>> &merge_plane_list);

  // extract corner points from pre-build voxel map and clouds

  void binary_extractor(
      const std::vector<std::shared_ptr<Plane>> proj_plane_list,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      std::vector<BinaryDescriptor> &binary_descriptor_list);

  void extract_binary(const Eigen::Vector3d &project_center,
                      const Eigen::Vector3d &project_normal,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::vector<BinaryDescriptor> &binary_list);

  // non maximum suppression, to control the number of corners
  void non_maxi_suppression(std::vector<BinaryDescriptor> &binary_list);

  // build BTCs from corner points.
  void generate_btc(const std::vector<BinaryDescriptor> &binary_list,
                    const int &frame_id, std::vector<BTC> &btc_list);

  // Select a specified number of candidate frames according to the number of
  // BtcDescs rough matches
  void candidate_selector(const std::vector<BTC> &btcs_vec,
                          std::vector<BTCMatchList> &candidate_matcher_vec);

  // Get the best candidate frame by geometry check
  void candidate_verify(
      const BTCMatchList &candidate_matcher, double &verify_score,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
      std::vector<std::pair<BTC, BTC>> &sucess_match_vec);

  // Get the transform between a matched std pair
  void triangle_solver(std::pair<BTC, BTC> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);

  // Geometrical verification by plane-to-plane icp threshold
  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
};

#endif  // BTC_H