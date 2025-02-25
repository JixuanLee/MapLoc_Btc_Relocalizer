#include "include/btc.h"

void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting) {
  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << config_file
              << std::endl;
    exit(-1);
  }

  // for binary descriptor
  config_setting.useful_corner_num_ = fSettings["useful_corner_num"];
  config_setting.plane_merge_normal_thre_ =
      fSettings["plane_merge_normal_thre"];
  config_setting.plane_merge_dis_thre_ = fSettings["plane_merge_dis_thre"];
  config_setting.plane_detection_thre_ = fSettings["plane_detection_thre"];
  config_setting.voxel_size_ = fSettings["voxel_size"];
  config_setting.voxel_init_num_ = fSettings["voxel_init_num"];
  config_setting.proj_plane_num_ = fSettings["proj_plane_num"];
  config_setting.proj_image_resolution_ = fSettings["proj_image_resolution"];
  config_setting.proj_image_high_inc_ = fSettings["proj_image_high_inc"];
  config_setting.proj_dis_min_ = fSettings["proj_dis_min"];
  config_setting.proj_dis_max_ = fSettings["proj_dis_max"];
  config_setting.summary_min_thre_ = fSettings["summary_min_thre"];
  config_setting.line_filter_enable_ = fSettings["line_filter_enable"];

  // std descriptor
  config_setting.descriptor_near_num_ = fSettings["descriptor_near_num"];
  config_setting.descriptor_min_len_ = fSettings["descriptor_min_len"];
  config_setting.descriptor_max_len_ = fSettings["descriptor_max_len"];
  config_setting.non_max_suppression_radius_ = fSettings["max_constrait_dis"];
  config_setting.std_side_resolution_ = fSettings["triangle_resolution"];

  // candidate search
  config_setting.skip_near_num_ = fSettings["skip_near_num"];
  config_setting.candidate_num_ = fSettings["candidate_num"];
  config_setting.rough_dis_threshold_ = fSettings["rough_dis_threshold"];
  config_setting.similarity_threshold_ = fSettings["similarity_threshold"];
  config_setting.icp_threshold_ = fSettings["icp_threshold"];
  config_setting.normal_threshold_ = fSettings["normal_threshold"];
  config_setting.dis_threshold_ = fSettings["dis_threshold"];

  std::cout << "Sucessfully load config file:" << config_file << std::endl;
}

/**
 * @brief 一种手动降采样方法，每一块体素内，只保留一个点，这个点的属性是该体素内点属性的平均值。
 * @param pl_feat 
 * @param voxel_size 
 */
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  // int intensity = rand() % 255; // 限制其在0-254随机 unused
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    // 计算该点在体素尺度下的坐标（位置），即体素的坐标
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    // 对应体素的位置类对象（整数化），即所对应的体素ID
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }

  plsize = voxel_map.size(); // 改为点云所归属的体素的总数量
  pl_feat.clear();
  pl_feat.resize(plsize);

  // 输入点云改为体素属性输出（属性：体素内点云均值）
  uint i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

/**
 * @brief 评估二进制描述符的相似性：共有占据区间 占 所有占据区间的比例
 * @param b1 输入：第一个二进制描述符
 * @param b2 输入：第二个二进制描述符
 * @return double 
 */
double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2) {
  double dis = 0;// 相同分布的区间总数
  for (size_t i = 0; i < b1.occupy_array_.size(); i++) {
    // to be debug hanming distance
    if (b1.occupy_array_[i] == true && b2.occupy_array_[i] == true) {
      dis += 1;
    }
  }
  // 共有占据区间 占 所有占据区间的比例
  return 2 * dis / (b1.summary_ + b2.summary_);
}

bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b) {
  return (a.summary_ > b.summary_);
}

/**
 * @brief 平面（体素）的点数量越多，greater
 * @param plane1 
 * @param plane2 
 * @return true 
 * @return false 
 */
bool plane_greater_sort(std::shared_ptr<Plane> plane1,
                        std::shared_ptr<Plane> plane2) {
  return plane1->points_size_ > plane2->points_size_;
}

void OctoTree::init_octo_tree() {
  if (voxel_points_.size() > config_setting_.voxel_init_num_) {  // 如果该体素包含的点足够多，初始化平面
    init_plane();
  }
}

/**
 * @brief 判断体素内的点是否构成平面，若构成，则对平面参数进行评估。
 */
void OctoTree::init_plane() {
  plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
  plane_ptr_->center_ = Eigen::Vector3d::Zero();
  plane_ptr_->normal_ = Eigen::Vector3d::Zero();
  plane_ptr_->points_size_ = voxel_points_.size();
  plane_ptr_->radius_ = 0;
  
  // 通过体素点计算平面的无偏平均协方差与平均中心点
  for (auto pi : voxel_points_) {
    plane_ptr_->covariance_ += pi * pi.transpose();
    plane_ptr_->center_ += pi;
  }
  plane_ptr_->center_ = plane_ptr_->center_ / plane_ptr_->points_size_;
  plane_ptr_->covariance_ =
      plane_ptr_->covariance_ / plane_ptr_->points_size_ -
      plane_ptr_->center_ * plane_ptr_->center_.transpose();

  Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3d::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax; // 找到3个特征值的索引
  if (evalsReal(evalsMin) < config_setting_.plane_detection_thre_) {
    plane_ptr_->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane_ptr_->min_eigen_value_ = evalsReal(evalsMin);
    plane_ptr_->radius_ = sqrt(evalsReal(evalsMax));
    plane_ptr_->is_plane_ = true;

    plane_ptr_->d_ = -(plane_ptr_->normal_(0) * plane_ptr_->center_(0) +
                       plane_ptr_->normal_(1) * plane_ptr_->center_(1) +
                       plane_ptr_->normal_(2) * plane_ptr_->center_(2));
    plane_ptr_->p_center_.x = plane_ptr_->center_(0);
    plane_ptr_->p_center_.y = plane_ptr_->center_(1);
    plane_ptr_->p_center_.z = plane_ptr_->center_(2);
    plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
    plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
    plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
  } else {
    plane_ptr_->is_plane_ = false;  // 不是平面
  }
}

void publish_binary(const std::vector<BinaryDescriptor> &binary_list,
                    const Eigen::Vector3d &text_color,
                    const std::string &text_ns,
                    const ros::Publisher &text_publisher) {
  visualization_msgs::MarkerArray text_array;
  visualization_msgs::Marker text;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.ns = text_ns;
  text.color.a = 0.8;  // Don't forget to set the alpha!
  text.scale.z = 0.08;
  text.pose.orientation.w = 1.0;
  text.header.frame_id = "odom";
  for (size_t i = 0; i < binary_list.size(); i++) {
    text.pose.position.x = binary_list[i].location_[0];
    text.pose.position.y = binary_list[i].location_[1];
    text.pose.position.z = binary_list[i].location_[2];
    std::ostringstream str;
    str << std::to_string((int)(binary_list[i].summary_));
    text.text = str.str();
    text.scale.x = 0.5;
    text.scale.y = 0.5;
    text.scale.z = 0.5;
    text.color.r = text_color[0];
    text.color.g = text_color[1];
    text.color.b = text_color[2];
    text.color.a = 1;
    text.id++;
    text_array.markers.push_back(text);
  }
  for (int i = 1; i < 100; i++) {
    text.color.a = 0;
    text.id++;
    text_array.markers.push_back(text);
  }
  text_publisher.publish(text_array);
  return;
}

void publish_std_list(const std::vector<BTC> &btc_list,
                      const ros::Publisher &std_publisher) {
  // publish descriptor
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "std";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.5;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "odom";
  m_line.id = 0;
  m_line.points.clear();
  m_line.color.r = 0;
  m_line.color.g = 1;
  m_line.color.b = 0;
  m_line.color.a = 1;
  for (auto var : btc_list) {
    geometry_msgs::Point p;
    p.x = var.binary_A_.location_[0];
    p.y = var.binary_A_.location_[1];
    p.z = var.binary_A_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_B_.location_[0];
    p.y = var.binary_B_.location_[1];
    p.z = var.binary_B_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.binary_C_.location_[0];
    p.y = var.binary_C_.location_[1];
    p.z = var.binary_C_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_B_.location_[0];
    p.y = var.binary_B_.location_[1];
    p.z = var.binary_B_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.binary_C_.location_[0];
    p.y = var.binary_C_.location_[1];
    p.z = var.binary_C_.location_[2];
    m_line.points.push_back(p);
    p.x = var.binary_A_.location_[0];
    p.y = var.binary_A_.location_[1];
    p.z = var.binary_A_.location_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 1000 * 3; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

/**
 * @brief 对二进制三角形描述符进行可视化
 * @param match_std_list BTC匹配对集合
 * @param transform1 
 * @param transform2 
 * @param std_publisher 
 */
void publish_std(const std::vector<std::pair<BTC, BTC>> &match_std_list,
                 const Eigen::Matrix4d &transform1,
                 const Eigen::Matrix4d &transform2,
                 const ros::Publisher &std_publisher) {
  // publish descriptor
  // bool transform_enable = true;
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "odom";
  m_line.id = 0;

  int max_pub_cnt = 1;

  // 把所有匹配的三角形都描绘出来
  for (auto var : match_std_list) {
    if (max_pub_cnt > 100) { // 限制最多发布100个三角形
      break;
    }

    max_pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    // m_line.color.r = 0 / 255;
    // m_line.color.g = 233.0 / 255;
    // m_line.color.b = 0 / 255;
    m_line.color.r = 252.0 / 255;
    m_line.color.g = 233.0 / 255;
    m_line.color.b = 79.0 / 255;
    geometry_msgs::Point p;
    Eigen::Vector3d t_p;

    // 2nd A-B线段
    t_p = var.second.binary_A_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_B_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    // 2nd B-C线段
    t_p = var.second.binary_C_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_B_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    // 2nd A-C线段
    t_p = var.second.binary_C_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.second.binary_A_.location_;
    t_p = transform2.block<3, 3>(0, 0) * t_p + transform2.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // another
    m_line.points.clear();
    // 252; 233; 79

    // 更新线段颜色
    m_line.color.r = 1; // 白色
    m_line.color.g = 1;
    m_line.color.b = 1;
    // m_line.color.r = 252.0 / 255;
    // m_line.color.g = 233.0 / 255;
    // m_line.color.b = 79.0 / 255;

    // 1st A-B线段
    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    // 1st B-C线段
    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.first.binary_B_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    // 1st A-C线段
    t_p = var.first.binary_C_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    t_p = var.first.binary_A_.location_;
    t_p = transform1.block<3, 3>(0, 0) * t_p + transform1.block<3, 1>(0, 3);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // debug
    // std_publisher.publish(ma_line);
    // std::cout << "var first: " << var.first.triangle_.transpose()
    //           << " , var second: " << var.second.triangle_.transpose()
    //           << std::endl;
    // getchar();
  }

  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++; // id 被clear之后从0开始，可以清除之前的显示。
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

/**
 * @brief 评判输入的一组BTC匹配对的平均三角形形状相似程度。
 * 距离：每一对三角形边长之差的平方和开根/第一个三角形边长的平方和开根，因此评判指标消除了绝对大小的影响，只评估形状相似性。
 * @param match_std_list 
 * @return double 输入的一组BTC匹配对的平均形状相似程度
 */
double calc_triangle_dis(
    const std::vector<std::pair<BTC, BTC>> &match_std_list) {
  double mean_triangle_dis = 0;
  for (auto var : match_std_list) {
    mean_triangle_dis += (var.first.triangle_ - var.second.triangle_).norm() /
                         var.first.triangle_.norm();
  }
  if (match_std_list.size() > 0) {
    mean_triangle_dis = mean_triangle_dis / match_std_list.size();
  } else {
    mean_triangle_dis = -1;
  }
  return mean_triangle_dis;
}

/**
 * @brief 评判输入的一组BTC匹配对的平均二进制特征相似程度。
 * 距离：每一对BTC的二进制之间的高程占据区间重合得分的均值的均值，评估2个BTC的一共6个顶点在高程上是不是相似。
 * @param match_std_list 
 * @return double 输入的一组BTC匹配对的平均二进制特征相似程度
 */
double calc_binary_similaity(
    const std::vector<std::pair<BTC, BTC>> &match_std_list) 
{
  double mean_binary_similarity = 0;
  for (auto var : match_std_list) {
    mean_binary_similarity +=
        (binary_similarity(var.first.binary_A_, var.second.binary_A_) +
         binary_similarity(var.first.binary_B_, var.second.binary_B_) +
         binary_similarity(var.first.binary_C_, var.second.binary_C_)) /
        3;
  }
  if (match_std_list.size() > 0) {
    mean_binary_similarity = mean_binary_similarity / match_std_list.size();
  } else {
    mean_binary_similarity = -1;
  }
  return mean_binary_similarity;
}

/**
 * @brief 计算从一个法向量到四元数的旋转
 * @param vec 
 * @param axis 
 * @param q 
 */
void CalcQuation(const Eigen::Vector3d &vec, const int axis,
  geometry_msgs::Quaternion &q) 
{
  Eigen::Vector3d x_body = vec; // 定义局部坐标系的 x 轴方向，与输入法向量 vec 对齐
  Eigen::Vector3d y_body(1, 1, 0); // 定义局部坐标系的初始 y 轴方向，初始值为 (1, 1, 0)

  if (x_body.norm() == 0) {
    throw std::invalid_argument("Input vector cannot be zero.");
  }

  // 根据 x_body 的 z 分量是否为零，调整 y_body 的 z 分量
  if (x_body(2) != 0) {
    y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
  } else {
    // 如果 x_body 的 z 分量为零，检查 y 分量是否为零
    if (x_body(1) != 0) {
      y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    } else {
      // 如果 x_body 的 y 和 z 分量都为零，将 y_body 的 x 分量设置为零
      y_body(0) = 0;
    }
  }

  y_body.normalize();// 归一化 y_body，确保其为单位向量
  Eigen::Vector3d z_body = x_body.cross(y_body);// 计算局部坐标系的 z 轴方向，通过 x_body 和 y_body 的叉积得到

  // 构造旋转矩阵，将局部坐标系的 x、y、z 轴方向存储到矩阵中
  Eigen::Matrix3d rot;
  rot << x_body(0), x_body(1), x_body(2),
        y_body(0), y_body(1), y_body(2),
        z_body(0), z_body(1), z_body(2);

  // 转置旋转矩阵，得到从局部坐标系到全局坐标系的旋转矩阵
  Eigen::Matrix3d rotation = rot.transpose();

  // 如果指定的轴为 z 轴（axis == 2），则应用一个额外的旋转
  if (axis == 2) {
    Eigen::Matrix3d rot_inc;
    rot_inc << 0, 0, 1,
               0, 1, 0,
              -1, 0, 0;
    rotation = rotation * rot_inc;
  }

  // 将旋转矩阵转换为四元数
  Eigen::Quaterniond eq(rotation);

  // 将四元数的值赋给输出参数 q
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

/**
 * @brief 发布一个表示平面的Marker,输入为1个平面。【改进】
 * @param plane_pub 
 * @param plane_ns 
 * @param plane_id 
 * @param normal_ps 存储很多个平面，每个平面包含：位置法向量信息、可视化参数（颜色、半径与比例）
 */
void pubPlane(
  const ros::Publisher &plane_pub, const std::string plane_ns, const int plane_id, const double during_time, 
  const std::vector<std::pair<pcl::PointXYZINormal,std::pair<Eigen::Vector3d,std::pair<float, float>>>> normal_ps) 
{
  // 创建一个Marker消息
  visualization_msgs::MarkerArray planes;
  int timer = 0;
  for (auto normal_p:normal_ps)
  {
    visualization_msgs::Marker plane;
    plane.header.frame_id = "odom"; // 设置Marker的参考坐标系
    plane.header.stamp = ros::Time(); // 设置时间戳
    plane.ns = plane_ns; // 设置命名空间，自定义
    plane.id = (plane_id - 1) * normal_ps.size() + ++timer; // 设置ID，可以理解为submap-id 【改进】避免ID重复
    plane.type = visualization_msgs::Marker::CUBE; // 设置类型为立方体, TRIANGLE_LIST怎么样？
    plane.action = visualization_msgs::Marker::ADD; // 设置动作类型为添加
  
    // 设置Marker的位置
    plane.pose.position.x = normal_p.first.x;
    plane.pose.position.y = normal_p.first.y;
    plane.pose.position.z = normal_p.first.z;
  
    // 创建一个四元数
    geometry_msgs::Quaternion q;
    // 将法线向量转换为四元数
    Eigen::Vector3d normal_vec(normal_p.first.normal_x, normal_p.first.normal_y, normal_p.first.normal_z);
    CalcQuation(normal_vec, 2, q);
    // 设置Marker的朝向
    plane.pose.orientation = q;
    // 设置Marker的尺寸
    plane.scale.x = normal_p.second.second.second * normal_p.second.second.first; // 比例*半径
    plane.scale.y = normal_p.second.second.second * normal_p.second.second.first;
    plane.scale.z = 0.1;
    // 设置Marker的颜色
    plane.color.a = 0.7; // 透明度
    plane.color.r = std::min(1.0, std::max(0.0, normal_p.second.first[0]));
    plane.color.g = std::min(1.0, std::max(0.0, normal_p.second.first[1]));
    plane.color.b = std::min(1.0, std::max(0.0, normal_p.second.first[2]));
    // 设置Marker的生命周期
    plane.lifetime = ros::Duration(during_time);
    // plane.lifetime = ros::Duration();
    planes.markers.push_back(plane);
  }

  // 发布Marker
  // std::cout<<"DEBUG = "<<plane<<std::endl;
  plane_pub.publish(planes);
}

/**
 * @brief 使用trans后的全局本帧点云，生成BTC描述符
 * @param input_cloud trans后的全局本帧点云
 * @param frame_id submap_id
 * @param btcs_vec 输出结果
 */
void BtcDescManager::GenerateBtcDescs(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, const int frame_id,
    std::vector<BTC> &btcs_vec) {  
  // step1,体素化与平面检测
  std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map; // 点云数据转换为 [体素-八叉树] 格式进行hash管理
  init_voxel_map(input_cloud, voxel_map); // 输入点云进行体素化与八叉树转换

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZINormal>); // 本帧点云析出的平面的特征点云
  std::vector<std::shared_ptr<Plane>> thisFramePlanes;
  get_plane(voxel_map, plane_cloud, thisFramePlanes);
  plane_ori_vec.push_back(thisFramePlanes); // [优化]

  if (print_debug_info_) {
    std::cout << "[Description] planes size:" << plane_cloud->size()
              << std::endl;
  }

  plane_cloud_vec_.push_back(plane_cloud);

  // step2,平面合并
  std::vector<std::shared_ptr<Plane>> proj_plane_list; // 一次合并后的平面Vec
  std::vector<std::shared_ptr<Plane>> merge_plane_list; // 二次合并后的平面Vec
  get_project_plane(voxel_map, proj_plane_list); // 将voxel_map的平面进行合并

  if (proj_plane_list.size() == 0) {
    std::shared_ptr<Plane> single_plane(new Plane);
    single_plane->normal_ << 0, 0, 1;
    single_plane->center_ << input_cloud->points[0].x, input_cloud->points[0].y,
        input_cloud->points[0].z;
    merge_plane_list.push_back(single_plane);
  } else {
    sort(proj_plane_list.begin(), proj_plane_list.end(), plane_greater_sort); // 将一次合并后的平面进行排序
    merge_plane(proj_plane_list, merge_plane_list);// 二次合并
    sort(merge_plane_list.begin(), merge_plane_list.end(), plane_greater_sort); // 将二次合并后的平面进行排序
  }
  plane_merged_vec.push_back(merge_plane_list); // 【改进】

  // step3, binary描述符提取
  std::vector<BinaryDescriptor> binary_list;
  binary_extractor(merge_plane_list, input_cloud, binary_list);
  history_binary_list_.push_back(binary_list);
  // corner_cloud_vec_.push_back(corner_points);
  if (print_debug_info_) {
    std::cout << "[Description] binary size:" << binary_list.size()
              << std::endl;
  }

  // step4, stable triangle描述符生成
  btcs_vec.clear();
  generate_btc(binary_list, frame_id, btcs_vec);
  if (print_debug_info_) {
    std::cout << "[Description] btcs size:" << btcs_vec.size() << std::endl;
  }

  // step5, 清理内存
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    delete (iter->second);
  }

  return;
}

/**
 * @brief 基于输入的本帧BTC们，找到N个候选的old帧，再从候选者中找到最优的old帧，如果这个最优old还是不够优，就不算回环，否则回环成功。
 * @param btcs_vec 输入的BTC描述符列表
 * @param loop_result 返回的匹配结果（最佳候选帧ID和匹配分数）
 * @param loop_transform 返回的最佳变换（平移和旋转），若回环失败则不做更改
 * @param loop_std_pair 返回的本帧-最优回环帧 之间的成功的BTC匹配对们
 */
void BtcDescManager::SearchLoop(
  const std::vector<BTC> &btcs_vec, 
  std::pair<int, double> &loop_result, 
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
  std::vector<std::pair<BTC, BTC>> &loop_std_pair) 
{ 
  // 检查输入是否为空
  if (btcs_vec.size() == 0) {
    ROS_ERROR_STREAM("[BTC-SearchLoop] No STDescs!"); // 输出错误信息
    loop_result = std::pair<int, double>(-1, 0); // 返回无效结果
    return;
  }

  // Step 1: 选择候选匹配帧
  auto t1 = std::chrono::high_resolution_clock::now(); // 记录开始时间
  std::vector<BTCMatchList> candidate_matcher_vec; // 存储候选匹配帧的列表（有N个old帧入围候选）
  candidate_selector(btcs_vec, candidate_matcher_vec); // 调用候选选择函数
  auto t2 = std::chrono::high_resolution_clock::now(); // 记录结束时间

  // Step 2: 从候选匹配帧中选择最佳匹配
  double best_score = 0; // 最佳匹配分数
  int best_candidate_id = -1; // 最佳候选帧ID
  // int triggle_candidate = -1; // 触发候选帧的索引（未使用，可能是预留的）
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform; // 最佳变换（平移和旋转）
  std::vector<std::pair<BTC, BTC>> best_sucess_match_vec; // 最佳成功匹配对列表

  // 遍历所有候选匹配帧，找到最优的那个old帧
  for (size_t i = 0; i < candidate_matcher_vec.size(); i++) 
  {
    double verify_score = -1; // 当前候选帧的验证分数
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose; // 相对位姿（平移和旋转）
    std::vector<std::pair<BTC, BTC>> sucess_match_vec; // 成功匹配对列表

    // 验证当前候选帧
    candidate_verify(candidate_matcher_vec[i], verify_score, relative_pose, sucess_match_vec);

    // 如果启用了调试信息输出，打印当前候选帧的验证信息
    if (print_debug_info_) {
      std::cout << "[Retreival] try frame:"
                << candidate_matcher_vec[i].match_id_.second << ", rough size:"
                << candidate_matcher_vec[i].match_list_.size()
                << ", score:" << verify_score << std::endl;
    }

    // 如果当前候选帧的验证分数高于最佳分数，则更新最佳匹配信息
    if (verify_score > best_score) {
      best_score = verify_score; // 更新最佳分数
      best_candidate_id = candidate_matcher_vec[i].match_id_.second; // 更新最佳候选帧ID
      best_transform = relative_pose; // 更新最佳变换
      best_sucess_match_vec = sucess_match_vec; // 更新最佳成功匹配对列表
      // triggle_candidate = i; // 更新触发候选帧索引
    }
  }

  auto t3 = std::chrono::high_resolution_clock::now(); // 记录结束时间

  // 如果启用了调试信息输出，打印最佳候选帧的信息
  if (print_debug_info_) {
    std::cout << "[Retreival] best candidate:" << best_candidate_id
              << ", score:" << best_score << std::endl;
  }

  // 【优化】基于距离进行判断
  // 在大体信任给定的定位pose数据的情况下，可以启用
  static double loop_dis_threshold = 20.0;
  double loop_dis = best_transform.first.norm();

  // 如果最佳分数大于ICP阈值，则返回成功匹配结果
  // 这个最佳分数，是plane-plane的重合得分
  // if (best_score > config_setting_.icp_threshold_) {
  if (best_score > config_setting_.icp_threshold_ && loop_dis < loop_dis_threshold) { // 【优化】
    loop_result = std::pair<int, double>(best_candidate_id, best_score); // 返回最佳候选帧ID和分数
    loop_transform = best_transform; // 返回最佳变换
    loop_std_pair = best_sucess_match_vec; // 返回最佳成功匹配对列表
    return;
  } else {
    // 否则返回无效结果
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
}

/**
 * @brief 将输入的BTC们，一个一个的查阅其三角形，按照三角形形状找到数据库中对应的位置，存入该BTC。
 * @param btcs_vec 
 */
void BtcDescManager::AddBtcDescs(const std::vector<BTC> &btcs_vec) {
  // update frame id
  for (auto single_std : btcs_vec) {
    // calculate the position of single std
    BTC_LOC position;
    position.x = (int)(single_std.triangle_[0] + 0.5); // 四舍五入到最近的int
    position.y = (int)(single_std.triangle_[1] + 0.5);
    position.z = (int)(single_std.triangle_[2] + 0.5);
    auto iter = data_base_.find(position);
    // 如果这个三角形有过类似的了，那就在这个三角形索引下直接存储该BTC
    if (iter != data_base_.end()) {
      data_base_[position].push_back(single_std);
    } 
    // 如果这个三角形是全新的，那就在新的三角形索引下存储它。
    else {
      std::vector<BTC> descriptor_vec;
      descriptor_vec.push_back(single_std);
      data_base_[position] = descriptor_vec;
    }
  }
  return;
}

/**
 * @brief 【优化】将某一帧的Plane格式平面全部转为pcl点格式
 * @param planes_ptr 输入：某一帧的所有平面的Plane格式的集合
 * @param points_ptr 输出：这一帧的所有平面的pcl点格式的集合（1个点云）
 */
void BtcDescManager::planesToPoints(const std::vector<std::shared_ptr<Plane>>& planes_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& points_ptr)
{
  for (auto plane_ptr:planes_ptr)
  {
    pcl::PointXYZINormal out;
    out.x = plane_ptr->center_[0];
    out.y = plane_ptr->center_[1];
    out.z= plane_ptr->center_[2];
    out.normal_x = plane_ptr->normal_[0];
    out.normal_y = plane_ptr->normal_[1];
    out.normal_z = plane_ptr->normal_[2];
    points_ptr->points.push_back(out);
  }
  points_ptr->header.frame_id = "odom"; // 表示这些平面的位置、法向量是坐落在odom下的。
}


bool BtcDescManager::checkPlaneIcpResult(const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform)
{
  // TODO：待完善
  // static double thresh_t = 3.0;
  // if (transform.first.norm() > thresh_t) // 20250217:这个检查没啥道理
  // {
  //   std::cout<<"[BTC] PlaneIcp result is bad in t."<<std::endl<<std::endl;
  //   return false;
  // }

  return true;
}

/**
 * @brief 前提：src来自本帧，tar来自最优粗匹配帧。针对每一个src平面，基于初始变换将其同步搭配tar坐标系，
 * 进而找到该src理应最靠近的tar平面。建立残差：将src平面基于初始变换同步到tar坐标系，然后算出src平面位置点到tar平面的[点到面]距离作为残差。
 * 优化项：初始变换可以分出旋转与平移，分别将二者作为待优化参数。
 * 执行优化，算出“当残差最小”时的待优化参数，直接替换初始变换，输出。
 * @param source_cloud 输入：本帧src平面特征点云
 * @param target_cloud 输入：最优粗匹配帧tar平面特征点云
 * @param transform 输入：初始变换（预测的src与tar帧间变换）
 * @param transform_opti 输出:优化后的变换
 * @param doOptiCheck 是否执行结果检查
 * @return true 
 * @return false 
 */
bool BtcDescManager::PlaneGeomrtricIcp(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform,
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform_opti, 
  const bool doOptiCheck)
{
  // Step 1: 构建 KD 树用于快速搜索邻近点
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 将tar平面转换为仅包含位置信息的点云
  for (size_t i = 0; i < target_cloud->size(); i++) {
  // for (size_t i = 0; i < std::min(20, int(source_cloud->size())); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x; // 设置点的 x 坐标
    pi.y = target_cloud->points[i].y; // 设置点的 y 坐标
    pi.z = target_cloud->points[i].z; // 设置点的 z 坐标
    input_cloud->push_back(pi); // 将点添加到点云中
  }

  if (input_cloud->points.empty()) {
    std::cerr << "Error: Target point cloud is empty." << std::endl;
    return false;
  }

  kd_tree->setInputCloud(input_cloud); // 将点云数据输入到 KD 树中，在tar中搜索

  // Step 2: 初始化 Ceres 优化问题
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold; // 四元数流形
  ceres::Problem problem; // Ceres 优化问题
  ceres::LossFunction *loss_function = nullptr; // 不使用损失函数

  // 从输入变换中提取旋转和平移
  Eigen::Matrix3d rot = transform.second; // 旋转矩阵
  Eigen::Quaterniond q(rot); // 将旋转矩阵转换为四元数
  Eigen::Vector3d t = transform.first; // 平移向量

  // 将四元数和平移向量转换为 Ceres 参数块
  double para_q[4] = {q.x(), q.y(), q.z(), q.w()}; // 四元数参数
  double para_t[3] = {t(0), t(1), t(2)}; // 平移参数

  // 将参数块添加到优化问题中
  problem.AddParameterBlock(para_q, 4, quaternion_manifold); // 添加四元数参数块
  problem.AddParameterBlock(para_t, 3); // 添加平移参数块

  // 映射 Ceres 参数块到 Eigen 对象
  Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q); // 当前四元数，通过内存映射，可以直接操作q_last_curr，而不用复制数据
  Eigen::Map<Eigen::Vector3d> t_last_curr(para_t); // 当前平移向量

  // Step 3: 遍历源点云，构建优化问题的残差块
  std::vector<int> pointIdxNKNSearch(1); // 存储搜索到的邻近点索引
  std::vector<float> pointNKNSquaredDistance(1); // 存储搜索到的邻近点距离
  int useful_match = 0; // 记录有效匹配点的数量

  // // for debug
  // static int number2save = 1;
  // static int nowNumber = 0;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr curr_nice_plane_pointcloud_trans_by_looptransform(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr curr_nice_plane_pointcloud_odom(new pcl::PointCloud<pcl::PointXYZINormal>());
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr loop_nice_plane_pointcloud_odom(new pcl::PointCloud<pcl::PointXYZINormal>());

  // 遍历每个src平面
  for (size_t i = 0; i < source_cloud->size(); i++) 
  // for (size_t i = 0; i < std::min(15, int(source_cloud->size())); i++) 
  {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i]; // 当前搜索点
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z); // 当前点的位置
    // std::cout<<"$$$$$ pi = "<<pi<<std::endl;
    pi = rot * pi + t; // 将当前点变换到tar坐标系
    // std::cout<<"$$$$$ rot = "<<rot<<" t = "<<t<<std::endl;

    // 将变换后的点转换为 pcl::PointXYZ 类型
    pcl::PointXYZ use_search_point;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];

    // 变换当前点的法向量到tar坐标系
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z);
    ni = rot * ni;

    // 在tar的平面中，找1个与当前src平面位置最靠近的tar平面
    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
    {
      // std::cout<<"@@@@@ = "<<std::sqrt(pointNKNSquaredDistance[0])<<std::endl;
      // std::cout<<"&&&&& = "<<pointIdxNKNSearch[0]<<std::endl;
      // std::cout<<"$$$$$ = "<<target_cloud->points[pointIdxNKNSearch[0]]<<std::endl;
      pcl::PointXYZINormal nearstPoint = target_cloud->points[pointIdxNKNSearch[0]]; // 最近tar平面
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z); // 最近tar的位置
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y, nearstPoint.normal_z); // 最近tar的法向量

      // 计算法向量差异和距离
      Eigen::Vector3d normal_inc = ni - tni; // 法向量差异
      Eigen::Vector3d normal_add = ni + tni; // 法向量和
      double point_to_point_dis = (pi - tpi).norm(); // src-最近tar的平面位置点的点到点距离
      double point_to_plane = fabs(tni.transpose() * (pi - tpi)); // src位置点-最近tar平面的点到面距离

      // 判断是否为有效匹配点:2个平面必须几乎平行、平面点到面距离很小、平面位置点到点距离够近
// std::cout << "[Debug] Match attempt " << i
// << ": normal_diff = " << normal_inc.norm()
// << ", normal_sum = " << normal_add.norm()
// << ", point_to_plane = " << point_to_plane
// << ", point_to_point = " << point_to_point_dis
// << std::endl;
      if ((normal_inc.norm() < config_setting_.normal_threshold_ ||
          normal_add.norm() < config_setting_.normal_threshold_) &&
          point_to_plane < config_setting_.dis_threshold_ &&
          point_to_point_dis < 3) 
      {
        useful_match++; // 增加有效匹配点计数

        // 创建 Ceres 残差块
        ceres::CostFunction *cost_function;
        Eigen::Vector3d curr_point(source_cloud->points[i].x, // src平面的位置（未变换坐标系，是全局坐标系下位置）
                                  source_cloud->points[i].y,
                                  source_cloud->points[i].z);
        Eigen::Vector3d curr_normal(source_cloud->points[i].normal_x, // src平面的法向量（未变换坐标系，是全局坐标系下法向量）
                                    source_cloud->points[i].normal_y,
                                    source_cloud->points[i].normal_z);


        // // for debug
        // curr_nice_plane_pointcloud_trans_by_looptransform->points.push_back(use_search_point);
        // curr_nice_plane_pointcloud_odom->points.push_back(source_cloud->points[i]);
        // loop_nice_plane_pointcloud_odom->points.push_back(nearstPoint);

        // std::cout<<"This pair of plane : curr-loop:"<<i<<"-"<<pointIdxNKNSearch[0]<<std::endl;
        // std::cout<<"Curr plane-bef : pos"<<curr_point<<std::endl;
        // std::cout<<"Curr plane-bef : nor"<<curr_normal<<std::endl;
        // std::cout<<"$$$$$ Curr plane-aft = "<<pi<<std::endl;
        // std::cout<<"Loop plane : pos"<<tpi<<std::endl;
        // std::cout<<"Loop plane : nor"<<tni<<std::endl;
        // std::cout<<std::endl;

        // 创建点到平面的残差函数，输入了2个平面各自在全局下的位置、法向量，构造src平面位置点到其最近tar平面的距离（点到面距离）作为残差
        cost_function = PlaneSolver::Create(curr_point, curr_normal, tpi, tni); 
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t); // 添加残差块，包括残差、损失函数（无）、待优化参数
      }
    }
    
      // std::cout<<"@@@@@ = "<<std::sqrt(pointNKNSquaredDistance[0])<<std::endl;
      // std::cout<<"&&&&& = "<<pointIdxNKNSearch[0]<<std::endl;
      // std::cout<<"$$$$$ tar = "<<target_cloud->points[pointIdxNKNSearch[0]]<<std::endl;
      // std::cout<<"$$$$$ src-aft= "<<use_search_point<<std::endl;
      // std::cout<<"$$$$$ = "<<transform.first<<std::endl;

  }
  // for debug
  // std::cout<<"@@@@ use num = "<<useful_match<<std::endl;
  // // 设置点云的宽度和高度
  // if (number2save == nowNumber++)
  // {
  //   curr_nice_plane_pointcloud_trans_by_looptransform->width = curr_nice_plane_pointcloud_trans_by_looptransform->points.size();
  //   curr_nice_plane_pointcloud_trans_by_looptransform->height = 1;
  //   curr_nice_plane_pointcloud_trans_by_looptransform->is_dense = false;
  
  //   curr_nice_plane_pointcloud_odom->width = curr_nice_plane_pointcloud_odom->points.size();
  //   curr_nice_plane_pointcloud_odom->height = 1;
  //   curr_nice_plane_pointcloud_odom->is_dense = false;
  
  //   loop_nice_plane_pointcloud_odom->width = loop_nice_plane_pointcloud_odom->points.size();
  //   loop_nice_plane_pointcloud_odom->height = 1;
  //   loop_nice_plane_pointcloud_odom->is_dense = false;
  //   pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/curr_nice_plane_pointcloud_trans_by_looptransform.pcd", *curr_nice_plane_pointcloud_trans_by_looptransform);
  //   pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/curr_nice_plane_pointcloud_odom.pcd", *curr_nice_plane_pointcloud_odom);
  //   pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/loop_nice_plane_pointcloud_odom.pcd", *loop_nice_plane_pointcloud_odom);
  // }


  // Step 4: 配置并运行 Ceres 求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // 使用稀疏 Cholesky 求解器
  options.max_num_iterations = 100; // 最大迭代次数
  options.minimizer_progress_to_stdout = false;// 不输出优化进度
  ceres::Solver::Summary summary; // 优化结果摘要
  ceres::Solve(options, &problem, &summary); // 运行优化
  // std::cout << summary.BriefReport() << "\n"; // debug

  // Step 5: 更新变换结果
  Eigen::Quaterniond q_opt(para_q[3], para_q[0], para_q[1], para_q[2]); // 优化后的四元数
  rot = q_opt.toRotationMatrix(); // 将四元数转换为旋转矩阵
  t << t_last_curr(0), t_last_curr(1), t_last_curr(2); // 优化后的平移向量

  // 输出优化后的变换
  transform_opti.first = t;
  transform_opti.second = rot;
  if (doOptiCheck)
  {
    if (!checkPlaneIcpResult(transform_opti))
    {
      transform_opti = transform;
      return false;
    }
  }
  return true;

}

/**
 * @brief 使用全局下的输入点云构建voxel地图（包括八叉树及其平面）
 * @param input_cloud 
 * @param voxel_map 
 */
void BtcDescManager::init_voxel_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  uint plsize = input_cloud->size();

  for (uint i = 0; i < plsize; i++) {
    Eigen::Vector3d p_c(input_cloud->points[i].x, input_cloud->points[i].y,
                        input_cloud->points[i].z);
    double loc_xyz[3]; // 存储该点在体素坐标系下的坐标，将点分配到体素中
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] / config_setting_.voxel_size_;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position); // 完成对Voxel_map的构造，一个体素（即：体素坐标系整数坐标）对应一个八叉树对象，八叉树中存储该体素包含的点的实际坐标
    if (iter != voxel_map.end()) {
      voxel_map[position]->voxel_points_.push_back(p_c);
    } else {
      OctoTree *octo_tree = new OctoTree(config_setting_);
      voxel_map[position] = octo_tree;
      voxel_map[position]->voxel_points_.push_back(p_c);
    }
  }

  std::vector<std::unordered_map<VOXEL_LOC, OctoTree *>::iterator> iter_list;
  std::vector<size_t> index;
  size_t i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {  //遍历每一个体素，得到其迭代器与索引
    index.push_back(i);
    i++;
    iter_list.push_back(iter);
    // iter->second->init_octo_tree();
  }
  std::for_each( // 并行初始化八叉树对象
      std::execution::par_unseq, index.begin(), index.end(),
      [&](const size_t &i) { iter_list[i]->second->init_octo_tree(); });
}

/**
 * @brief 从Voxel_map中将包含平面的体素的平面的中心与法向量保存为点云格式。
 * @param voxel_map 
 * @param plane_cloud 
 */
void BtcDescManager::get_plane(
    const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
    std::vector<std::shared_ptr<Plane>> &planes) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      pcl::PointXYZINormal pi;
      pi.x = iter->second->plane_ptr_->center_[0];
      pi.y = iter->second->plane_ptr_->center_[1];
      pi.z = iter->second->plane_ptr_->center_[2];
      pi.normal_x = iter->second->plane_ptr_->normal_[0];
      pi.normal_y = iter->second->plane_ptr_->normal_[1];
      pi.normal_z = iter->second->plane_ptr_->normal_[2];
      plane_cloud->push_back(pi);

      // 【优化】
      planes.push_back(iter->second->plane_ptr_);
    }
  }
}

/**
 * @brief 在voxel_map所包含的平面中，找寻相似平面，并进行合并，并输出合并后的平面们
 * @param voxel_map 
 * @param project_plane_list 
 */
void BtcDescManager::get_project_plane(
  std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map, // 输入：体素地图，存储体素位置和对应的八叉树节点
  std::vector<std::shared_ptr<Plane>> &project_plane_list) // 输出：合并后的平面列表
{ 
  std::vector<std::shared_ptr<Plane>> origin_list; // 存储所有有效的平面

  // Step 1: 遍历体素地图，提取所有有效的平面
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) { // 如果当前体素的平面是有效的
      origin_list.push_back(iter->second->plane_ptr_); // 将平面添加到origin_list中
    }
  }

  // Step 2: 初始化所有平面的ID为0
  for (size_t i = 0; i < origin_list.size(); i++) origin_list[i]->id_ = 0;

  int current_id = 1; // 用于为合并的平面分配唯一ID

  // Step 3: 双向遍历所有平面，合并相似的平面
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) {
      // 计算两个平面的法向量差异和法向量和
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;

      // 计算两个平面之间的距离
      double dis1 =
          fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
          (*iter)->normal_(1) * (*iter2)->center_(1) +
          (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
          (*iter2)->normal_(1) * (*iter)->center_(1) +
          (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);

      // 如果两个平面的法向量差异或法向量和小于阈值，且距离小于阈值，则合并这两个平面
      if (normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
          normal_add.norm() < config_setting_.plane_merge_normal_thre_)
        if (dis1 < config_setting_.plane_merge_dis_thre_ &&
            dis2 < config_setting_.plane_merge_dis_thre_) {
          if ((*iter)->id_ == 0 && (*iter2)->id_ == 0) { // 如果两个平面都未分配ID
            (*iter)->id_ = current_id; // 分配新的ID（若ID≠0，则该id至少包含2个平面）
            (*iter2)->id_ = current_id;
            current_id++;
          } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0) // 如果其中一个平面已分配ID
            (*iter)->id_ = (*iter2)->id_; // 将未分配ID的平面设置为相同的ID
          else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0)
            (*iter2)->id_ = (*iter)->id_;
        }
    }
  }

  std::vector<std::shared_ptr<Plane>> merge_list; // 存储合并后的平面
  std::vector<int> merge_flag; // 记录已经合并的平面ID

  // Step 4: 遍历所有平面，合并具有相同ID的平面
  for (size_t i = 0; i < origin_list.size(); i++) {

    auto it = std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end()) 
      continue; // 如果当前平面已经合并过，则跳过
    if (origin_list[i]->id_ == 0) { // 如果平面未分配ID，则跳过
      continue;
    }

    // 找到了一个id≠0，可以合并，且未被合并过
    // 创建一个新的合并平面
    std::shared_ptr<Plane> merge_plane(new Plane);
    (*merge_plane) = (*origin_list[i]); // 初始化为当前平面
    bool is_merge = false; // 标记是否进行了合并

    // 二次遍历所有平面，查找具有相同ID的平面并进行合并
    for (size_t j = 0; j < origin_list.size(); j++) {
      if (i == j) continue; // 跳过自身
      if (origin_list[j]->id_ == origin_list[i]->id_) { // 如果平面ID相同
        is_merge = true; // 标记为已合并

        // 合并两个平面的协方差矩阵和中心点
        Eigen::Matrix3d P_PT1 =
            (merge_plane->covariance_ +
            merge_plane->center_ * merge_plane->center_.transpose()) *
            merge_plane->points_size_;
        Eigen::Matrix3d P_PT2 =
            (origin_list[j]->covariance_ +
            origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
            origin_list[j]->points_size_;
        Eigen::Vector3d merge_center =
            (merge_plane->center_ * merge_plane->points_size_ +
            origin_list[j]->center_ * origin_list[j]->points_size_) /
            (merge_plane->points_size_ + origin_list[j]->points_size_);
        Eigen::Matrix3d merge_covariance =
            (P_PT1 + P_PT2) /
                (merge_plane->points_size_ + origin_list[j]->points_size_) -
            merge_center * merge_center.transpose();

        // 更新合并后的平面信息
        merge_plane->covariance_ = merge_covariance;
        merge_plane->center_ = merge_center;
        merge_plane->points_size_ = merge_plane->points_size_ + origin_list[j]->points_size_;
        merge_plane->sub_plane_num_++; // 增加子平面数量

        // 计算合并后平面的法向量和半径
        Eigen::EigenSolver<Eigen::Matrix3d> es(merge_plane->covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        merge_plane->normal_ << evecs.real()(0, evalsMin),
            evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        merge_plane->radius_ = sqrt(evalsReal(evalsMax));
        merge_plane->d_ = -(merge_plane->normal_(0) * merge_plane->center_(0) +
                            merge_plane->normal_(1) * merge_plane->center_(1) +
                            merge_plane->normal_(2) * merge_plane->center_(2));

        // 更新平面中心点的坐标和法向量
        merge_plane->p_center_.x = merge_plane->center_(0);
        merge_plane->p_center_.y = merge_plane->center_(1);
        merge_plane->p_center_.z = merge_plane->center_(2);
        merge_plane->p_center_.normal_x = merge_plane->normal_(0);
        merge_plane->p_center_.normal_y = merge_plane->normal_(1);
        merge_plane->p_center_.normal_z = merge_plane->normal_(2);
      }
    }
    // 该不为0的id所属的平面已经全部合并完成

    // 如果进行了合并，则将合并后的平面添加到merge_list中
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_list.push_back(merge_plane);
    }
  }

  // Step 5: 将合并后的平面列表赋值给输出参数
  project_plane_list = merge_list;
}

/**
 * @brief 在输入的平面Vec中再找一遍相似平面，将相似的进行合并，加上单独的不相似的，一起输出
 * @param origin_list 
 * @param merge_plane_list 
 */
void BtcDescManager::merge_plane(
  std::vector<std::shared_ptr<Plane>> &origin_list, // 输入：原始平面列表
  std::vector<std::shared_ptr<Plane>> &merge_plane_list) { // 输出：合并后的平面列表
  // 如果原始平面列表只有一个平面，则直接返回该平面
  if (origin_list.size() == 1) {
    merge_plane_list = origin_list;
    return;
  }

  // Step 1: 初始化所有平面的ID为0
  for (size_t i = 0; i < origin_list.size(); i++) origin_list[i]->id_ = 0;

  int current_id = 1; // 用于为合并的平面分配唯一ID

  // Step 2: 遍历所有平面，合并相似的平面
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) {
      // 计算两个平面的法向量差异和法向量和
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;

      // 计算两个平面之间的距离
      double dis1 =
          fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
              (*iter)->normal_(1) * (*iter2)->center_(1) +
              (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
              (*iter2)->normal_(1) * (*iter)->center_(1) +
              (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);

      // 如果两个平面的法向量差异或法向量和小于阈值，且距离小于阈值，则合并这两个平面
      if (normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
          normal_add.norm() < config_setting_.plane_merge_normal_thre_)
        if (dis1 < config_setting_.plane_merge_dis_thre_ &&
            dis2 < config_setting_.plane_merge_dis_thre_) {
          if ((*iter)->id_ == 0 && (*iter2)->id_ == 0) { // 如果两个平面都未分配ID
            (*iter)->id_ = current_id; // 分配新的ID
            (*iter2)->id_ = current_id;
            current_id++;
          } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0) // 如果其中一个平面已分配ID
            (*iter)->id_ = (*iter2)->id_; // 将未分配ID的平面设置为相同的ID
          else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0)
            (*iter2)->id_ = (*iter)->id_;
        }
    }
  }

  std::vector<int> merge_flag; // 记录已经合并的平面ID

  // Step 3: 遍历所有平面，合并具有相同ID的平面
  for (size_t i = 0; i < origin_list.size(); i++) {
    auto it =
        std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end()) continue; // 如果当前平面已经合并过，则跳过

    // 如果平面未分配ID，则直接添加到合并列表中
    if (origin_list[i]->id_ == 0) {
      merge_plane_list.push_back(origin_list[i]);
      continue;
    }

    // 创建一个新的合并平面
    std::shared_ptr<Plane> merge_plane(new Plane);
    (*merge_plane) = (*origin_list[i]); // 初始化为当前平面
    bool is_merge = false; // 标记是否进行了合并

    // 遍历所有平面，查找具有相同ID的平面并进行合并
    for (size_t j = 0; j < origin_list.size(); j++) {
      if (i == j) continue; // 跳过自身
      if (origin_list[j]->id_ == origin_list[i]->id_) { // 如果平面ID相同
        is_merge = true; // 标记为已合并

        // 合并两个平面的协方差矩阵和中心点
        Eigen::Matrix3d P_PT1 =
            (merge_plane->covariance_ +
            merge_plane->center_ * merge_plane->center_.transpose()) *
            merge_plane->points_size_;
        Eigen::Matrix3d P_PT2 =
            (origin_list[j]->covariance_ +
            origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
            origin_list[j]->points_size_;
        Eigen::Vector3d merge_center =
            (merge_plane->center_ * merge_plane->points_size_ +
            origin_list[j]->center_ * origin_list[j]->points_size_) /
            (merge_plane->points_size_ + origin_list[j]->points_size_);
        Eigen::Matrix3d merge_covariance =
            (P_PT1 + P_PT2) /
                (merge_plane->points_size_ + origin_list[j]->points_size_) -
            merge_center * merge_center.transpose();

        // 更新合并后的平面信息
        merge_plane->covariance_ = merge_covariance;
        merge_plane->center_ = merge_center;
        merge_plane->points_size_ =
            merge_plane->points_size_ + origin_list[j]->points_size_;
        merge_plane->sub_plane_num_ += origin_list[j]->sub_plane_num_; // 增加子平面数量

        // 计算合并后平面的法向量和半径
        Eigen::EigenSolver<Eigen::Matrix3d> es(merge_plane->covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        merge_plane->normal_ << evecs.real()(0, evalsMin),
            evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        merge_plane->radius_ = sqrt(evalsReal(evalsMax));
        merge_plane->d_ = -(merge_plane->normal_(0) * merge_plane->center_(0) +
                            merge_plane->normal_(1) * merge_plane->center_(1) +
                            merge_plane->normal_(2) * merge_plane->center_(2));

        // 更新平面中心点的坐标和法向量
        merge_plane->p_center_.x = merge_plane->center_(0);
        merge_plane->p_center_.y = merge_plane->center_(1);
        merge_plane->p_center_.z = merge_plane->center_(2);
        merge_plane->p_center_.normal_x = merge_plane->normal_(0);
        merge_plane->p_center_.normal_y = merge_plane->normal_(1);
        merge_plane->p_center_.normal_z = merge_plane->normal_(2);
      }
    }

    // 如果进行了合并，则将合并后的平面添加到merge_plane_list中
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_plane_list.push_back(merge_plane);
    }
  }
}

void BtcDescManager::binary_extractor(
    const std::vector<std::shared_ptr<Plane>> proj_plane_list,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_descriptor_list) {
  binary_descriptor_list.clear();
  std::vector<BinaryDescriptor> temp_binary_list;
  Eigen::Vector3d last_normal(0, 0, 0);
  int useful_proj_num = 0;
  for (int i = 0; i < proj_plane_list.size(); i++) { // 遍历每一个二次合并后的平面
    std::vector<BinaryDescriptor> prepare_binary_list;
    Eigen::Vector3d proj_center = proj_plane_list[i]->center_; // 当前平面的中心
    Eigen::Vector3d proj_normal = proj_plane_list[i]->normal_; // 当前平面的法向量
    if (proj_normal.z() < 0) {
      proj_normal = -proj_normal;
    }
    // 检测当前平面与上一个平面是否相似，相似才执行，否则跳过。
    if ((proj_normal - last_normal).norm() < 0.3 ||
        (proj_normal + last_normal).norm() > 0.3) {
      last_normal = proj_normal;
      if (print_debug_info_) {
        std::cout << "[Description] reference plane normal:"
                  << proj_normal.transpose()
                  << ", center:" << proj_center.transpose() << std::endl;
      }

      useful_proj_num++;
      extract_binary(proj_center, proj_normal, input_cloud,
                     prepare_binary_list);
      for (auto bi : prepare_binary_list) {
        temp_binary_list.push_back(bi);
      }
      if (useful_proj_num == config_setting_.proj_plane_num_) { //只提取足够数量的平面
        break;
      }
    }
  }

  non_maxi_suppression(temp_binary_list);// 对提取的二进制描述符进行非极大值抑制

  // 只需要前 useful_corner_num_ 个二进制描述符
  if (config_setting_.useful_corner_num_ > temp_binary_list.size()) {
    binary_descriptor_list = temp_binary_list;
  } else {
    std::sort(temp_binary_list.begin(), temp_binary_list.end(),
              binary_greater_sort);
    for (size_t i = 0; i < config_setting_.useful_corner_num_; i++) {
      binary_descriptor_list.push_back(temp_binary_list[i]);
    }
  }
  return;
}

/**
 * @brief 
 * @param project_center 
 * @param project_normal 
 * @param input_cloud 
 * @param binary_list 
 */
void BtcDescManager::extract_binary(
  const Eigen::Vector3d &project_center, // 平面的中心
  const Eigen::Vector3d &project_normal, // 平面的法向量
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, // 输入点云
  std::vector<BinaryDescriptor> &binary_list) { // 输出：提取的二进制描述符
  binary_list.clear(); // 清空输出列表

  // 从配置中获取参数
  double binary_min_dis = config_setting_.summary_min_thre_;
  double resolution = config_setting_.proj_image_resolution_;
  double dis_threshold_min = config_setting_.proj_dis_min_;
  double dis_threshold_max = config_setting_.proj_dis_max_;
  double high_inc = config_setting_.proj_image_high_inc_;
  bool line_filter_enable = config_setting_.line_filter_enable_;

  // 计算平面方程参数
  double A = project_normal[0];
  double B = project_normal[1];
  double C = project_normal[2];
  double D = -(A * project_center[0] + B * project_center[1] + C * project_center[2]);

  // 定义投影平面的 x 轴和 y 轴
  Eigen::Vector3d x_axis(1, 0, 0);
  if (C != 0) {
    x_axis[2] = -(A + B) / C;
  } else if (B != 0) {
    x_axis[1] = -A / B;
  } else {
    x_axis[0] = 0;
    x_axis[1] = 1;
  }
  x_axis.normalize();
  Eigen::Vector3d y_axis = project_normal.cross(x_axis);
  y_axis.normalize();

  // 计算 x 轴和 y 轴的方程参数
  double ax = x_axis[0];
  double bx = x_axis[1];
  double cx = x_axis[2];
  double dx = -(ax * project_center[0] + bx * project_center[1] + cx * project_center[2]);

  double ay = y_axis[0];
  double by = y_axis[1];
  double cy = y_axis[2];
  double dy = -(ay * project_center[0] + by * project_center[1] + cy * project_center[2]);

  // 存储投影后的 2D 点和 3D 点
  std::vector<Eigen::Vector2d> point_list_2d;
  pcl::PointCloud<pcl::PointXYZ> point_list_3d;
  std::vector<double> dis_list_2d;

  // 遍历输入点云，进行投影
  for (size_t i = 0; i < input_cloud->size(); i++) {
    double x = input_cloud->points[i].x;
    double y = input_cloud->points[i].y;
    double z = input_cloud->points[i].z;
    double dis = x * A + y * B + z * C + D; // 计算点到平面的距离

    // 过滤距离不在阈值范围内的点
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    } else {
      pcl::PointXYZ pi;
      pi.x = x;
      pi.y = y;
      pi.z = z;
      point_list_3d.points.push_back(pi);
    }

    // 计算点在平面上的投影
    Eigen::Vector3d cur_project;
    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C)) / (A * A + B * B + C * C);
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C)) / (A * A + B * B + C * C);
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B)) / (A * A + B * B + C * C);

    // 计算点在投影平面上的 2D 坐标
    double project_x = cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    double project_y = cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;
    Eigen::Vector2d p_2d(project_x, project_y);

    // 存储投影后的 2D 点和距离
    point_list_2d.push_back(p_2d);
    dis_list_2d.push_back(dis);
  }

  // 如果投影点数量不足，直接返回
  if (point_list_2d.size() <= 5) {
    return;
  }

  // 计算投影点的边界
  double min_x = 10, max_x = -10, min_y = 10, max_y = -10;
  for (auto pi : point_list_2d) {
    if (pi[0] < min_x) min_x = pi[0];
    if (pi[0] > max_x) max_x = pi[0];
    if (pi[1] < min_y) min_y = pi[1];
    if (pi[1] > max_y) max_y = pi[1];
  }

  // 分段处理投影点
  // 1张大平面 = 很多个小局部区域 = 密密麻麻的网格；局部区域自定义，网格尺寸为分辨率。
  int segmen_base_num = 5; // 所研究的局部区域网格尺寸，例如5：以5*5网格作为局部区域
  double segmen_len = segmen_base_num * resolution; // 每个 局部区域 的实际边长
  int x_segment_num = (max_x - min_x) / segmen_len + 1; // x方向可以被分为局部区域的数量
  int y_segment_num = (max_y - min_y) / segmen_len + 1; // y方向可以被分为局部区域的数量
  int x_axis_len = (int)((max_x - min_x) / resolution + segmen_base_num); // x方向总的网格数量
  int y_axis_len = (int)((max_y - min_y) / resolution + segmen_base_num); // y方向总的网格数量

  // 动态分配二维数组
  std::vector<double> **dis_container = new std::vector<double> *[x_axis_len];// 存储网格[][]中的每个点到平面的距离
  BinaryDescriptor **binary_container = new BinaryDescriptor *[x_axis_len];// 网格[][]中对应的binary描述符，具体细分为 该网格高程被占据区间的总数+被占据区间的分布
  double **img_count = new double *[x_axis_len];// 网格[][]中的总共的点数量
  double **dis_array = new double *[x_axis_len];// 存储网格[][]内的高程区间中有多少个区间被占据
  double **mean_x_list = new double *[x_axis_len];// 网格[][]中的x的累加
  double **mean_y_list = new double *[x_axis_len];// 网格[][]中的y的累加

  for (int i = 0; i < x_axis_len; i++) {
    dis_container[i] = new std::vector<double>[y_axis_len];
    binary_container[i] = new BinaryDescriptor[y_axis_len];
    img_count[i] = new double[y_axis_len];
    dis_array[i] = new double[y_axis_len];
    mean_x_list[i] = new double[y_axis_len];
    mean_y_list[i] = new double[y_axis_len];
  }

  // 初始化数组
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      img_count[x][y] = 0;
      mean_x_list[x][y] = 0;
      mean_y_list[x][y] = 0;
      dis_array[x][y] = 0;
      dis_container[x][y].clear();
    }
  }

  // 将投影点分配到对应的网格中
  for (size_t i = 0; i < point_list_2d.size(); i++) {
    int x_index = (int)((point_list_2d[i][0] - min_x) / resolution); // 该2d点所属的网格坐标
    int y_index = (int)((point_list_2d[i][1] - min_y) / resolution);
    mean_x_list[x_index][y_index] += point_list_2d[i][0]; // 网格[][]中的x的累加
    mean_y_list[x_index][y_index] += point_list_2d[i][1]; // 网格[][]中的y的累加
    img_count[x_index][y_index]++; // 网格[][]中的总共的点数量
    dis_container[x_index][y_index].push_back(dis_list_2d[i]); // 存储网格[][]中的每个点到平面的距离
  }

  // 计算每个网格的二进制描述符
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      if (img_count[x][y] > 0) { // 若该网格有点才执行
        int cut_num = (dis_threshold_max - dis_threshold_min) / high_inc; // 该网格在高度上分出的区间数量
        std::vector<bool> occup_list(cut_num, false);
        std::vector<double> cnt_list(cut_num, 0);

        // 统计每个高度区间的点数
        for (size_t j = 0; j < dis_container[x][y].size(); j++) {
          int cnt_index = (dis_container[x][y][j] - dis_threshold_min) / high_inc;
          if (cnt_index >= cut_num) // ljx防止越界
            continue;
          cnt_list[cnt_index]++;
        }

        // 计算描述符的占用数组
        double segmnt_dis = 0;
        for (size_t i = 0; i < cut_num; i++) {
          if (cnt_list[i] >= 1) {
            segmnt_dis++;
            occup_list[i] = true;
          }
        }

        // 存储描述符
        dis_array[x][y] = segmnt_dis;
        BinaryDescriptor single_binary;
        single_binary.occupy_array_ = occup_list;
        single_binary.summary_ = segmnt_dis;
        binary_container[x][y] = single_binary;
      }
    }
  }

  // 过滤并选择描述符
  std::vector<double> max_dis_list; // 存储所有 关键网格 的高程区间占据数量
  std::vector<int> max_dis_x_index_list; // 存储所有 关键网格 的网格x索引
  std::vector<int> max_dis_y_index_list; // 存储所有 关键网格 的网格y索引

  // 遍历每个 局部区域
  for (int x_segment_index = 0; x_segment_index < x_segment_num; x_segment_index++) {
    for (int y_segment_index = 0; y_segment_index < y_segment_num; y_segment_index++) {
      double max_dis = 0;
      int max_dis_x_index = -10, max_dis_y_index = -10;

      // 在当前局部区域中，找到被占据最多的网格
      for (int x_index = x_segment_index * segmen_base_num;
          x_index < (x_segment_index + 1) * segmen_base_num; x_index++) {
        for (int y_index = y_segment_index * segmen_base_num;
            y_index < (y_segment_index + 1) * segmen_base_num; y_index++) {
          if (dis_array[x_index][y_index] > max_dis) {
            max_dis = dis_array[x_index][y_index];
            max_dis_x_index = x_index;
            max_dis_y_index = y_index;
          }
        }
      }

      // 如果最大距离满足阈值，则记录
      if (max_dis >= binary_min_dis) {
        max_dis_list.push_back(max_dis);
        max_dis_x_index_list.push_back(max_dis_x_index);
        max_dis_y_index_list.push_back(max_dis_y_index);
      }
    }
  }

  // 方向列表，用于线过滤
  std::vector<Eigen::Vector2i> direction_list;
  direction_list.push_back(Eigen::Vector2i(0, 1));
  direction_list.push_back(Eigen::Vector2i(1, 0));
  direction_list.push_back(Eigen::Vector2i(1, 1));
  direction_list.push_back(Eigen::Vector2i(1, -1));

  // 过滤并生成最终的二进制描述符
  // 遍历每个 关键网格
  for (size_t i = 0; i < max_dis_list.size(); i++) {
    Eigen::Vector2i p(max_dis_x_index_list[i], max_dis_y_index_list[i]); // 关键网格的网格坐标

    // 检查边界
    if (p[0] <= 0 || p[0] >= x_axis_len - 1 || p[1] <= 0 || p[1] >= y_axis_len - 1) {
      continue;
    }

    bool is_add = true;

    // 线过滤
    if (line_filter_enable) {
      for (int j = 0; j < 4; j++) {
        Eigen::Vector2i p1 = p + direction_list[j]; // p1 p2 为p的上下左右对角一共8个相邻网格
        Eigen::Vector2i p2 = p - direction_list[j];
        double threshold = dis_array[p[0]][p[1]] - 3;

        // 检查相邻网格是否满足条件，若当前网格 金鸡独立、在一众邻居中脱颖而出（if不成立），则表面该网格很好，可以生成描述符
        if (dis_array[p1[0]][p1[1]] >= threshold || dis_array[p2[0]][p2[1]] >= threshold) {
          is_add = false;
          break;
        }
      }
    }

    // 如果满足条件，则生成描述符
    if (is_add) {
      // 计算当前网格映射到物理世界时的2D平均坐标
      double px = mean_x_list[max_dis_x_index_list[i]][max_dis_y_index_list[i]] / img_count[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      double py = mean_y_list[max_dis_x_index_list[i]][max_dis_y_index_list[i]] / img_count[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      Eigen::Vector3d coord = py * x_axis + px * y_axis + project_center; // 反向映射回3D坐标

      // 创建二进制描述符
      BinaryDescriptor single_binary = binary_container[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      single_binary.location_ = coord;
      binary_list.push_back(single_binary);
    }
  }

  // 释放动态分配的内存
  for (int i = 0; i < x_axis_len; i++) {
    delete[] binary_container[i];
    delete[] dis_container[i];
    delete[] img_count[i];
    delete[] dis_array[i];
    delete[] mean_x_list[i];
    delete[] mean_y_list[i];
  }
  delete[] binary_container;
  delete[] dis_container;
  delete[] img_count;
  delete[] dis_array;
  delete[] mean_x_list;
  delete[] mean_y_list;
}

/**
 * @brief 非极大值抑制：在邻域内只保留得分最大（占据高程区间数量最多）的网格的二进制描述符
 * @param binary_list 
 */
void BtcDescManager::non_maxi_suppression(
    std::vector<BinaryDescriptor> &binary_list) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr prepare_key_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  std::vector<int> pre_count_list;
  std::vector<bool> is_add_list;
  for (auto var : binary_list) {
    pcl::PointXYZ pi;
    pi.x = var.location_[0];
    pi.y = var.location_[1];
    pi.z = var.location_[2];
    prepare_key_cloud->push_back(pi);
    pre_count_list.push_back(var.summary_);
    is_add_list.push_back(true);
  }

  kd_tree.setInputCloud(prepare_key_cloud);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  double radius = config_setting_.non_max_suppression_radius_;

  for (size_t i = 0; i < prepare_key_cloud->size(); i++) {
    pcl::PointXYZ searchPoint = prepare_key_cloud->points[i];
    if (kd_tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0) {
      Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        Eigen::Vector3d pj(
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].x,
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].y,
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].z);
        if (pointIdxRadiusSearch[j] == i) {
          continue;
        }
        // 核心
        if (pre_count_list[i] <= pre_count_list[pointIdxRadiusSearch[j]]) {
          is_add_list[i] = false;
        }
      }
    }
  }

  std::vector<BinaryDescriptor> pass_binary_list;
  for (size_t i = 0; i < is_add_list.size(); i++) {
    if (is_add_list[i]) {
      pass_binary_list.push_back(binary_list[i]);
    }
  }

  binary_list.clear();
  for (auto var : pass_binary_list) {
    binary_list.push_back(var);
  }

  return;
}

/**
 * @brief 针对输入的每个二进制描述符，找到它周围的一些同类，构建合法合规合理的三角形进行输出，没有额外数量限制
 * @param binary_list 输入：二进制描述符列表
 * @param frame_id 输入：当前帧的 ID
 * @param btc_list 输出：生成的 BTC 描述符列表
 */
void BtcDescManager::generate_btc(
  const std::vector<BinaryDescriptor> &binary_list,
  const int &frame_id,  
  std::vector<BTC> &btc_list) 
{
  if (binary_list.empty()) // ljx
  {
    std::cout<<"\033[31m[BTC] BinaryDescriptor is empty when we're ready to generate btc!\033[0m"<<std::endl;
    return;
  }

  double scale = 1.0 / config_setting_.std_side_resolution_; // 计算缩放比例
  std::unordered_map<VOXEL_LOC, bool> feat_map; // 用于记录已生成的 BTC 描述符位置
  pcl::PointCloud<pcl::PointXYZ> key_cloud;     // 存储二进制描述符的位置点云

  // Step 1: 将二进制描述符的位置信息转换为点云
  for (auto var : binary_list) {
    pcl::PointXYZ pi;
    pi.x = var.location_[0]; // 设置点的 x 坐标
    pi.y = var.location_[1]; // 设置点的 y 坐标
    pi.z = var.location_[2]; // 设置点的 z 坐标
    key_cloud.push_back(pi); // 将点添加到点云中
  }

  // Step 2: 构建 KD 树用于快速搜索邻近点
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  kd_tree->setInputCloud(key_cloud.makeShared()); // 将 二进制描述符位置 输入到 KD 树中

  int K = config_setting_.descriptor_near_num_; // 获取邻近点数量
  std::vector<int> pointIdxNKNSearch(K);       // 存储搜索到的邻近点索引
  std::vector<float> pointNKNSquaredDistance(K); // 存储搜索到的邻近点距离

  // Step 3: 遍历每个二进制描述符位置，生成 BTC 描述符
  for (size_t i = 0; i < key_cloud.size(); i++) {
    pcl::PointXYZ searchPoint = key_cloud.points[i]; // 当前搜索点

    // 搜索当前点的 K 个邻近点
    if (kd_tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
    {
      // 遍历邻近点组合，生成三角形描述符
      for (int m = 1; m < K - 1; m++) {
        for (int n = m + 1; n < K; n++) {
          pcl::PointXYZ p1 = searchPoint; // 当前点
          pcl::PointXYZ p2 = key_cloud.points[pointIdxNKNSearch[m]]; // 邻近点 1
          pcl::PointXYZ p3 = key_cloud.points[pointIdxNKNSearch[n]]; // 邻近点 2

          // 计算三角形的边长
          double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
          double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) + pow(p1.z - p3.z, 2));
          double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) + pow(p3.z - p2.z, 2));

          // 过滤不符合长度要求的三角形
          if (a > config_setting_.descriptor_max_len_ ||
              b > config_setting_.descriptor_max_len_ ||
              c > config_setting_.descriptor_max_len_ ||
              a < config_setting_.descriptor_min_len_ ||
              b < config_setting_.descriptor_min_len_ ||
              c < config_setting_.descriptor_min_len_) {
            continue;
          }

          // 对边长进行排序，确保 a <= b <= c
          double temp;
          Eigen::Vector3d A, B, C;
          Eigen::Vector3i l1, l2, l3;
          Eigen::Vector3i l_temp;
          l1 << 1, 2, 0; // l1与a绑定，表示最原始的a是p1-p2线段的长度，最终可以追溯到具体的点。
          l2 << 1, 0, 3;
          l3 << 0, 2, 3;

          if (a > b) { // 使得a是原先a b中的小者
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }
          if (b > c) { // 使得b是原先b c中的小者
            temp = b;
            b = c;
            c = temp;
            l_temp = l2;
            l2 = l3;
            l3 = l_temp;
          }
          if (a > b) { // 使得a是原先a b中的小者
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }

          // 过滤退化的三角形（边长接近 a + b）
          if (fabs(c - (a + b)) < 0.2) {
            continue;
          }

          // 将三角形边长映射到体素位置，1000是人为放大的系数
          pcl::PointXYZ d_p;
          d_p.x = a * 1000;
          d_p.y = b * 1000;
          d_p.z = c * 1000;
          VOXEL_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);

          // 检查是否已生成过该位置的 BTC 描述符，若已有，则不再生成，保证三角形形状（与大小）的唯一性。
          auto iter = feat_map.find(position);
          Eigen::Vector3d normal_1, normal_2, normal_3;
          BinaryDescriptor binary_A;
          BinaryDescriptor binary_B;
          BinaryDescriptor binary_C;

          if (iter == feat_map.end()) { // 如果未生成过
            // 根据排序结果确定三角形的顶点，保证A为短中边的顶点，B为短长边的顶点，C为中长边的顶点
            if (l1[0] == l2[0]) {
              A << p1.x, p1.y, p1.z;
              binary_A = binary_list[i];
            } else if (l1[1] == l2[1]) {
              A << p2.x, p2.y, p2.z;
              binary_A = binary_list[pointIdxNKNSearch[m]];
            } else {
              A << p3.x, p3.y, p3.z;
              binary_A = binary_list[pointIdxNKNSearch[n]];
            }

            if (l1[0] == l3[0]) {
              B << p1.x, p1.y, p1.z;
              binary_B = binary_list[i];
            } else if (l1[1] == l3[1]) {
              B << p2.x, p2.y, p2.z;
              binary_B = binary_list[pointIdxNKNSearch[m]];
            } else {
              B << p3.x, p3.y, p3.z;
              binary_B = binary_list[pointIdxNKNSearch[n]];
            }

            if (l2[0] == l3[0]) {
              C << p1.x, p1.y, p1.z;
              binary_C = binary_list[i];
            } else if (l2[1] == l3[1]) {
              C << p2.x, p2.y, p2.z;
              binary_C = binary_list[pointIdxNKNSearch[m]];
            } else {
              C << p3.x, p3.y, p3.z;
              binary_C = binary_list[pointIdxNKNSearch[n]];
            }

            // 创建 BTC 描述符
            BTC single_descriptor;
            single_descriptor.binary_A_ = binary_A;
            single_descriptor.binary_B_ = binary_B;
            single_descriptor.binary_C_ = binary_C;
            single_descriptor.center_ = (A + B + C) / 3; // 计算三角形中心
            single_descriptor.triangle_ << scale * a, scale * b, scale * c; // 缩放后的边长
            single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2)); // 计算角度，ljx认为原作者写了一坨答辩，有待观察后面是否用到angle。
            single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
            single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
            single_descriptor.frame_number_ = frame_id; // 设置帧 ID

            // 记录生成的 BTC 描述符
            feat_map[position] = true;
            btc_list.push_back(single_descriptor);
          }
        }
      }
    }
  }
}

/**
 * @brief 针对本帧的每个BTC，找到他们各自匹配的old帧与该old帧的某个BTC；
 * 针对本帧的每个BTC，找到他们各自匹配的old帧，并对old帧进行投票；
 * 找到票数最多的N个old帧，他们是胜选者；
 * 针对每个胜选帧，找到本帧-该胜选帧的所有BTC匹配情况，并保存到输出中；
 * 完成所有胜选帧的完善，输出所有候选匹配BTCMatchList对象。
 * @param current_STD_list 输入：本帧的所有BTC描述符
 * @param candidate_matcher_vec 输出：候选BTC描述符
 */
void BtcDescManager::candidate_selector(
    const std::vector<BTC> &current_STD_list,
    std::vector<BTCMatchList> &candidate_matcher_vec) {
  // int outlier = 0; // 未使用的变量，可能是预留的
  // double max_dis = 50; // 未使用的变量，可能是预留的
  // int query_num = 0; // 未使用的变量，可能是预留的
  // int pass_num = 0; // 未使用的变量，可能是预留的

  // 生成3D体素网格的偏移量（-1, 0, 1）的组合
  std::vector<Eigen::Vector3i> voxel_round; // 存储3D体素网格的偏移量
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z); // 创建一个3D偏移量
        voxel_round.push_back(voxel_inc); // 将偏移量添加到voxel_round中
      }
    }
  }


  // 初始化一些辅助数据结构
  // 获取当前帧的ID（假设current_STD_list非空）
  int current_frame_id = current_STD_list[0].frame_number_;
  double match_array[20000] = {0}; // 用于记录每个帧的匹配次数，初始化为0
  std::vector<std::pair<BTC, BTC>> match_list; // 存储匹配对
  std::vector<int> match_list_index; // 存储本帧all BTC匹配到的老帧的id们（会重复）
  std::vector<bool> useful_match(current_STD_list.size()); // 标记本帧每个BTC是否有有效匹配
  std::vector<std::vector<size_t>> useful_match_index(current_STD_list.size()); // 存储本帧有效匹配的BTC宽泛化后所匹配到的all过去BTC的帧id
  std::vector<std::vector<BTC_LOC>> useful_match_position(current_STD_list.size()); // 存储本帧有效匹配的BTC宽泛化后的三角形缩放后int边长
  std::vector<size_t> index(current_STD_list.size()); // 用于并行处理的索引

  // 初始化索引和有用匹配标记
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i; // 索引从0到current_STD_list.size()-1
    useful_match[i] = false; // 初始化为false
  }

  std::mutex mylock; // 用于多线程环境下的锁
  auto t0 = std::chrono::high_resolution_clock::now(); // 记录开始时间

  // 并行遍历current_STD_list中的每个描述符
  // 针对本帧的每个BTC，找到他们各自匹配的old帧与该old帧的某个BTC
  std::for_each(
      std::execution::par_unseq, index.begin(), index.end(),
      [&](const size_t &i) {
        BTC descriptor = current_STD_list[i]; // 获取当前描述符
        BTC_LOC position; // 用于存储描述符的边长
        int best_index = 0; // 未使用的变量，可能是预留的
        BTC_LOC best_position; // 未使用的变量，可能是预留的
        // 计算形状相似阈值 = BTC缩放后的边长平方和（动态的，如果三角形很大，则系数很大，保证在三角形相似性对比时我们只考虑形状而非大小） 
        //                  * 人为阈值（源码0.01，干预因子）
        double dis_threshold =
            descriptor.triangle_.norm() *
            config_setting_.rough_dis_threshold_; 

        // 对三角形形状进行细微变形进行特征宽泛处理
        for (auto voxel_inc : voxel_round) {
          position.x = (int)(descriptor.triangle_[0] + voxel_inc[0]); // 计算体素x索引，宽泛三角形的短边长（舍去小数版）
          position.y = (int)(descriptor.triangle_[1] + voxel_inc[1]); // 计算体素y索引，宽泛三角形的中边长（舍去小数版）
          position.z = (int)(descriptor.triangle_[2] + voxel_inc[2]); // 计算体素z索引，宽泛三角形的长边长（舍去小数版）
          Eigen::Vector3d voxel_center((double)position.x + 0.5, // 计算体素中心，即宽泛三角形的短中长边长的大致实际边长（double）
                                       (double)position.y + 0.5,
                                       (double)position.z + 0.5);

          // 第1次检查：当前描述符是否在体素中心附近，即三角形宽泛处理不能与原始三角形过于失真
          if ((descriptor.triangle_ - voxel_center).norm() < 1.5) 
          {
            // 第2次检查：数据库中有没有特征大差不差（因为宽泛化，因此形状粗略相近即可）的三角形
            auto iter = data_base_.find(position);
            if (iter != data_base_.end())  
            {
              // bool is_push_position = false; // 未使用的变量，可能是预留的
              for (size_t j = 0; j < data_base_[position].size(); j++) // 找到了形状粗略类似的三角形，遍历他们
              { 
                // 第3次检查：本BTC应该与找到的BTC有一定的时间差
                if ((descriptor.frame_number_ -
                     data_base_[position][j].frame_number_) >
                    config_setting_.skip_near_num_) 
                {
                  // if ((descriptor.center_-data_base_[position][j].center_).norm() > 100)
                  // 第4次检查：所研究的BTC与找到的BTC在三角形形状上应该足够相似（不再宽泛化，因此形状精确相近）
                  double dis = (descriptor.triangle_ - data_base_[position][j].triangle_).norm(); // 计算边长差距（形状相似度量）
                  if (dis < dis_threshold) 
                  { 
                    // 计算2个BTC的平均二进制描述符的相似度
                    double similarity =
                        (binary_similarity(descriptor.binary_A_,
                                           data_base_[position][j].binary_A_) +
                         binary_similarity(descriptor.binary_B_,
                                           data_base_[position][j].binary_B_) +
                         binary_similarity(descriptor.binary_C_,
                                           data_base_[position][j].binary_C_)) /
                        3;
                    // 第5次检查：所研究的BTC与找到的BTC在二进制特征上应该足够相似
                    // 如果相似度大于阈值，则标记为有用匹配
                    if (similarity > config_setting_.similarity_threshold_) {
                      useful_match[i] = true; // 所研究的BTC，标记为有用匹配
                      useful_match_position[i].push_back(position); // 记录本宽泛化的BTC三角形缩放后的边长
                      useful_match_index[i].push_back(j); // 记录宽泛化的BTC所匹配到的BTC的帧id
                      // 上述3个变量可能存储为：--ljx注
                      // true
                      // △1   △1    △1    △4    △4
                      // 2    3     10    3     5
                    }
                  }
                }
              }
            }
          }
        }
      });

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> index_recorder; // 存储本帧的all BTC的各自的all匹配对
  auto t1 = std::chrono::high_resolution_clock::now(); // 记录结束时间

  // 遍历所有本帧BTC，更新match_array和index_recorder
  for (size_t i = 0; i < useful_match.size(); i++) 
  {
    // 如果该BTC是有效匹配
    if (useful_match[i]) 
    { 
      for (size_t j = 0; j < useful_match_index[i].size(); j++) 
      {
        match_array[data_base_[useful_match_position[i][j]] // 数据库参数1：三角形形状； 数据库参数2：长这个形状的三角形中与当前BTC有效匹配了的那些三角形的帧id
                              [useful_match_index[i][j]] 
                                  .frame_number_] += 1; // 由此找到了数据库中，与当前BTC匹配成功的那个老BTC，查阅其帧id，在这个id的match_array投上一票

        Eigen::Vector2i match_index(i, j); // 创建匹配索引，1st：当前帧的某个BTC的id，2nd：与1st匹配上的过去BTC中的某个的id
        index_recorder.push_back(match_index); // 记录匹配索引
        match_list_index.push_back(
            data_base_[useful_match_position[i][j]][useful_match_index[i][j]]
                .frame_number_); // 记录帧号
      }
    }
  }

  bool multi_thread_en = false; // 是否启用多线程
  if (multi_thread_en) { // 如果启用多线程
    std::for_each(
        std::execution::par_unseq, index.begin(), index.end(),
        [&](const size_t &i) {
          if (useful_match[i]) { // 如果有用匹配
            std::pair<BTC, BTC> single_match_pair; // 创建匹配对
            single_match_pair.first = current_STD_list[i]; // 设置第一个描述符
            for (size_t j = 0; j < useful_match_index[i].size(); j++) {
              single_match_pair.second = data_base_[useful_match_position[i][j]]
                                                   [useful_match_index[i][j]]; // 设置第二个描述符
              mylock.lock(); // 加锁
              match_array[single_match_pair.second.frame_number_] += 1; // 更新匹配次数
              match_list.push_back(single_match_pair); // 添加匹配对
              match_list_index.push_back(
                  single_match_pair.second.frame_number_); // 记录帧号
              mylock.unlock(); // 解锁
            }
          }
        });
  }

  auto t2 = std::chrono::high_resolution_clock::now(); // 记录结束时间

  // 选出最优的N个匹配帧，输出。
  for (int cnt = 0; cnt < config_setting_.candidate_num_; cnt++) 
  {
    double max_vote = 1; // 哪个old帧里的BTC 被 本帧的BTC们 匹配到最多，次数是？
    int max_vote_index = -1; // 哪个old帧里的BTC 被 本帧的BTC们 匹配到最多？
    for (int i = 0; i < 20000; i++) { // 遍历match_array
      if (match_array[i] > max_vote) { // 如果当前帧的匹配次数大于最大投票数
        max_vote = match_array[i]; // 更新最大投票数
        max_vote_index = i; // 更新最大投票数的帧索引
      }
    }

    BTCMatchList match_triangle_list; // 第n个最优匹配
    if (max_vote_index >= 0 && max_vote >= 5) 
    { // 如果得票最多的old帧是有效选举
      match_array[max_vote_index] = 0; // 将该old帧的匹配次数清零，下次循环不再找这个胜选者，选稍微次一点的
      match_triangle_list.match_frame_ = max_vote_index; // 设置该胜选的old帧号
      match_triangle_list.match_id_.first = current_frame_id; // 设置当前帧号
      match_triangle_list.match_id_.second = max_vote_index; // 设置该胜选的old帧号

      // double mean_dis = 0; // 未使用的变量，可能是预留的
      // 找到 本帧-胜选帧 之间的所有BTC匹配对
      for (size_t i = 0; i < index_recorder.size(); i++) { // 遍历index_recorder
        if (match_list_index[i] == max_vote_index) { // 如果这个匹配对 所匹配到的帧 刚好就是 胜选的old帧
          std::pair<BTC, BTC> single_match_pair; // 存储all 本帧-胜选帧 的 BTC匹配对，1st-2nd：本帧与胜选帧的BTC匹配对。
          single_match_pair.first = current_STD_list[index_recorder[i][0]];
          single_match_pair.second =
              data_base_[useful_match_position[index_recorder[i][0]]
                                              [index_recorder[i][1]]]
                        [useful_match_index[index_recorder[i][0]]
                                           [index_recorder[i][1]]];
          match_triangle_list.match_list_.push_back(single_match_pair); // 添加匹配对
        }
      }
      candidate_matcher_vec.push_back(match_triangle_list); // 将第n个最优匹配保存到输出结果，保存足够数量就不再保存。
    }
  }
}

/**
 * @brief 针对输入的本帧与第n个候选帧，通过坐标变换尝试，找到2帧间最优秀的BTC匹配对。
 * 依据该最优，可以得到相对位姿、较优秀（其中包括最优）的2帧间BTC匹配对们、几何验证得分。
 * @param candidate_matcher 输入：第n个候选帧的匹配对象
 * @param verify_score 输出：验证得分
 * @param relative_pose 输出：相对位姿（平移和旋转）
 * @param sucess_match_list 输出：依据 最优2帧间BTC匹配对的对齐结果，可以较为准确对齐的那些 2帧间BTC匹配对们
 */
void BtcDescManager::candidate_verify(
  const BTCMatchList &candidate_matcher, 
  double &verify_score,
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
  std::vector<std::pair<BTC, BTC>> &sucess_match_list)
{
  sucess_match_list.clear(); // 依据 最优2帧间BTC匹配对的对齐结果，可以较为准确对齐的那些 2帧间BTC匹配们
  double dis_threshold = 2;  // 距离阈值，用于判断匹配是否成功

  // Step 1: 对候选匹配列表进行采样，减少计算量
  int skip_len = (int)(candidate_matcher.match_list_.size() / 50) + 1; // 采样间隔，与2帧之间的BTC匹配总量有关
  int use_size = candidate_matcher.match_list_.size() / skip_len; // 采样后的匹配对数量
  std::vector<size_t> index(use_size); // 存储采样后的索引
  std::vector<int> vote_list(use_size); // 存储 依据第i个BTC匹配对给出的旋转平移估计 在全体BTC匹配对中 成功的次数

  // 初始化索引
  for (size_t i = 0; i < index.size(); i++) {
    index[i] = i;
  }

  std::mutex mylock; // 用于多线程环境下的锁
  auto t0 = std::chrono::high_resolution_clock::now(); // 记录开始时间

  // Step 2: 并行遍历采样后的匹配对，计算每个匹配对的投票数
  std::for_each(
      std::execution::par_unseq, index.begin(), index.end(),
      [&](const size_t &i) {
        auto single_pair = candidate_matcher.match_list_[i * skip_len]; // 获取所研究的 2帧间的BTC匹配对
        int vote = 0; // 当前匹配对的投票数
        Eigen::Matrix3d test_rot; // 测试旋转矩阵
        Eigen::Vector3d test_t;   // 测试平移向量

        // 求解当前匹配对的相对位姿
        triangle_solver(single_pair, test_t, test_rot);

        // 遍历所有匹配对，验证当前位姿的匹配效果
        for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) 
        {
          auto verify_pair = candidate_matcher.match_list_[j]; // 获取每一个2帧间的BTC匹配对
          Eigen::Vector3d A = verify_pair.first.binary_A_.location_;
          Eigen::Vector3d A_transform = test_rot * A + test_t; // 变换后的点 A
          Eigen::Vector3d B = verify_pair.first.binary_B_.location_;
          Eigen::Vector3d B_transform = test_rot * B + test_t; // 变换后的点 B
          Eigen::Vector3d C = verify_pair.first.binary_C_.location_;
          Eigen::Vector3d C_transform = test_rot * C + test_t; // 变换后的点 C

          // 计算变换后的点与目标点的距离
          double dis_A = (A_transform - verify_pair.second.binary_A_.location_).norm();
          double dis_B = (B_transform - verify_pair.second.binary_B_.location_).norm();
          double dis_C = (C_transform - verify_pair.second.binary_C_.location_).norm();

          // 如果距离小于阈值，则投票数加 1
          if (dis_A < dis_threshold && dis_B < dis_threshold && dis_C < dis_threshold) {
            vote++;
          }
        }

        // 加锁，更新投票数列表
        mylock.lock();
        vote_list[i] = vote;
        mylock.unlock();
      });

  // Step 3: 找到投票数最多的匹配对
  int max_vote_index = 0; // 最优秀的 2帧间BTC匹配对在候选对象中的[采样]索引
  int max_vote = 0;       // 最优秀的 2帧间BTC匹配对累积的成功次数
  for (size_t i = 0; i < vote_list.size(); i++) {
    if (max_vote < vote_list[i]) {
      max_vote_index = i;
      max_vote = vote_list[i];
    }
  }

  // Step 4: 如果最大投票数满足阈值，则计算相对位姿并生成成功匹配对列表
  if (max_vote >= 4) { // 阈值设为 4
    auto best_pair = candidate_matcher.match_list_[max_vote_index * skip_len]; // 最佳 2帧间BTC匹配对
    Eigen::Matrix3d best_rot; // 最佳旋转矩阵
    Eigen::Vector3d best_t;   // 最佳平移向量

    // 求解最佳匹配对的相对位姿
    triangle_solver(best_pair, best_t, best_rot);
    relative_pose.first = best_t;  // 设置输出平移向量
    relative_pose.second = best_rot; // 设置输出旋转矩阵

    // 遍历所有匹配对，生成成功匹配对列表
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) 
    {
      auto verify_pair = candidate_matcher.match_list_[j]; // 获取验证匹配对
      Eigen::Vector3d A = verify_pair.first.binary_A_.location_;
      Eigen::Vector3d A_transform = best_rot * A + best_t; // 变换后的点 A
      Eigen::Vector3d B = verify_pair.first.binary_B_.location_;
      Eigen::Vector3d B_transform = best_rot * B + best_t; // 变换后的点 B
      Eigen::Vector3d C = verify_pair.first.binary_C_.location_;
      Eigen::Vector3d C_transform = best_rot * C + best_t; // 变换后的点 C

      // 计算变换后的点与目标点的距离
      double dis_A = (A_transform - verify_pair.second.binary_A_.location_).norm();
      double dis_B = (B_transform - verify_pair.second.binary_B_.location_).norm();
      double dis_C = (C_transform - verify_pair.second.binary_C_.location_).norm();

      // 如果距离小于阈值，则添加到成功匹配对列表
      if (dis_A < dis_threshold && dis_B < dis_threshold && dis_C < dis_threshold) {
        sucess_match_list.push_back(verify_pair);
      }
    }

    // Step 5: 计算几何验证得分
    verify_score = plane_geometric_verify(
        plane_cloud_vec_.back(), // 当前帧的平面们
        plane_cloud_vec_[candidate_matcher.match_id_.second], // 所研究的第n个胜选帧的平面们
        relative_pose); // 相对位姿
  } else {
    verify_score = -1; // 如果最大投票数不满足阈值，则验证得分为 -1
  }

  return;
}

/**
 * @brief 得到2个输入三角形的旋转平移变换（由第一个三角形指向第二个）
 * @param std_pair 
 * @param t 
 * @param rot 
 */
void BtcDescManager::triangle_solver(std::pair<BTC, BTC> &std_pair,
                                     Eigen::Vector3d &t, Eigen::Matrix3d &rot) {
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero(); // 第一个BTC三角形的顶点中心向量
  Eigen::Matrix3d ref = Eigen::Matrix3d::Zero(); // 第二个BTC三角形的顶点中心向量
  src.col(0) = std_pair.first.binary_A_.location_ - std_pair.first.center_;
  src.col(1) = std_pair.first.binary_B_.location_ - std_pair.first.center_;
  src.col(2) = std_pair.first.binary_C_.location_ - std_pair.first.center_;
  ref.col(0) = std_pair.second.binary_A_.location_ - std_pair.second.center_;
  ref.col(1) = std_pair.second.binary_B_.location_ - std_pair.second.center_;
  ref.col(2) = std_pair.second.binary_C_.location_ - std_pair.second.center_;
  Eigen::Matrix3d covariance = src * ref.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();
  rot = V * U.transpose();
  if (rot.determinant() < 0) {
    Eigen::Matrix3d K;
    K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    rot = V * K * U.transpose();
  }
  t = -rot * std_pair.first.center_ + std_pair.second.center_;
}

/**
 * @brief 针对每一个src平面，利用预估相对变换将其对齐在tar帧的坐标系下，
 * 查看该变换后src平面 是否与 其最近的tar平面 重合。
 * 如果近似重合，则加分。
 * @param source_cloud 输入：src帧的平面们
 * @param target_cloud 输入：tar帧的平面们
 * @param transform 输入：预估的2帧间相对变换
 * @return double，src中重合平面/src全部平面
 */
double BtcDescManager::plane_geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) 
{
  Eigen::Vector3d t = transform.first;
  Eigen::Matrix3d rot = transform.second;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }

  kd_tree->setInputCloud(input_cloud); // tar的平面们的位置作为input

  // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  double useful_match = 0;
  double normal_threshold = config_setting_.normal_threshold_;
  double dis_threshold = config_setting_.dis_threshold_;
  
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i]; // src当前的平面
    pcl::PointXYZ use_search_point; // src当前的平面进行变换
    use_search_point.x = searchPoint.x;
    use_search_point.y = searchPoint.y;
    use_search_point.z = searchPoint.z;
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;// src当前的平面的位置进行变换
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni; // src当前的平面的法向量进行变换

    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint = target_cloud->points[pointIdxNKNSearch[0]];// 在tar中找到了src当前平面变换后 位置最近 的平面
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z); // 当前src最近的tar平面位置
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y, nearstPoint.normal_z); // 当前src最近的tar平面法向量
      Eigen::Vector3d normal_inc = ni - tni; // 坐标系对齐后，当前src与其最近tar 平面的 法向量的差
      Eigen::Vector3d normal_add = ni + tni; // 坐标系对齐后，当前src与其最近tar 平面的 法向量的和
      
      double point_to_plane = fabs(tni.transpose() * (pi - tpi)); // 1个平面位置到另外一个平面的距离（点到面距离）

      // 如果当前src、其最近tar 坐标系对齐后 法向量相似、且距离很小
      if ((normal_inc.norm() < normal_threshold ||
           normal_add.norm() < normal_threshold) &&
          point_to_plane < dis_threshold) 
      {
        useful_match++; // 得分++
      }
    }
  }
  return useful_match / source_cloud->size();
}
