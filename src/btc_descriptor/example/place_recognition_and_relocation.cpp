/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-17 17:13:44
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-25 12:19:30
 * @FilePath: /OnlineLTSlam/src/btc_descriptor/example/place_recognition_and_relocation.cpp
 * @Description:
 * @Logs:
 *      1.2025-02-11：对BTC提取采用了sub-map方法，提高了BTC场景识别-粗匹配的检出率；
 *      2.2025-02-12：对BTC内的粗回环检测部分进行优化改进，加入距离阈值的判断，降低BTC场景识别-粗匹配的误判率；
 *      3.2025-02-13：增加平面可视化功能，提供灵活的API，便于观测；新增PlaceRecognitionList结构体，作为粗匹配+场景识别的最终结果保存格式；
 *      4.2025-02-14：完成精匹配API接口贯通，现已打通精匹配流程；
 *      5.Tips：生成BTC的是odom下的实时点云，因此BTC、平面特征均是odom坐标系下；BTC粗匹配的结果loop_transform是curr近似的一个定位误差；
 *      6.2025-02-17：精匹配已经得到运行，将精度从4m控制到1m（旋转角控制在0.4°）；
 *      7.2025-02-19：精匹配已得到验证：输出的位姿纯算法优化未必真值，但基于优化结果对同场景点云的拼接却是极度重合准确的。
 *      8.单帧完整BTC耗时：30ms；10帧拼接完整BTC耗时：150ms。拼接与否，点云拼接重合效果都很好。
 *      9.2025-02-20：新增NDT匹配模块，完成所有API。
 *      10.2025-2-21：修复了NDT模块的一些已知问题，增加可视化与测试打印。
 *      11.2025-2-25：完善了BTC-NDT效果对比的输出格式，即采用统一curr单帧(基于ori与opti-*生成的全局点云)与其loop(未必同帧，采用相同的submap构建方式)进行对比；也进行了耗时对比。
 */

#include "example/place_recognition_and_relocation.h"

std::unique_ptr<BtcDescManager> btc_manager;

const int keyFrameIdTestPrint = 2113; // 010109：2566  010113：3372 

void signalHandler(int signal)
{
    isRunning = false;
}

double radianToDegreeClamped(double radian) 
{
    // 将弧度转换为度数，并限制在 -180° 到 +180° 的范围内
    // 将弧度转换为角度
    double degrees = radian * (180.0 / M_PI);
    
    // 将角度规范到[-180, 180]范围
    degrees = std::fmod(degrees, 360.0);
    
    if (degrees > 180.0) {
        degrees -= 360.0;
    } else if (degrees < -180.0) {
        degrees += 360.0;
    }

    // 如果角度接近 -180° 或 180°，调整到接近 0°
    if (std::abs(degrees + 180.0) < 1e-6) {
        degrees = 0.0;
    } else if (std::abs(degrees - 180.0) < 1e-6) {
        degrees = 0.0;
    }
    
    return degrees;
}

Eigen::Vector3d turnRadianVec3dToDegreeVec3d(const Eigen::Vector3d& radVec )
{
    Eigen::Vector3d degVec;
    for (size_t i=0; i<3; ++i)
        degVec[i] = radianToDegreeClamped(radVec[i]);
    return degVec;
}

void turnPairPose2EigenMatrix4d(const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& poseIn, Eigen::Matrix4d& poseOut)
{
    poseOut = Eigen::Matrix4d::Identity();
    poseOut.block<3,3>(0,0) = poseIn.second;
    poseOut.block<3,1>(0,3) = poseIn.first;
}

void initParam(ros::NodeHandle &nh)
{
    nh.param<double>("cloud_overlap_thr", cloud_overlap_thr, 0.5);
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("pcds_dir", pcds_dir, "");
    nh.param<std::string>("pose_file", pose_file, "");
    nh.param<bool>("read_bin", read_bin, true);

    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubCureentCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    pubCurrentBinary = nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    // pubPath = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    // pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/current_pose", 10);
    // pubMatchedPose = nh.advertise<nav_msgs::Odometry>("/matched_pose", 10);
    pubMatchedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    pubMatchedBinary = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    pubLoopStatus = nh.advertise<visualization_msgs::MarkerArray>("/loop_status", 100); // 输入位姿的连线
    pubBTC = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);      // 三角形们  std_msgs::ColorRGBA color_tp;
    pubCurrentPlane = nh.advertise<visualization_msgs::MarkerArray>("/current_plane", 3);      // 发布当前帧的平面

    // 描绘回环状态（体现在路径上）的参数
    scale_tp = 4.0; // BTC粗匹配成功、场景识别正确
    color_tp.a = 1.0;
    color_tp.r = 0.0 / 255.0;
    color_tp.g = 255.0 / 255.0;
    color_tp.b = 0.0 / 255.0;
    scale_fp = 5.0; // BTC粗匹配成功、场景识别错误
    color_fp.a = 1.0;
    color_fp.r = 1.0;
    color_fp.g = 0.0;
    color_fp.b = 0.0;
    scale_path = 3.0; // BTC粗匹配失败
    color_path.a = 0.8;
    color_path.r = 255.0 / 255.0;
    color_path.g = 255.0 / 255.0;
    color_path.b = 255.0 / 255.0;

    inilerFilter.setRadiusSearch(0.5);                  // 设置搜索半径
    inilerFilter.setMinNeighborsInRadius(3);            // 设置一个内点最少的邻居数目 
    downSampleFilter.setLeafSize(0.2, 0.2, 0.2);

    load_config_setting(setting_path, config_setting);

    btc_manager = std::make_unique<BtcDescManager>(config_setting);
    btc_manager->print_debug_info_ = false;

    isRunning = true;

}

void transPclPointCloud(pcl::PointCloud<PointType>::Ptr& inout_cloud, 
    const Eigen::Vector3d translation, const Eigen::Matrix3d rotation)
{
    // 将旋转矩阵和平移向量转换为单精度以加速计算
    const Eigen::Matrix3f R = rotation.cast<float>();
    const Eigen::Vector3f t = translation.cast<float>();
    const size_t num_points = inout_cloud->size();
    
    // 使用OpenMP并行处理点云
    #pragma omp parallel for
    for (size_t j = 0; j < num_points; ++j)
    {
        auto& p = inout_cloud->points[j];
        const Eigen::Vector3f pv = R * Eigen::Vector3f(p.x, p.y, p.z) + t;// 内联坐标转换，直接进行矩阵运算
        p.x = pv[0];
        p.y = pv[1];
        p.z = pv[2];
    }
}
void transPclPointCloud(pcl::PointCloud<PointType>::Ptr& in_cloud, 
    const Eigen::Vector3d translation, const Eigen::Matrix3d rotation,
    pcl::PointCloud<PointType>::Ptr& out_cloud)
{
    // 将旋转矩阵和平移向量转换为单精度以加速计算
    const Eigen::Matrix3f R = rotation.cast<float>();
    const Eigen::Vector3f t = translation.cast<float>();
    const size_t num_points = in_cloud->size();
    out_cloud->resize(num_points);
    
    // 使用OpenMP并行处理点云
    #pragma omp parallel for
    for (size_t j = 0; j < num_points; ++j)
    {
        auto& pi = in_cloud->points[j];
        auto& po = out_cloud->points[j];
        const Eigen::Vector3f pv = R * Eigen::Vector3f(pi.x, pi.y, pi.z) + t;// 内联坐标转换，直接进行矩阵运算
        po.x = pv[0];
        po.y = pv[1];
        po.z = pv[2];
    }
}


void loadPoses()
{
    // pose is only for visulization and gt overlap calculation
    load_evo_pose_with_time(pose_file, pose_list, time_list); // pose格式如bxn
    std::string print_msg = "Successfully load pose file:" + pose_file + ". pose size:" + std::to_string(time_list.size());
    ROS_INFO_STREAM(print_msg.c_str());
}

bool loadPointcloudBinAndTrans(pcl::PointCloud<PointType>::Ptr& cloud, 
    size_t bin_id)
{
    std::stringstream ss;
    // Get pose information, only for gt overlap calculation
    Eigen::Vector3d translation = pose_list[bin_id].first;
    Eigen::Matrix3d rotation = pose_list[bin_id].second;
    ss << pcds_dir << "/" << std::setfill('0') << std::setw(6) << bin_id << ".bin";
    std::string pcd_file = ss.str();
    auto t_load_start = std::chrono::high_resolution_clock::now();
    std::vector<float> lidar_data = read_lidar_data(ss.str());
    if (lidar_data.size() == 0)
    {
        return false;
    }

    for (std::size_t i = 0; i < lidar_data.size(); i += 4)
    {
        PointType point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        point.intensity = lidar_data[i + 3];
        cloud->points.push_back(point);
    }
    auto t_load_end = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] load cloud for bin: " << time_inc(t_load_end, t_load_start)
                << "ms, " << std::endl;
    return true;
}

bool loadPointcloudPcdAndTrans(pcl::PointCloud<PointType>::Ptr& cloud, 
    size_t pcd_id)
{
    // Load point cloud from pcd file
    std::stringstream ss;
    // Get pose information, only for gt overlap calculation
    double timestamp = time_list[pcd_id];

    // ss << pcds_dir << "/" << std::setfill('0') << std::setw(6) << timestamp << ".pcd";
    ss << pcds_dir << "/" << std::fixed << std::setprecision(6) << timestamp << ".pcd";
    std::string pcd_file = ss.str();

    auto t_load_start = std::chrono::high_resolution_clock::now();
    // pcl::io::loadPCDFile<PointType>(pcd_file, *cloud)
    if (reader.read(pcd_file, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Couldn't read file " << pcd_file);
        return false;
    }
    auto t_load_end = std::chrono::high_resolution_clock::now();
    // std::cout << "[Time] load cloud from pcd: " << time_inc(t_load_end, t_load_start)
    //             << "ms, " << std::endl;

    return true;
}

void loadAllPointCloud(std::vector<pcl::PointCloud<PointType>::Ptr>& ori_clouds, bool doDownSample = false)
{
    std::mutex map_mutex;
    std::for_each(std::execution::par, 
                ori_clouds.begin(), 
                ori_clouds.end(),
                [&](auto& cloud_ptr) 
    {
        size_t submap_id = &cloud_ptr - &ori_clouds[0];
        
        pcl::PointCloud<PointType>::Ptr local_cloud(new pcl::PointCloud<PointType>());

        if (read_bin) {
            if (!loadPointcloudBinAndTrans(local_cloud, submap_id)) 
                return;
        } else {
            if (!loadPointcloudPcdAndTrans(local_cloud, submap_id)) 
                return;
        }
    
        if (doDownSample)
        {
            downSampleFilter.setInputCloud(local_cloud);
            downSampleFilter.filter(*local_cloud);
        }

        std::lock_guard<std::mutex> lock(map_mutex);
        ori_clouds[submap_id] = local_cloud;
    });
}

void pointCloudPreprocess(const pcl::PointCloud<PointType>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut)
{
    if (!cloudIn) {
        PCL_ERROR("Input point cloud is empty!\n");
        return;
    }

    pcl::PointCloud<PointType>::Ptr cloud = cloudIn->makeShared();    

    // 滤除离散点 耗时约50ms
    // inilerFilter.setInputCloud(cloud);
    // inilerFilter.filter(*cloud);  
    
    // TODO

    *cloudOut = *cloud;
    cloud.reset();
}

void savePlaceRecognitionList(const int curr_id, const int best_loop_id, 
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform,
    const std::vector<std::pair<BTC, BTC>>& loop_std_pair)
{
    std::shared_ptr<PlaceRecognitionList> this_place_recog_ptr = std::make_shared<PlaceRecognitionList>(); // 别忘了初始化

    this_place_recog_ptr->match_id_ = std::pair<int,int>(curr_id, best_loop_id);
    this_place_recog_ptr->loop_transform_ = loop_transform;
    // this_place_recog_ptr->match_list_ = loop_std_pair;
    this_place_recog_ptr->match_list_ = std::move(loop_std_pair); // 避免深拷贝降低开销，但是loop_std_pair后续最好不再用。
    btc_manager->nice_place_recognition_vec.push_back(this_place_recog_ptr);
}

void savePosesToTxt(const std::string& filename,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& pose1,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& pose2,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& pose3)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // 第一行：解释性字符串
    outFile << "loop's input_pose, curr's input_pose, curr's opti_pose_by_btc/ndt. All the poses are lidar2odom." << std::endl;

    Eigen::Matrix4d poseMat1;
    Eigen::Matrix4d poseMat2;
    Eigen::Matrix4d poseMat3;
    turnPairPose2EigenMatrix4d(pose1, poseMat1);
    turnPairPose2EigenMatrix4d(pose2, poseMat2);
    turnPairPose2EigenMatrix4d(pose3, poseMat3);

    // 第二行：第一个矩阵
    outFile << poseMat1.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ")) << std::endl;

    // 第三行：第二个矩阵
    outFile << poseMat2.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ")) << std::endl;

    // 第四行：第三个矩阵
    outFile << poseMat3.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ")) << std::endl;

    outFile.close();
    std::cout << "Matrices saved to " << filename << std::endl;
}

/**
 * @brief pose1 到 pose2 的旋转平移变换（差异），或者在pose1坐标系下，pose2的坐标
 * @param pose1 
 * @param pose2 
 * @return PosesDiff 
 */
PosesDiff calculateDiffBtw2Poses(const std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose1, 
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose2)
{
    PosesDiff this_pd;
    this_pd.t_diff_xyz = pose2.first - pose1.first;
    this_pd.t_diff_value = this_pd.t_diff_xyz.norm();
    
    this_pd.rot_diff_mat = pose1.second.inverse() * pose2.second;
    this_pd.rot_diff_rpy_deg = this_pd.rot_diff_mat.eulerAngles(0, 1, 2);
    for (size_t i=0; i<3; ++i)
        this_pd.rot_diff_rpy_deg[i] = radianToDegreeClamped(this_pd.rot_diff_rpy_deg[i]);
    Eigen::AngleAxisd angle_axis(this_pd.rot_diff_mat);
    this_pd.rot_diff_value_deg = angle_axis.angle();
    this_pd.rot_diff_value_deg = radianToDegreeClamped(this_pd.rot_diff_value_deg);

    this_pd.pose_diff.first = this_pd.t_diff_xyz;
    this_pd.pose_diff.second = this_pd.rot_diff_mat;

    return this_pd;
}

void reLocation()
{
    T.start(3);
    if (btc_manager->nice_place_recognition_vec.empty()) {
        ROS_ERROR("nice_place_recognition_vec is empty");
        return;
    }
    
    // 本次精匹配所涉及的粗匹配数据输入
    std::shared_ptr<PlaceRecognitionList> this_nice_place_recognition = btc_manager->nice_place_recognition_vec.back(); 
    if (!this_nice_place_recognition) {
        ROS_ERROR("this_nice_place_recognition is null");
        return;
    }

    // 初始化 本帧定位误差(包括精估计与粗估计)
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform = this_nice_place_recognition->loop_transform_; // 粗匹配的本帧定位误差
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> opti_transform; // 精匹配的本帧定位误差
    opti_transform.first = Eigen::Vector3d(0.0, 0.0, 0.0);
    opti_transform.second = Eigen::Matrix3d::Identity();
    Eigen::Vector3d loop_rot = turnRadianVec3dToDegreeVec3d(loop_transform.second.eulerAngles(0, 1, 2));
    // loop_transform = opti_transform;

    const std::string GREEN_COLOR = "\033[32m";
    const std::string RESET_COLOR = "\033[0m";
    std::cout <<GREEN_COLOR<< "[BTC][PIcp] t of cupipei: " << loop_transform.first.transpose() <<RESET_COLOR<< std::endl;
    std::cout << "[BTC][PIcp] rot of cupipei: " << loop_rot.transpose() << std::endl;

    // 使用原始平面构建残差，约束更多
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> curr_planes_points(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> loop_planes_points(new pcl::PointCloud<pcl::PointXYZINormal>);
    curr_planes_points = btc_manager->plane_cloud_vec_[this_nice_place_recognition->match_id_.first]; 
    loop_planes_points = btc_manager->plane_cloud_vec_[this_nice_place_recognition->match_id_.second];

    // 执行精匹配
    bool isPlaneIcpSuccess = false;
    isPlaneIcpSuccess = btc_manager->PlaneGeomrtricIcp(curr_planes_points, loop_planes_points, loop_transform, opti_transform, true);
    Eigen::Vector3d opti_rot = turnRadianVec3dToDegreeVec3d(opti_transform.second.eulerAngles(0, 1, 2));

    PosesDiff loop_opti_pd = calculateDiffBtw2Poses(loop_transform,opti_transform); // [位姿之差的差]基于BTC与平面优化：相较于BTC粗匹配给出的[帧间粗估计位姿]，平面优化精匹配给出的[帧间精估计位姿]变化了多少
    
    // 本帧原始的定位结果
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> ori_curr_pose = pose_list[this_nice_place_recognition->match_id_.first];
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> ori_loop_pose = pose_list[this_nice_place_recognition->match_id_.second];
    PosesDiff ori_pd = calculateDiffBtw2Poses(ori_loop_pose,ori_curr_pose); // [位姿之差]基于输入的原始位姿：相较于过去loop帧，当前帧变化了多少（帧间原始位姿）
    Eigen::Vector3d bef_curr_rot = turnRadianVec3dToDegreeVec3d(ori_curr_pose.second.eulerAngles(0, 1, 2));

    // 本帧最终优化后的定位结果
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> aft_curr_pose;
    aft_curr_pose.first = ori_curr_pose.second * opti_transform.first + ori_curr_pose.first;
    aft_curr_pose.second = opti_transform.second * ori_curr_pose.second;
    Eigen::Vector3d aft_curr_rot = turnRadianVec3dToDegreeVec3d(aft_curr_pose.second.eulerAngles(0, 1, 2));

    // 计算本回环成功帧的总计时间
    T.print(3, "[BTC] Timer cost of accurate btc: ");

    // for debug
    // static int number2save = 1;
    // static int nowNumber = 0;
    // if (number2save == nowNumber++)
    if(this_nice_place_recognition->match_id_.first == keyFrameIdTestPrint)
    {
        pcl::PointCloud<PointType>::Ptr curr_points_odom(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr loop_points_odom(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr curr_points_opti_by_p2picp(new pcl::PointCloud<PointType>);

        curr_points_odom = btc_manager->key_ori_cloud_vec_[this_nice_place_recognition->match_id_.first];
        // loop_points_odom = btc_manager->key_ori_cloud_vec_[this_nice_place_recognition->match_id_.second];
        
        // 为了测试
        int rsLoopID = this_nice_place_recognition->match_id_.second;
        static int halfNumOfNear2MergeLoopSubmap = 4;
        for (int j=std::max(0,int(rsLoopID)-halfNumOfNear2MergeLoopSubmap); j<=std::min(int(pose_list.size()),int(rsLoopID)+halfNumOfNear2MergeLoopSubmap); j++)
        {
            *loop_points_odom += *btc_manager->key_ori_cloud_vec_[j];
        }

        Eigen::Matrix4d opti_transform_mat = Eigen::Matrix4d::Identity();
        opti_transform_mat.block<3,3>(0,0) = opti_transform.second;
        opti_transform_mat.block<3,1>(0,3) = opti_transform.first;
        pcl::transformPointCloud(*curr_points_odom, *curr_points_opti_by_p2picp, opti_transform_mat.cast<float>());

        pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/curr_points_odom.pcd", *curr_points_odom);
        pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/curr_points_opti.pcd", *curr_points_opti_by_p2picp);
        pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/loop_points_odom.pcd", *loop_points_odom);
        pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/curr-planes.pcd", *curr_planes_points);
        pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523/loop-planesloop_planes_points.pcd", *loop_planes_points);

        savePosesToTxt("/home/jixuanlee/pcdsOF523/btc-poses-for-compare.txt", ori_loop_pose, ori_curr_pose, aft_curr_pose);
    }
        

    // 打印
    if (isPlaneIcpSuccess)
    {
        // Error：本帧定位误差
        // Diff：精优化前后，本帧定位误差的变化
        // Trans：根据输入pose，2帧间定位的变换量（含误差）
        // Position：精优化前后，本帧定位数据
        // Change：精优化前后，本帧定位数据被优化了多少

        std::cout <<GREEN_COLOR<< "[BTC][PIcp] Error of curr aft: Translation: " << opti_transform.first.transpose() <<RESET_COLOR<< std::endl;
        std::cout << "[BTC][PIcp] Error of curr aft: Rotation: " << opti_rot.transpose() <<" in RPY deg."<< std::endl;
        std::cout << "[BTC][PIcp] Diff btw the error of bef & aft: Translation: " << loop_opti_pd.t_diff_xyz.transpose() << std::endl;
        std::cout << "[BTC][PIcp] Diff btw the error of bef & aft: Translation magnitude: [" << loop_opti_pd.t_diff_value <<"] m."<< std::endl;
        std::cout << "[BTC][PIcp] Diff btw the error of bef & aft: Rotation: [" << loop_opti_pd.rot_diff_value_deg<< "] deg." << std::endl;
        std::cout << "[BTC][PIcp] Trans btw curr&loop bef: Translation: " << ori_pd.pose_diff.first.transpose() << std::endl;
        std::cout << "[BTC][PIcp] Trans btw curr&loop bef: Rotation" << ori_pd.rot_diff_rpy_deg.transpose() <<" in RPY deg."<< std::endl;
        std::cout <<GREEN_COLOR<< "[BTC][PIcp] Position of curr bef: Translation: " << ori_curr_pose.first.transpose() <<RESET_COLOR<< std::endl;
        std::cout << "[BTC][PIcp] Position of curr bef: Rotation: " << bef_curr_rot.transpose() << " in RPY deg."<< std::endl;
        std::cout <<GREEN_COLOR<< "[BTC][PIcp] Position of curr aft: Translation: " << aft_curr_pose.first.transpose() <<RESET_COLOR<< std::endl;
        std::cout << "[BTC][PIcp] Position of curr aft: Rotation: " << aft_curr_rot.transpose() << " in RPY deg."<< std::endl;
        std::cout << "[BTC][PIcp] Change of curr position: Translation: [" << (ori_curr_pose.first - aft_curr_pose.first).norm() << "] m."<< std::endl;
        std::cout<<std::endl;
    }


}

void placeRecognition()
{
    int triggle_loop_num = 0; // 仅靠BTC的粗匹配回环成功次数（BTC触发）
    int true_loop_num = 0; // 仅靠BTC的粗匹配回环成功次数（BTC触发&&点云足够重合）

    ros::Rate loop(50000);
    ros::Rate slow_loop(1000);

    std::deque<pcl::PointCloud<PointType>::Ptr> submap_trans_cloud; // 存储临近的odom的点云
    
    // 执行核心的场景识别与重定位算法
    for (size_t submap_id = 0; submap_id < pose_list.size(); ++submap_id)
    {
        T.start(2);
        T.start(4);
        if (!isRunning)
            break;

        bool isPlaceRecognitionOkey = false;

        pcl::PointCloud<PointType>::Ptr curr_cloud(new pcl::PointCloud<PointType>()); // 本帧点云
        pcl::PointCloud<PointType>::Ptr near_trans_cloud(new pcl::PointCloud<PointType>()); // near帧全局坐标系下点云
        if (read_bin) {
            if (!loadPointcloudBinAndTrans(curr_cloud, submap_id)) 
                return;
        } else {
            if (!loadPointcloudPcdAndTrans(curr_cloud, submap_id)) 
                return;
        }
        transPclPointCloud(curr_cloud, pose_list[submap_id].first, pose_list[submap_id].second); //转移到odom系

        // 【优化】用sub-map代替本帧
        static size_t numToMerge = 3
        ;
        if (submap_trans_cloud.size() >= numToMerge)
            submap_trans_cloud.pop_front();
        submap_trans_cloud.push_back(curr_cloud);
        for (auto thisCloud:submap_trans_cloud)
            *near_trans_cloud += *thisCloud;

        // 点云预处理
        // pointCloudPreprocess(near_trans_cloud, near_trans_cloud);

        btc_manager->key_ori_cloud_vec_.push_back(near_trans_cloud);
        
        // step1. 描述符提取
        // std::cout << "[Description] submap id:" << submap_id << std::endl;
        auto t_descriptor_begin = std::chrono::high_resolution_clock::now();

        std::vector<BTC> btcs_vec; // 本帧的BTC特征集
        btc_manager->GenerateBtcDescs(near_trans_cloud->makeShared(), submap_id, btcs_vec);

        auto t_descriptor_end = std::chrono::high_resolution_clock::now();
        descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

        // T.print(2, "[BTC] Timer cost of btc making: "); // 不测试时间时，注释之。

        // step2. 搜索回环
        auto t_query_begin = std::chrono::high_resolution_clock::now();
        std::pair<int, double> search_result(-1, 0); // first=过去的id, second=匹配得分
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<BTC, BTC>> loop_std_pair;

        if (submap_id > config_setting.skip_near_num_)
        {
            if (btcs_vec.size() == 0)
            {
                ROS_ERROR("[BTC] The BTC is Empty!");
                continue;
            }
            btc_manager->SearchLoop(btcs_vec, search_result, loop_transform, loop_std_pair);
        }

        if (search_result.first > 0)
        {
            std::cout << "[Loop Detection] triggle loop: " << submap_id << "--"
                        << search_result.first << ", score:" << search_result.second
                        << std::endl;
        }
        auto t_query_end = std::chrono::high_resolution_clock::now();
        querying_time.push_back(time_inc(t_query_end, t_query_begin));

        // step3. 将本帧BTC描述符 加入到 全局数据库
        auto t_map_update_begin = std::chrono::high_resolution_clock::now();
        btc_manager->AddBtcDescs(btcs_vec);
        auto t_map_update_end = std::chrono::high_resolution_clock::now();
        update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));

        pcl::PointCloud<PointType>::Ptr loop_cloud(new pcl::PointCloud<PointType>());// 回环帧的点云(单帧)
        if (search_result.first >= 0)
        {
            if (read_bin) {
                if (!loadPointcloudBinAndTrans(loop_cloud, search_result.first)) 
                    return;
            } else {
                if (!loadPointcloudPcdAndTrans(loop_cloud, search_result.first)) 
                    return;
            }
            transPclPointCloud(loop_cloud, pose_list[search_result.first].first, pose_list[search_result.first].second);
            down_sampling_voxel(*loop_cloud, 0.5);// 手动进行体素平均降采样    
        }

        // T.print(4, "[BTC] Timer cost of rough(include add btcs and sth.) btc: ");

        down_sampling_voxel(*curr_cloud, 0.5);// 手动进行体素平均降采样

        // step4. 可视化
        {
            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(*curr_cloud, pub_cloud);
            pub_cloud.header.frame_id = "odom";
            pubCureentCloud.publish(pub_cloud); // 发布1 坐标变换后的本帧点云(只发单帧)
    
            pcl::PointCloud<pcl::PointXYZ> key_points_cloud;
            for (auto var : btc_manager->history_binary_list_.back())
            {
                pcl::PointXYZ pi;
                pi.x = var.location_[0];
                pi.y = var.location_[1];
                pi.z = var.location_[2];
                key_points_cloud.push_back(pi);
            }
            pcl::toROSMsg(key_points_cloud, pub_cloud);
            pub_cloud.header.frame_id = "odom";
            pubCurrentBinary.publish(pub_cloud); // 发布2 本帧点云提取的binary特征（三角形描述符的顶点位置）
    
            // 发布3 本帧的平面【改进】
            static int whatToPub = 2; //1: 合并后平面 2：原始平面
            std::vector<std::shared_ptr<Plane>> current_planes;
            if (whatToPub == 1)
                current_planes = btc_manager->plane_merged_vec.back();
            else if (whatToPub == 2)
                current_planes = btc_manager->plane_ori_vec.back();

            int plane_num_to_pub = std::min(500, int(current_planes.size()));
            std::vector<std::pair<pcl::PointXYZINormal,std::pair<Eigen::Vector3d,std::pair<float,float>>>> planes_cloud_rgb_rr;
            for (int i=0; i<plane_num_to_pub; ++i)
            {
                auto this_plane = current_planes[i];
                pcl::PointXYZINormal this_plane_point;
                this_plane_point.x = this_plane->center_[0];
                this_plane_point.y = this_plane->center_[1];
                this_plane_point.x = this_plane->center_[0];
                this_plane_point.normal_x = this_plane->normal_[0];
                this_plane_point.normal_y = this_plane->normal_[1];
                this_plane_point.normal_z = this_plane->normal_[2];
    
                Eigen::Vector3d this_rgb;
                float this_ratio;
                if (whatToPub == 1)
                {
                    switch (i)
                    {
                    case 0:
                        this_rgb = Eigen::Vector3d(0.0, 0.8, 0.0); // 深绿色
                        this_ratio = 3.0;
                        break;
                    case 1:
                        this_rgb = Eigen::Vector3d(0.0, 0.0, 0.6); // 深蓝色
                        this_ratio = 1.25;
                        break;
                    case 2:
                        this_rgb = Eigen::Vector3d(0.4, 0.4, 0.0); // 浅黄色
                        this_ratio = 0.7;
                        break;
                    default:
                        this_rgb = Eigen::Vector3d(0.2, 0.2, 0.2); // 浅灰色
                        this_ratio = 0.35;
                        break;
                    }
                }
                else if (whatToPub == 2){
                    this_rgb = Eigen::Vector3d(0.7, 0.7, 0.0); // 黄色
                    this_ratio = 1.5;
                }

                planes_cloud_rgb_rr.push_back(
                    std::pair<pcl::PointXYZINormal,std::pair<Eigen::Vector3d,std::pair<float,float>>>(this_plane_point, 
                        std::pair<Eigen::Vector3d,std::pair<float,float>>(this_rgb, 
                            std::pair<float,float>(this_plane->radius_, this_ratio))));
            }
            pubPlane(pubCurrentPlane, "current_plane", submap_id, 0.1, planes_cloud_rgb_rr);
    
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = "colored_path";
            marker.id = submap_id;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
    
            // 粗匹配成功才会执行
            if (search_result.first >= 0)
            {
                triggle_loop_num++;
    
                // 发布4 粗匹配所对应的2帧的三角形们
                Eigen::Matrix4d transform1 = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d transform2 = Eigen::Matrix4d::Identity();
                publish_std(loop_std_pair, transform1, transform2, pubBTC);
                slow_loop.sleep();
    
                // 发布5 本帧回环匹配到的old帧的所有三角形描述符顶点位置
                pcl::PointCloud<pcl::PointXYZ> match_key_points_cloud; 
                for (auto var : btc_manager->history_binary_list_[search_result.first])
                {
                    pcl::PointXYZ pi;
                    pi.x = var.location_[0];
                    pi.y = var.location_[1];
                    pi.z = var.location_[2];
                    match_key_points_cloud.push_back(pi);
                }
                pcl::toROSMsg(match_key_points_cloud, pub_cloud);
                pub_cloud.header.frame_id = "odom";
                pubMatchedBinary.publish(pub_cloud);
    
                // 点云重合检测（使用 手动体素平均降采样的点云）
                double cloud_overlap = calc_overlap(curr_cloud, loop_cloud, 0.5);
                
                // BTC检测的回环帧，点云很重合，场景识别成功
                if (cloud_overlap >= cloud_overlap_thr)
                {
                    true_loop_num++;
                    
                    // 保存场景识别成功的信息
                    isPlaceRecognitionOkey = true;
                    savePlaceRecognitionList(submap_id, search_result.first, loop_transform, loop_std_pair);
                    std::cout << "[Loop Detection] place recognition success: " << submap_id << "--"
                        << search_result.first << std::endl;

                    // 发布6A 粗匹配BTC匹配到，且点云很重和的，old帧的点云（绿色），好的回环点
                    pcl::PointCloud<pcl::PointXYZRGB> matched_cloud;
                    matched_cloud.resize(loop_cloud->size());
    
                    // 遍历BTC匹配到的old帧的每个点
                    for (size_t i = 0;
                            i < loop_cloud->size();
                            i++)
                    {
                        pcl::PointXYZRGB pi;
                        pi.x =
                            loop_cloud->points[i].x;
                        pi.y =
                            loop_cloud->points[i].y;
                        pi.z =
                            loop_cloud->points[i].z;
                        pi.r = 0;
                        pi.g = 255;
                        pi.b = 0;
                        matched_cloud.points[i] = pi;
                    }
                    pcl::toROSMsg(matched_cloud, pub_cloud);
                    pub_cloud.header.frame_id = "odom";
                    pubMatchedCloud.publish(pub_cloud);
                    slow_loop.sleep();
    
                    // 当前帧与上一帧构成连线add
                    marker.scale.x = scale_tp;
                    marker.color = color_tp; // g
                    geometry_msgs::Point point1;
                    point1.x = pose_list[submap_id - 1].first[0];
                    point1.y = pose_list[submap_id - 1].first[1];
                    point1.z = pose_list[submap_id - 1].first[2];
                    geometry_msgs::Point point2;
                    point2.x = pose_list[submap_id].first[0];
                    point2.y = pose_list[submap_id].first[1];
                    point2.z = pose_list[submap_id].first[2];
                    marker.points.push_back(point1);
                    marker.points.push_back(point2);
                }
    
                // BTC检测的回环帧，点云不咋重合，场景识别失败
                else
                {
                    // 发布6B 粗匹配BTC匹配到，且点云不重和的，过去帧的点云（红色），坏的回环点
                    pcl::PointCloud<pcl::PointXYZRGB> matched_cloud;
                    matched_cloud.resize(loop_cloud->size());
                    for (size_t i = 0;
                            i < loop_cloud->size();
                            i++)
                    {
                        pcl::PointXYZRGB pi;
                        pi.x =
                            loop_cloud->points[i].x;
                        pi.y =
                            loop_cloud->points[i].y;
                        pi.z =
                            loop_cloud->points[i].z;
                        pi.r = 255;
                        pi.g = 0;
                        pi.b = 0;
                        matched_cloud.points[i] = pi;
                    }
                    pcl::toROSMsg(matched_cloud, pub_cloud);
                    pub_cloud.header.frame_id = "odom";
                    pubMatchedCloud.publish(pub_cloud);
                    slow_loop.sleep();
    
                    // 当前帧与上一帧构成连线add
                    marker.scale.x = scale_fp;
                    marker.color = color_fp;
                    geometry_msgs::Point point1;
                    point1.x = pose_list[submap_id - 1].first[0];
                    point1.y = pose_list[submap_id - 1].first[1];
                    point1.z = pose_list[submap_id - 1].first[2];
                    geometry_msgs::Point point2;
                    point2.x = pose_list[submap_id].first[0];
                    point2.y = pose_list[submap_id].first[1];
                    point2.z = pose_list[submap_id].first[2];
                    marker.points.push_back(point1);
                    marker.points.push_back(point2);
                }
            }
            // 如果没有BTC回环成功！
            else
            {
                if (submap_id > 0)
                {
                    // 把路径描绘一下
                    marker.scale.x = scale_path;
                    marker.color = color_path;
                    geometry_msgs::Point point1;
                    point1.x = pose_list[submap_id - 1].first[0];
                    point1.y = pose_list[submap_id - 1].first[1];
                    point1.z = pose_list[submap_id - 1].first[2];
                    geometry_msgs::Point point2;
                    point2.x = pose_list[submap_id].first[0];
                    point2.y = pose_list[submap_id].first[1];
                    point2.z = pose_list[submap_id].first[2];
                    marker.points.push_back(point1);
                    marker.points.push_back(point2);
                }
            }
    
            // 发布7 位姿的连线（包含了回环的状态信息）
            marker_array.markers.push_back(marker);
            pubLoopStatus.publish(marker_array);
            loop.sleep();
        }

        // 场景识别成功，开始重定位精匹配
        if (isPlaceRecognitionOkey)
            reLocation();
    }
}



bool detectRadiusSearchLoop(const int currID, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pose_cloud_map, 
    const float searchRadius, const float searchTimeTh, const int searchIdxTh, int& loopID)
{
    if (pose_cloud_map->empty() || currID<0 || currID>=pose_list.size())
        return false;

    pcl::PointXYZ pose_point_this;
    pose_point_this.x = pose_list[currID].first[0];
    pose_point_this.y = pose_list[currID].first[1];
    pose_point_this.z = pose_list[currID].first[2];

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreePoses(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreePoses->setInputCloud(pose_cloud_map);
    kdtreePoses->radiusSearch(pose_point_this, searchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    // 在候选关键帧集合中，找到1个与当前帧时间相隔较远的帧，设为候选匹配帧
    loopID = -1;
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(time_list[id] - time_list[currID]) > searchTimeTh
            && abs(id - currID) >searchIdxTh)
        {
            loopID = id;
            break;
        }
    }

    if (loopID == -1 || currID == loopID)
    {
        loopID = -1;
        return false;
    }

    // std::cout<<"******debug time="<<abs(time_list[loopID] - time_list[currID])<<std::endl;
    // std::cout<<"******debug dis="<<(pose_list[currID].first-pose_list[loopID].first).norm()<<std::endl;

    return true;
}

void ndtLoopDetect(OnlineLTSlam::ndtLocalizer& ndt_, const float* radiusSearchParam)
{
    // 所有pose格式转换 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pose_cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
    pose_cloud_map->reserve(pose_list.size());

    // 加载所有点云
    std::vector<pcl::PointCloud<PointType>::Ptr> ori_clouds(pose_list.size()); // 所有的原始点云（lidar系）
    T.start(0);
    loadAllPointCloud(ori_clouds, false);
    T.print(0, "[NDT] Timer cost of load all.pcd ndt: ");

    
    
    ros::Rate rate(100);
    for (size_t i=0; i<time_list.size(); ++i)  // 遍历每一帧寻求回环
    {
        T.start(1);
        if (!isRunning)
            break;

        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
        transPclPointCloud(ori_clouds[i], pose_list[i].first, pose_list[i].second, cloud_trans);

        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*cloud_trans, pub_cloud);
        pub_cloud.header.frame_id = "odom";
        pubCureentCloud.publish(pub_cloud); // 发布1 坐标变换后的本帧点云

        visualization_msgs::MarkerArray marker_array; // 预备 发布2 位姿
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.ns = "colored_path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        // 动态增量式构建点云：每次循环添加前一帧的pose（i=0时无操作）,此时pose_cloud_map包含0~i-1帧的pose
        if (i > 0) 
        { 
            const auto& prev_pose = pose_list[i-1]; // 直接访问第i-1帧
            pose_cloud_map->push_back(pcl::PointXYZ(
                prev_pose.first[0], 
                prev_pose.first[1], 
                prev_pose.first[2]
            ));
        }

        // 基于距离检测回环
        int rsLoopID = -1;
        if (!detectRadiusSearchLoop(i, pose_cloud_map, radiusSearchParam[0], radiusSearchParam[1], int(radiusSearchParam[2]), rsLoopID))
        {
            if (i > 0)
            {
                // 没触发基础的距离时间回环
                // 发布2 位姿-白色
                marker.scale.x = scale_path;
                marker.color = color_path;
                geometry_msgs::Point point1;
                point1.x = pose_list[i - 1].first[0];
                point1.y = pose_list[i - 1].first[1];
                point1.z = pose_list[i - 1].first[2];
                geometry_msgs::Point point2;
                point2.x = pose_list[i].first[0];
                point2.y = pose_list[i].first[1];
                point2.z = pose_list[i].first[2];
                marker.points.push_back(point1);
                marker.points.push_back(point2);
                marker_array.markers.push_back(marker); 
                pubLoopStatus.publish(marker_array);
            }

            rate.sleep();
            continue;
        }

        // 本帧全局位姿
        auto currPoseOri = pose_list[i];
        auto loopPoseOri = pose_list[rsLoopID];
        Eigen::Matrix4d thisPoseMat = Eigen::Matrix4d::Identity();
        thisPoseMat.block<3,3>(0,0) = currPoseOri.second;
        thisPoseMat.block<3,1>(0,3) = currPoseOri.first;
        geometry_msgs::PoseStamped result_ndt;

        // 本帧（拼接）点云、过去帧点云子地图
        pcl::PointCloud<PointType>::Ptr rsLoopSubmap(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr rsCurrSubmap_curr_lidar(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr rsCurrSubmap(new pcl::PointCloud<PointType>());
        static int halfNumOfNear2MergeLoopSubmap = 4;
        static int halfNumOfNear2MergeCurrSubmap = 0;
        for (int j=std::max(0,int(rsLoopID)-halfNumOfNear2MergeLoopSubmap); j<=std::min(int(pose_list.size()),int(rsLoopID)+halfNumOfNear2MergeLoopSubmap); j++)
        {
            pcl::PointCloud<PointType>::Ptr old_cloud_trans(new pcl::PointCloud<PointType>());
            transPclPointCloud(ori_clouds[j], pose_list[j].first, pose_list[j].second, old_cloud_trans);
            *rsLoopSubmap += *old_cloud_trans;
        }
        for (int j=std::max(0,int(i)-halfNumOfNear2MergeCurrSubmap); j<=std::min(int(pose_list.size()),int(i)+halfNumOfNear2MergeCurrSubmap); j++)
        {
            pcl::PointCloud<PointType>::Ptr near_cloud_trans(new pcl::PointCloud<PointType>());
            transPclPointCloud(ori_clouds[j], pose_list[j].first, pose_list[j].second, near_cloud_trans);
            *rsCurrSubmap += *near_cloud_trans; // 当前帧的临近帧都变到世界坐标系下拼接
        }
        // 拼接后再变回到当前帧的lidar坐标系下
        transPclPointCloud(rsCurrSubmap, 
            -pose_list[i].second.transpose()*pose_list[i].first, 
            pose_list[i].second.transpose(), 
            rsCurrSubmap_curr_lidar);        

        bool isNdtSuccess = false;
        std::cout<<std::endl<<"--------------Frame["<<i<<"]'s NDT--------------"<<std::endl;
        isNdtSuccess = ndt_.ndtAlignOnce(result_ndt, rsCurrSubmap_curr_lidar, ros::Time(time_list[i]),  // lidar的curr，odom的loop-submap
                                         rsLoopSubmap, thisPoseMat.cast<float>(), true);

        
        if (!isNdtSuccess)
        {
            // 通过了距离时间回环检测，但是NDT匹配失败了
            // 发布2 位姿-红色
            marker.scale.x = scale_fp;
            marker.color = color_fp;
            geometry_msgs::Point point1;
            point1.x = pose_list[i - 1].first[0];
            point1.y = pose_list[i - 1].first[1];
            point1.z = pose_list[i - 1].first[2];
            geometry_msgs::Point point2;
            point2.x = pose_list[i].first[0];
            point2.y = pose_list[i].first[1];
            point2.z = pose_list[i].first[2];
            marker.points.push_back(point1);
            marker.points.push_back(point2);
            marker_array.markers.push_back(marker);
            pubLoopStatus.publish(marker_array);

            std::cout << "[NDT] Loop State: RS-true, NDT-failed: " << i<<"&"<<rsLoopID << std::endl;

            rate.sleep();
            continue;
        }

        // 发布2 位姿-绿色
        marker.scale.x = scale_tp;
        marker.color = color_tp;
        geometry_msgs::Point point1;
        point1.x = pose_list[i - 1].first[0];
        point1.y = pose_list[i - 1].first[1];
        point1.z = pose_list[i - 1].first[2];
        geometry_msgs::Point point2;
        point2.x = pose_list[i].first[0];
        point2.y = pose_list[i].first[1];
        point2.z = pose_list[i].first[2];
        marker.points.push_back(point1);
        marker.points.push_back(point2);
        marker_array.markers.push_back(marker);
        pubLoopStatus.publish(marker_array);
        

        std::pair<Eigen::Vector3d, Eigen::Matrix3d> currPoseAft;
        currPoseAft.first[0] = result_ndt.pose.position.x;
        currPoseAft.first[1] = result_ndt.pose.position.y;
        currPoseAft.first[2] = result_ndt.pose.position.z;
        Eigen::Quaterniond thisQ(result_ndt.pose.orientation.w,
            result_ndt.pose.orientation.x,
            result_ndt.pose.orientation.y,
            result_ndt.pose.orientation.z);
        currPoseAft.second = thisQ.toRotationMatrix();

        // PosesDiff poseDiffCurrBtwNdtBefAft = calculateDiffBtw2Poses(currPoseOri, currPoseAft);
        PosesDiff poseDiffBefNdtBtwLoopCurr = calculateDiffBtw2Poses(loopPoseOri, currPoseOri);
        Eigen::Vector3d thisPoseOriRot = turnRadianVec3dToDegreeVec3d(currPoseOri.second.eulerAngles(0, 1, 2));
        Eigen::Vector3d thisPoseAftRot = turnRadianVec3dToDegreeVec3d(currPoseAft.second.eulerAngles(0, 1, 2));

        
        // for debug 
        // static int number2save = 0;
        // static int nowNumber = 0;
        // if (number2save == nowNumber++)
        if(i == keyFrameIdTestPrint)
        {
            pcl::PointCloud<PointType>::Ptr curr_points_opti_by_p2picp(new pcl::PointCloud<PointType>);
            transPclPointCloud(rsCurrSubmap_curr_lidar, currPoseAft.first, currPoseAft.second, curr_points_opti_by_p2picp);
    
            pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523-ndt/ndt-curr_points_odom.pcd", *rsCurrSubmap);
            pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523-ndt/ndt-curr_points_opti.pcd", *curr_points_opti_by_p2picp);
            pcl::io::savePCDFile("/home/jixuanlee/pcdsOF523-ndt/ndt-loop_points_odom.pcd", *rsLoopSubmap); // 这里直接使用了loop的submap
            savePosesToTxt("/home/jixuanlee/pcdsOF523-ndt/ndt-poses-for-compare.txt", loopPoseOri, currPoseOri, currPoseAft);
        }

        // 打印
        // Trans：根据输入pose，2帧间定位的变换量（含误差）
        // Position：优化前后，本帧定位数据
        // Change：优化前后，本帧定位数据被优化了多少
        const static std::string GREEN_COLOR = "\033[32m";
        const static std::string RESET_COLOR = "\033[0m";
        std::cout <<GREEN_COLOR<< "[NDT] Loop State: RS-true, NDT-true: " << i<<"&"<<rsLoopID <<RESET_COLOR<< std::endl;
        std::cout << "[NDT] Trans btw curr&loop bef: Translation: " << poseDiffBefNdtBtwLoopCurr.pose_diff.first.transpose() << std::endl;
        std::cout << "[NDT] Trans btw curr&loop bef: Rotation: " << poseDiffBefNdtBtwLoopCurr.rot_diff_rpy_deg.transpose() <<" in RPY deg."<< std::endl;
        std::cout <<GREEN_COLOR<< "[NDT] Position of curr bef: Translation: " << currPoseOri.first.transpose() <<RESET_COLOR<< std::endl;
        std::cout << "[NDT] Position of curr bef: Rotation: " << thisPoseOriRot.transpose() << " in RPY deg."<< std::endl;
        std::cout <<GREEN_COLOR<< "[NDT] Position of curr aft: Translation: " << currPoseAft.first.transpose() <<RESET_COLOR<< std::endl;
        std::cout << "[NDT] Position of curr aft: Rotation: " << thisPoseAftRot.transpose() << " in RPY deg."<< std::endl;
        std::cout << "[NDT] Change of curr position: Translation: [" << (currPoseOri.first - currPoseAft.first).norm() << "] m."<< std::endl;
        // std::cout<<std::endl;

        T.print(1, "[NDT] Timer cost of all ndt: ");
        std::cout<<std::endl;

        rate.sleep();
    }
}

void ndtEntrance(ros::NodeHandle& nh)
{
    // 初始化NDT插件
    OnlineLTSlam::ndtLocalizer ndt_(nh);
    static double trans_epsilon = 0.05;
    static double step_size = 0.10;
    static double resolution = 2.0 ;
    static int max_iterations = 30;
    static double converged_param_transform_probability = 5.0;
    ndt_.getNdtParam("odom", "lidar", trans_epsilon, step_size, resolution, max_iterations, converged_param_transform_probability);

    // 执行NDT
    static float radiusSearchParam[3] = {10.0, 30.0, 40.0}; // const float searchRadius, const float searchTimeTh, const int searchIdxTh
    ndtLoopDetect(ndt_, radiusSearchParam);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "place_recognition_and_relocation");
    ros::NodeHandle nh;

    signal(SIGINT, signalHandler);

    initParam(nh);

    loadPoses();

    static int mode = 0; // O:BTC; 1:NDT
    if(mode == 0)
        placeRecognition();
    else if (mode == 1)
        ndtEntrance(nh);
    else     
        std::cout<<"[BTC]Error mode!"<<std::endl;

    
    return 0;
}