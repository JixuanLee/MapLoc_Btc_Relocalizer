/*
 * @Author: Jixuan Lee
 * @Date: 2025-01-13 11:38:58
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-01-16 12:07:59
 * @FilePath: /OnlineLTSlam/src/TOOL/files_to_ros/src/files_to_ros.cpp
 * @Description: Extract the pose and pointcloud info of keyframes from the files which we have got, 
 *               and output ROS format pointcloud, odometer, low-frequency global pointcloud map, 
 *               and cumulative path for each keyframe.
 * @Logs: 
 */


#include "files_to_ros/files_to_ros.h"

std::vector<std::pair<double, PointTypePose>> pathVec;
pcl::PointCloud<PointType>::Ptr globalPointCloudFramePtr(new pcl::PointCloud<PointType>);
nav_msgs::Path globalPath;

/**
 * @brief Using Ctrl+C to stop the program.
 * @param signal 
 */
void signalHandler(int signal)
{
    ROS_INFO("[f2r] Received signal %d, saving global map and shutting down.", signal);
    isRunning = false;
}

/**
 * @brief Get Pcd files' paths and join them into a HashMap.
 */
void createPcdFilesNameHashMap()
{
    for (const auto& entry : fs::directory_iterator(file_pcds))
    {
        if (entry.path().extension() == ".pcd")
        {
            std::string filename = entry.path().filename().string();
            size_t dot_pos = filename.find_last_of('.');
            std::string timestamp_str = filename.substr(0, dot_pos);
            pcd_files_map[timestamp_str] = entry.path().string();
        }pcl::PointCloud<PointType>::Ptr thisPointCloudFramePtr(new pcl::PointCloud<PointType>);

    }
    std::cout<<"Extract all pcd file names."<<std::endl;
}

/**
 * @brief When program needs to save/pub global map in global coordinate system
 * such as "/odom", this func helps to transform global coordinate system to its own init position.
 * @param p The pose we need to trans
 * @param pInit The bias pose referenced when converting coordinate systems
 */
void transToInit(PointTypePose& p, const PointTypePose& pInit)
{
    p.x = p.x - pInit.x;
    p.y = p.y - pInit.y;
    p.z = p.z - pInit.z;
    // p.x =0;
    // p.y=0;
    // p.z=0;
    // p.roll = p.roll - pInit.roll;
    // p.pitch = p.pitch - pInit.pitch;
    // p.yaw = p.yaw - pInit.yaw;
    // p.yaw = 0;
    // p.pitch = 0;
    // p.roll = 0;
}

/**
 * @brief This func refers the transfromIn to transform the cloudIn into a new (probably global) coordinate.
 * @param cloudIn 
 * @param transformIn 
 * @return pcl::PointCloud<PointType>::Ptr 
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

/**
 * @brief Initiate the system and parameters.
 * @param nh 
 */
void initParam(ros::NodeHandle& nh)
{
    nh.param("/files_to_ros/base_path", base_path, std::string("/home/jixuanlee/datasetBXN/2025-01-01-09-30-30/"));
    nh.param("/files_to_ros/file_pcds", file_pcds, std::string("deskew_cloud/"));
    nh.param("/files_to_ros/path_poses", path_poses, std::string("opti_pose_enu.txt"));
    nh.param("/files_to_ros/path_save_global_pcd", path_save_global_pcd, std::string("GlobalMap.pcd"));

    nh.param("/files_to_ros/leafSize", leafSize, {4.0});
    nh.param("/files_to_ros/skipHz", skipHz, {5});
    nh.param("/files_to_ros/doPubWithFrequency", doPubWithFrequency, true);
    nh.param("/files_to_ros/pubFrequency", pubFrequency, {10.0});

    nh.param("/files_to_ros/topicOriPcd", topicOriPcd, std::string("/BXN/ori_pc"));
    nh.param("/files_to_ros/topicGlobalPcd", topicGlobalPcd, std::string("/BXN/map_pc"));
    nh.param("/files_to_ros/topicGlobalPath", topicGlobalPath, std::string("/BXN/path"));
    nh.param("/files_to_ros/topicScanOdom", topicScanOdom, std::string("/BXN/odometry"));

    file_pcds = base_path + file_pcds;
    path_poses = base_path + path_poses;
    path_save_global_pcd = base_path + path_save_global_pcd;
    
    downSizeFilterGlobalPCD.setLeafSize(leafSize, leafSize, leafSize);

    pubOriPcd = nh.advertise<sensor_msgs::PointCloud2>(topicOriPcd, 10);
    pubGlobalPcd = nh.advertise<sensor_msgs::PointCloud2>(topicGlobalPcd, 10);
    pubGlobalPath = nh.advertise<nav_msgs::Path>(topicGlobalPath, 10);
    pubScanOdom = nh.advertise<nav_msgs::Odometry>(topicScanOdom, 10);

    isRunning = true;
    numberOfCores = 8;
}

/**
 * @brief Pub the pointcloud.
 * @param data What to pub
 * @param frame What's the frame_id
 * @param pubObj Which pub object we use
 */
void pubPointCloud(const pcl::PointCloud<PointType>::Ptr& data, std::string frame, ros::Publisher& pubObj)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*data,msg);
    msg.header.frame_id = frame;
    pubObj.publish(msg);
}

/**
 * @brief Pub the path.
 * @param pose What to pub
 * @param timestamp What the pub-data need
 */
void pubPathTopic(const PointTypePose& pose, const double& timestamp)
{
    geometry_msgs::PoseStamped thisPath;
    thisPath.header.frame_id = "odom";
    thisPath.pose.position.x = pose.x;
    thisPath.pose.position.y = pose.y;
    thisPath.pose.position.z = pose.z;
    tf::Quaternion thisQ;
    thisQ.setRPY(pose.roll, pose.pitch, pose.yaw);
    tf::Matrix3x3 thisM(thisQ);
    thisM.getRotation(thisQ);
    thisPath.pose.orientation.x = thisQ.x();
    thisPath.pose.orientation.y = thisQ.y();
    thisPath.pose.orientation.z = thisQ.z();
    thisPath.pose.orientation.w = thisQ.w();
    globalPath.poses.push_back(thisPath);
    globalPath.header.frame_id = "odom";
    globalPath.header.stamp.fromSec(timestamp);
    pubGlobalPath.publish(globalPath);
}

/**
 * @brief Pub the odometry.
 * @param pose What to pub
 * @param timestamp What the pub-data need
 */
void pubOdometryTopic(const PointTypePose& pose, const double& timestamp)
{
    nav_msgs::Odometry thisOdom;
    thisOdom.header.frame_id = "odom";
    thisOdom.header.stamp.fromSec(timestamp);
    thisOdom.child_frame_id = "lidar";
    thisOdom.pose.pose.position.x = pose.x;
    thisOdom.pose.pose.position.y = pose.y;
    thisOdom.pose.pose.position.z = pose.z;
    tf::Quaternion thisQ;
    thisQ.setRPY(pose.roll, pose.pitch, pose.yaw);
    tf::Matrix3x3 thisM(thisQ);
    thisM.getRotation(thisQ);
    thisOdom.pose.pose.orientation.x = thisQ.x();
    thisOdom.pose.pose.orientation.y = thisQ.y();
    thisOdom.pose.pose.orientation.z = thisQ.z();
    thisOdom.pose.pose.orientation.w = thisQ.w();
    pubScanOdom.publish(thisOdom);
}

/**
 * @brief Save the global map as pcd.
 * @param savePath Where to save
 * @param globalPtr What to save
 */
void saveGlobalPcd(const std::string& savePath, const pcl::PointCloud<PointType>::Ptr& globalPtr)
{
    pcl::io::savePCDFileASCII(savePath, *globalPtr);
    std::cout<<"[f2r] Save PCD-map (in /odom) now, at: "<<savePath.c_str()<<std::endl<<std::endl;
}

/**
 * @brief Read and get the poses data from .txt.
 */
void loadPoses()
{
    PointTypePose thisPose;
    std::ifstream posesDataFile(path_poses);

    if (!posesDataFile.is_open())
    {
        ROS_ERROR("[f2r] Failed to open Poses txt!");
        return;
    }

    std::string lineOfPoses;
    bool isFirst = true;
    while (std::getline(posesDataFile, lineOfPoses))
    {
        // std::fixed >> std::setprecision(6)控制小数点后的精度
        std::istringstream iss(lineOfPoses);
        iss >> std::fixed >> std::setprecision(6) >> thisPose.time;
        iss >> std::fixed >> std::setprecision(6) >> thisPose.x;
        iss >> std::fixed >> std::setprecision(6) >> thisPose.y;
        iss >> std::fixed >> std::setprecision(6) >> thisPose.z;

        double ox, oy, oz, ow, rr, pp, yy;
        iss >> std::fixed >> std::setprecision(6) >> ox;
        iss >> std::fixed >> std::setprecision(6) >> oy;
        iss >> std::fixed >> std::setprecision(6) >> oz;
        iss >> std::fixed >> std::setprecision(6) >> ow;

        tf::Quaternion thisQ(ox, oy, oz, ow);
        tf::Matrix3x3 thisM(thisQ);
        thisM.getRPY(rr, pp, yy);

        thisPose.roll = rr;
        thisPose.pitch = pp;
        thisPose.yaw = yy;

        if (isFirst)
        {
            isFirst = false;
            initPose = thisPose;
        }

        pathVec.push_back(std::pair<double, PointTypePose>(thisPose.time, thisPose));

        // 打印位姿矩阵以验证结果
        // static int nnn = 9;
        // if (nnn-- >= 0)
        // {
        //     std::cout << "[f2r] this pose:" << std::endl;
        //     std::cout << "[f2r] time: " << std::fixed << std::setprecision(6) << thisPose.time << std::endl;
        //     std::cout << "[f2r] x: " << std::fixed << std::setprecision(6) << thisPose.x << std::endl;
        //     std::cout << "[f2r] y: " << std::fixed << std::setprecision(6) << thisPose.y << std::endl;
        //     std::cout << "[f2r] z: " << std::fixed << std::setprecision(6) << thisPose.z << std::endl;
        //     std::cout << "[f2r] ox: " << std::fixed << std::setprecision(6) << ox << std::endl;
        //     std::cout << "[f2r] oy: " << std::fixed << std::setprecision(6) << oy << std::endl;
        //     std::cout << "[f2r] oz: " << std::fixed << std::setprecision(6) << oz << std::endl;
        //     std::cout << "[f2r] ow: " << std::fixed << std::setprecision(6) << ow << std::endl;
        //     std::cout << "[f2r] roll: " << std::fixed << std::setprecision(6) << thisPose.roll << std::endl;
        //     std::cout << "[f2r] pitch: " << std::fixed << std::setprecision(6) << thisPose.pitch << std::endl;
        //     std::cout << "[f2r] yaw: " << std::fixed << std::setprecision(6) << thisPose.yaw << std::endl;
        //     std::cout << std::endl;
        // }

    }
    posesDataFile.close();

    // 按time字段排序
    std::sort(pathVec.begin(), pathVec.end(),
              [](const std::pair<double, PointTypePose>& a, const std::pair<double, PointTypePose>& b) {
                  return a.first < b.first;
              });

    std::cout << "The num of Poses: " << pathVec.size() << std::endl;
}

/**
 * @brief The main process where we match .pcd with poses by HashMap, create global pcd map and pub.
 */
void dealWithScanPCDs()
{
    pcl::PointCloud<PointType>::Ptr thisPointCloudFramePtr(new pcl::PointCloud<PointType>);

    const static double startTime = ros::Time::now().toSec();
    const int numOfAll = pathVec.size();
    static int numOfNow = 0;
    static ros::Rate r(pubFrequency);
    
    for (const auto& thisPose : pathVec) 
    {
        if (!isRunning)
        {
            saveGlobalPcd(path_save_global_pcd, globalPointCloudFramePtr);
            std::cout<<"[f2r] We totally cost "<<ros::Time::now().toSec()-startTime<<" s during we deal with those pcds."<<std::endl;
            break;
        }
            
        double timestamp = thisPose.first;
        PointTypePose pose = thisPose.second;

        // transToInit(pose, initPose);

        std::stringstream ss;
        ss << std::fixed << std::setprecision(6) << timestamp;
        std::string timestamp_str = ss.str();

        if (pcd_files_map.find(timestamp_str) != pcd_files_map.end()) 
        {
            thisPointCloudFramePtr->clear();
            
            std::string pcd_path = pcd_files_map[timestamp_str];

            if (pcl::io::loadPCDFile<PointType>(pcd_path, *thisPointCloudFramePtr) == 0) 
            {
                // ROS_INFO("[f2r] Loaded PCD file: %s with Time: %.6f",pcd_path.c_str(), timestamp);

                pubPointCloud(thisPointCloudFramePtr, "lidar", pubOriPcd);

                downSizeFilterGlobalPCD.setInputCloud(thisPointCloudFramePtr);
                downSizeFilterGlobalPCD.filter(*thisPointCloudFramePtr);
                *globalPointCloudFramePtr += *transformPointCloud(thisPointCloudFramePtr, &pose);   

                static int timer = 0;
                if (timer++ % skipHz == 0)
                {
                    timer = 0;
                    pubPointCloud(globalPointCloudFramePtr, "odom", pubGlobalPcd);
                }
            }
            else 
            {
                std::cerr << "Failed to load PCD file: " << pcd_path << std::endl;
            }
        } 
        else 
        {
            std::cerr << "PCD file not found for timestamp: " << timestamp << std::endl;
        }


        pubPathTopic(pose, timestamp);
        pubOdometryTopic(pose, timestamp);

        
        if (numOfNow++ % 40 == 0)
            ROS_INFO("[f2r] Now we deal with %.1f%% keyframes, waiting...", float(numOfNow)/float(numOfAll)*100.0);
        if (numOfAll == numOfNow)
        {
            ROS_INFO("[f2r] Now we have dealed with ALL keyframes!");
            saveGlobalPcd(path_save_global_pcd, globalPointCloudFramePtr);
            std::cout<<"[f2r] We totally cost "<<ros::Time::now().toSec()-startTime<<" s during we deal with those pcds."<<std::endl;
        }

        // for DEBUG
        if (numOfNow == 6000)
        {
            saveGlobalPcd(path_save_global_pcd, globalPointCloudFramePtr);
            std::cout<<"[f2r] We totally cost "<<ros::Time::now().toSec()-startTime<<" s during we deal with those pcds."<<std::endl;
            break;
        }

        if (doPubWithFrequency)
            r.sleep();
    }
}

/**
 * @brief Read, get and deal the pointclouds data from .pcd.
 */
void loadScanPCDs()
{
    createPcdFilesNameHashMap();
    dealWithScanPCDs();
}

/**
 * @brief The entrance of process.
 * @param nh 
 */
void run(ros::NodeHandle& nh)
{
    initParam(nh);
    loadPoses();
    loadScanPCDs();
}

/**
 * @brief A temporary functional function that merges two global maps based on their respective biases at the same anchor.
 * @warning This func is not the main stream code, just for testing or doing some necessary work.
 */
void runToMerge2GlobalPcds()
{
    std::string pcd_path1 = "/home/jixuanlee/datasetBXN/2025-01-01-09-30-30/GlobalMap-87-6000.pcd";
    std::string pcd_path2 = "/home/jixuanlee/datasetBXN/2025-01-01-11-35-16/GlobalMap-141.pcd";
    std::string pose_path1 = "/home/jixuanlee/datasetBXN/2025-01-01-09-30-30/opti_pose_enu.txt";
    std::string pose_path2 = "/home/jixuanlee/datasetBXN/2025-01-01-11-35-16/opti_pose_enu.txt";
    std::string path_save_merge_global_pcd = "/home/jixuanlee/datasetBXN/merge.pcd";
    
    pcl::PointCloud<PointType>::Ptr thisPointCloudFramePtr1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr thisPointCloudFramePtr2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr mergePointCloudFramePtr(new pcl::PointCloud<PointType>);

    PointTypePose initPose1;
    PointTypePose initPose2;
    std::string lineOfPoses;

    std::ifstream posesDataFile1(pose_path1);
    if (!posesDataFile1.is_open()) 
    {
        ROS_ERROR("[f2r] Failed to open Poses 1 txt!");
        return;
    }
    std::getline(posesDataFile1, lineOfPoses);
    std::istringstream iss1(lineOfPoses);
    iss1 >> std::fixed >> std::setprecision(6) >> initPose1.time;
    iss1 >> std::fixed >> std::setprecision(6) >> initPose1.x;
    iss1 >> std::fixed >> std::setprecision(6) >> initPose1.y;
    iss1 >> std::fixed >> std::setprecision(6) >> initPose1.z;
    // double ox, oy, oz, ow, rr, pp, yy;
    // iss >> std::fixed >> std::setprecision(6) >> ox;
    // iss >> std::fixed >> std::setprecision(6) >> oy;
    // iss >> std::fixed >> std::setprecision(6) >> oz;
    // iss >> std::fixed >> std::setprecision(6) >> ow;
    // tf::Quaternion thisQ(ox, oy, oz, ow);
    // tf::Matrix3x3 thisM(thisQ);
    // thisM.getRPY(rr, pp, yy);
    initPose1.roll = 0;
    initPose1.pitch = 0;
    initPose1.yaw = 0;
    // 打印位姿矩阵以验证结果
    // static int nnn = 9;
    // if (nnn-- >= 0)
    // {
    //     std::cout << "[f2r] this pose:" << std::endl;
    //     std::cout << "[f2r] time: " << std::fixed << std::setprecision(6) << thisPose.time << std::endl;
    //     std::cout << "[f2r] x: " << std::fixed << std::setprecision(6) << thisPose.x << std::endl;
    //     std::cout << "[f2r] y: " << std::fixed << std::setprecision(6) << thisPose.y << std::endl;
    //     std::cout << "[f2r] z: " << std::fixed << std::setprecision(6) << thisPose.z << std::endl;
    //     std::cout << "[f2r] ox: " << std::fixed << std::setprecision(6) << ox << std::endl;
    //     std::cout << "[f2r] oy: " << std::fixed << std::setprecision(6) << oy << std::endl;
    //     std::cout << "[f2r] oz: " << std::fixed << std::setprecision(6) << oz << std::endl;
    //     std::cout << "[f2r] ow: " << std::fixed << std::setprecision(6) << ow << std::endl;
    //     std::cout << "[f2r] roll: " << std::fixed << std::setprecision(6) << thisPose.roll << std::endl;
    //     std::cout << "[f2r] pitch: " << std::fixed << std::setprecision(6) << thisPose.pitch << std::endl;
    //     std::cout << "[f2r] yaw: " << std::fixed << std::setprecision(6) << thisPose.yaw << std::endl;
    //     std::cout << std::endl;
    // }
    posesDataFile1.close();

    std::ifstream posesDataFile2(pose_path2);
    if (!posesDataFile2.is_open()) 
    {
        ROS_ERROR("[f2r] Failed to open Poses 2 txt!");
        return;
    }
    std::getline(posesDataFile2, lineOfPoses);
    std::istringstream iss2(lineOfPoses);
    iss2 >> std::fixed >> std::setprecision(6) >> initPose2.time;
    iss2 >> std::fixed >> std::setprecision(6) >> initPose2.x;
    iss2 >> std::fixed >> std::setprecision(6) >> initPose2.y;
    iss2 >> std::fixed >> std::setprecision(6) >> initPose2.z;
    // double ox, oy, oz, ow, rr, pp, yy;
    // iss >> std::fixed >> std::setprecision(6) >> ox;
    // iss >> std::fixed >> std::setprecision(6) >> oy;
    // iss >> std::fixed >> std::setprecision(6) >> oz;
    // iss >> std::fixed >> std::setprecision(6) >> ow;
    // tf::Quaternion thisQ(ox, oy, oz, ow);
    // tf::Matrix3x3 thisM(thisQ);
    // thisM.getRPY(rr, pp, yy);
    initPose2.roll = 0;
    initPose2.pitch = 0;
    initPose2.yaw = 0;
    // 打印位姿矩阵以验证结果
    // static int nnn = 9;
    // if (nnn-- >= 0)
    // {
    //     std::cout << "[f2r] this pose:" << std::endl;
    //     std::cout << "[f2r] time: " << std::fixed << std::setprecision(6) << thisPose.time << std::endl;
    //     std::cout << "[f2r] x: " << std::fixed << std::setprecision(6) << thisPose.x << std::endl;
    //     std::cout << "[f2r] y: " << std::fixed << std::setprecision(6) << thisPose.y << std::endl;
    //     std::cout << "[f2r] z: " << std::fixed << std::setprecision(6) << thisPose.z << std::endl;
    //     std::cout << "[f2r] ox: " << std::fixed << std::setprecision(6) << ox << std::endl;
    //     std::cout << "[f2r] oy: " << std::fixed << std::setprecision(6) << oy << std::endl;
    //     std::cout << "[f2r] oz: " << std::fixed << std::setprecision(6) << oz << std::endl;
    //     std::cout << "[f2r] ow: " << std::fixed << std::setprecision(6) << ow << std::endl;
    //     std::cout << "[f2r] roll: " << std::fixed << std::setprecision(6) << thisPose.roll << std::endl;
    //     std::cout << "[f2r] pitch: " << std::fixed << std::setprecision(6) << thisPose.pitch << std::endl;
    //     std::cout << "[f2r] yaw: " << std::fixed << std::setprecision(6) << thisPose.yaw << std::endl;
    //     std::cout << std::endl;
    // }
    posesDataFile2.close();

    if (pcl::io::loadPCDFile<PointType>(pcd_path1, *thisPointCloudFramePtr1))
    {
        std::cerr << "Failed to load PCD file: " << pcd_path1 << std::endl;
        return;
    }
    if (pcl::io::loadPCDFile<PointType>(pcd_path2, *thisPointCloudFramePtr2))
    {
        std::cerr << "Failed to load PCD file: " << pcd_path2 << std::endl;
        return;
    }  
    
    initPose1.x = -initPose1.x;
    initPose1.y = -initPose1.y;
    initPose1.z = -initPose1.z;
    initPose2.x = -initPose2.x;
    initPose2.y = -initPose2.y;
    initPose2.z = -initPose2.z;
    std::cout<<"We get points: "<<thisPointCloudFramePtr1->size()<<" and "<<thisPointCloudFramePtr2->size()<<std::endl;
    // *mergePointCloudFramePtr = *transformPointCloud(thisPointCloudFramePtr1, &initPose1) 
    //                             + *transformPointCloud(thisPointCloudFramePtr2, &initPose2);
    *mergePointCloudFramePtr = *thisPointCloudFramePtr1 + *thisPointCloudFramePtr2;

    saveGlobalPcd(path_save_merge_global_pcd, mergePointCloudFramePtr);
    std::cout<<"We save points: "<<mergePointCloudFramePtr->size()<<std::endl;

    
}

/**
 * @brief The entrance of program.
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "files_to_ros");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> files_to_ros Started.\033[0m");

    signal(SIGINT, signalHandler);
   
    run(nh);
    // runToMerge2GlobalPcds();

    return 0;
}