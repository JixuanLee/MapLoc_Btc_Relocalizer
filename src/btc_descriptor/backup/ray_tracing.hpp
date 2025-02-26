// #include <iostream>
// #include <unordered_map>
// #include "pcl/point_types.h"
// #include "pcl/point_cloud.h"
// #include "glog/logging.h"
// #include <pcl/io/pcd_io.h>
// #include <pcl/common/transforms.h>

// #define M_PI       3.14159265358979323846
// #define HASH_P 116101
// #define MAX_N 10000000000

// //  COPY
// // // align check for ndt
// //     ray_tracing alignCheckOri(0.2,"");
// //     alignCheckOri.make_voxel_map(submap_kf1, kf2->_pose.matrix());

// //     ray_tracing alignCheckAfter(0.2,"");
// //     alignCheckAfter.make_voxel_map(submap_kf1, kf2->_pose.matrix());

// //     double overlap_ori = alignCheckOri.calculate_overlap_rate(submap_kf2, kf2->_pose.matrix());
// //     double overlap_after = alignCheckAfter.calculate_overlap_rate(submap_kf2, Tw2.cast<double>());
// //     LOG(INFO)<<" source idx "<< kf2->_idx <<" target idx "<<kf1->_idx
// //             <<" overlap rate "<<overlap_ori<<" after ndt overlap rate "<<overlap_after;
// //     if( overlap_after - overlap_ori < -0.01){
// //       c.ndt_score_ = 6.11111; 
// //     }

// class VOXEL_LOC{
//   public:
//     int64_t x,y,z;

//   VOXEL_LOC(int64_t vx=0, int64_t vy =0, int64_t vz = 0  )
//     :x(vx), y(vy), z(vz){};
//   bool operator==(const VOXEL_LOC &other) const{
//     return (x == other.x && y == other.y && z == other.z);
//   }
// };

// // Hash value
// namespace std {
// template <>
// struct hash<VOXEL_LOC> {
//   int64_t operator()(const VOXEL_LOC &s) const {
//     using std::hash;
//     using std::size_t;
//     return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
//   }
// };
// }

// namespace sunhigh {

// class VOXEL_INFO{
//   public:
//     double mean_x, mean_y, mean_z;
//     int num;
//     double length;

//   VOXEL_INFO( int count =0 , double length = 0.5)
//     :num(count), length(length){};
// };



// class ray_tracing
// {
//   using PointType = pcl::PointXYZI;
//   using PointCloudType = pcl::PointCloud<PointType>;
//   using CloudPtr = PointCloudType::Ptr;

// private:
//   std::unordered_map<VOXEL_LOC, std::shared_ptr<VOXEL_INFO> > voxel_map;
//   double voxel_length;
//   double filled_voxel_num;
//   std::string out_dir;
//   Eigen::Vector3d origin;
// public:
//   ray_tracing(double length, std::string dir);
//   ~ray_tracing();
//   bool make_voxel_map( const CloudPtr& scan,  const Eigen::Matrix4d& pose);
//   bool make_voxel_map(const CloudPtr& scan, const Eigen::Matrix4d& pose, double range);
//   bool make_voxel_map_noH(const CloudPtr& scan, const Eigen::Matrix4d& pose);
//   bool ray_tracing_check(const CloudPtr& map, const Eigen::Matrix4d& pose, uint idx);
//   double cloud_structure_check(const CloudPtr& map, const Eigen::Matrix4d& pose, uint idx);
//   bool tracing(Eigen::Vector3d& point, const Eigen::Matrix4d& pose, CloudPtr output);
//   double calculate_overlap_rate(const CloudPtr& scan , const Eigen::Matrix4d& pose);
//   double get_valid_voxel_numbers();
// };

// ray_tracing::ray_tracing(double length, std::string dir)
// {
//   voxel_length = length;
//   out_dir = dir;
//   filled_voxel_num = 0;
// }

// ray_tracing::~ray_tracing()
// {
// }

// bool ray_tracing::make_voxel_map(const CloudPtr& scan, const Eigen::Matrix4d& pose){
//   if( scan->points.empty()){
//     LOG(INFO)<<"  make_voxel_map filed : no points in scan cloud ";
//     return false;
//   }

//   origin = Eigen::Vector3d(pose(0,3)-200, pose(1,3)-200, pose(2,3)-10 );
//   for( auto point : scan->points){

//     float loc_xyz[3];
//     loc_xyz[0] = point.x - origin.x();
//     loc_xyz[1] = point.y - origin.y();
//     loc_xyz[2] = point.z - origin.z();

//     for (int j = 0; j < 3; j++) {
//       loc_xyz[j] = loc_xyz[j] / voxel_length;
//       if (loc_xyz[j] < 0) {
//         loc_xyz[j] -= 1.0;
//       }
//     }
//     VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                        (int64_t)loc_xyz[2]);

//     auto iter = voxel_map.find(position);
//     if (iter != voxel_map.end()) {

//       voxel_map[position]->num++ ;
//     } else {

//       voxel_map[position] = std::make_shared<VOXEL_INFO>(0, voxel_length) ;
//       voxel_map[position]->num++ ;
//     }

//   }
//   return true;
  
// }

// bool ray_tracing::make_voxel_map(const CloudPtr& scan, const Eigen::Matrix4d& pose, double range){
//   if( scan->points.empty()){
//     LOG(INFO)<<"  make_voxel_map filed : no points in scan cloud ";
//     return false;
//   }


//   origin = Eigen::Vector3d(pose(0,3)-200, pose(1,3)-200, pose(2,3)-10 );
//   for( auto point : scan->points){

//     float loc_xyz[3];
//     loc_xyz[0] = point.x - origin.x();
//     loc_xyz[1] = point.y - origin.y();
//     loc_xyz[2] = point.z - origin.z();

//     double dis =std::sqrt( std::pow( (point.x - pose(0,3)), 2) + std::pow( (point.y - pose(1,3)), 2) );
//     if( dis > range){
//       continue;
//     }

//     for (int j = 0; j < 3; j++) {
//       loc_xyz[j] = loc_xyz[j] / voxel_length;
//       if (loc_xyz[j] < 0) {
//         loc_xyz[j] -= 1.0;
//       }
//     }
//     VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                        (int64_t)loc_xyz[2]);

//     auto iter = voxel_map.find(position);
//     if (iter != voxel_map.end()) {

//       voxel_map[position]->num++ ;
//     } else {

//       voxel_map[position] = std::make_shared<VOXEL_INFO>(0, voxel_length) ;
//       voxel_map[position]->num++ ;
//     }

//   }
//   return true;
  
// }

// double ray_tracing::get_valid_voxel_numbers(){
//   for(auto voxel : voxel_map){
//     if( voxel.second->num > 5){
//       filled_voxel_num++;
//     }
//   }
//   return filled_voxel_num;
// }

// // calculate overlap rate: intersect points / all points in scan
// double ray_tracing::calculate_overlap_rate(const CloudPtr& scan , const Eigen::Matrix4d& pose){
  
//   CloudPtr transed_cloud(new PointCloudType);
//   pcl::transformPointCloud(*scan, *transed_cloud, pose.cast<float>());

//   double overlapnum =0;
//   for( auto point: transed_cloud->points){
//     float loc_xyz[3];
//     loc_xyz[0] = point.x - origin.x();
//     loc_xyz[1] = point.y - origin.y();
//     loc_xyz[2] = point.z - origin.z();


//     for (int j = 0; j < 3; j++) {
//       loc_xyz[j] = loc_xyz[j] / voxel_length;
//       if (loc_xyz[j] < 0) {
//         loc_xyz[j] -= 1.0;
//       }
//     }

//     VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                        (int64_t)loc_xyz[2]);

//     auto iter = voxel_map.find(position);
//     if (iter != voxel_map.end() && voxel_map[position]->num++ > 5) {
//       overlapnum ++ ;
//     }
//   }
//   return overlapnum/scan->points.size();
// }

// double ray_tracing::cloud_structure_check(const CloudPtr& map, const Eigen::Matrix4d& pose, uint idx){
//   if( map->points.empty()){
//     LOG(INFO)<<"  make_voxel_map filed : no points in map cloud ";
//     return false;
//   }

//   Eigen::Vector3d origin(pose(0,3)-200, pose(1,3)-200, pose(2,3)-10 );
//   std::unordered_map<int, Eigen::Vector3d> angle2MaxHmap;
//   std::unordered_map<int, Eigen::Vector3d> angle2LowHmap;
//   for(auto point : map->points){
//     double x = point.x - pose(0,3);
//     double y = point.y - pose(1,3);
    
//     double angle = std::atan2( y, x) * 180 / M_PI;
//     // select max z point
//     if( angle2MaxHmap.find( int(angle) ) != angle2MaxHmap.end() ) {
//       if( angle2MaxHmap[int(angle)].z() < point.z){
//         angle2MaxHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//       }
//     }else{
//       angle2MaxHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//     }

//     // select low z point 
//     if( angle2LowHmap.find( int(angle) ) != angle2LowHmap.end()){
//       if( angle2LowHmap[int(angle)].z() > point.z){
//         angle2LowHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//       }
//     }else{
//       angle2LowHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//     } 
//   }

//   std::stringstream ss;
//   for(auto one : angle2MaxHmap){
//     ss<<"angle: "<<one.first<<" point: "<<one.second.x()<<"-"<<one.second.y()<<"-"<<one.second.z();
//   }
//   for(auto one : angle2LowHmap){
//     ss<<"angle: "<<one.first<<" point: "<<one.second.x()<<"-"<<one.second.y()<<"-"<<one.second.z();
//   }
//   // LOG(INFO)<<" tracing tgt point: "<<ss.str();

//   LOG(INFO)<<" idx "<<idx<<" all point numbers "<<angle2MaxHmap.size() + angle2LowHmap.size();

//   return angle2MaxHmap.size() + angle2LowHmap.size();
// }
// //true: block; 
// bool ray_tracing::ray_tracing_check(const CloudPtr& map, const Eigen::Matrix4d& pose, uint idx){

//   if( map->points.empty()){
//     LOG(INFO)<<"  make_voxel_map filed : no points in map cloud ";
//     return false;
//   }

//   Eigen::Vector3d origin(pose(0,3)-200, pose(1,3)-200, pose(2,3)-10 );
//   std::unordered_map<int, Eigen::Vector3d> angle2MaxHmap;
//   std::unordered_map<int, Eigen::Vector3d> angle2LowHmap;
//   for(auto point : map->points){
//     double x = point.x - pose(0,3);
//     double y = point.y - pose(1,3);
    
//     double angle = std::atan2( y, x) * 180 / M_PI;
//     // select max z point
//     if( angle2MaxHmap.find( int(angle) ) != angle2MaxHmap.end() ) {
//       if( angle2MaxHmap[int(angle)].z() < point.z){
//         angle2MaxHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//       }
//     }else{
//       angle2MaxHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//     }

//     // select low z point 
//     if( angle2LowHmap.find( int(angle) ) != angle2LowHmap.end()){
//       if( angle2LowHmap[int(angle)].z() > point.z){
//         angle2LowHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//       }
//     }else{
//       angle2LowHmap[int(angle)] = Eigen::Vector3d(point.x, point.y, point.z);
//     } 
//   }

//   std::stringstream ss;
//   for(auto one : angle2MaxHmap){
//     ss<<"angle: "<<one.first<<" point: "<<one.second.x()<<"-"<<one.second.y()<<"-"<<one.second.z();
//   }
//   for(auto one : angle2LowHmap){
//     ss<<"angle: "<<one.first<<" point: "<<one.second.x()<<"-"<<one.second.y()<<"-"<<one.second.z();
//   }
//   LOG(INFO)<<" tracing tgt point: "<<ss.str();

//   CloudPtr output(new PointCloudType);

//   int count =0 ;
//   for( auto one : angle2MaxHmap){
//     if( tracing(one.second, pose, output) ){
//       count++;
//     }
//   }
//   for( auto one : angle2LowHmap){
//     if( tracing(one.second, pose, output) ){
//       count++;
//     }
//   }
//   output->width = output->points.size();
//   output->height = 1;
//   std::string filename_out = out_dir + std::to_string( idx) +".pcd";
//   pcl::io::savePCDFileASCII(filename_out, *output);

//   LOG(INFO)<<" idx "<<idx<<" ray tracing block count "<<count
//           <<" all point numbers "<<angle2MaxHmap.size() + angle2LowHmap.size();

//   if( count == angle2MaxHmap.size() + angle2LowHmap.size()){
//     return true;
//   }

//   return false;
// }

// // true: block;  
// bool ray_tracing::tracing(Eigen::Vector3d& point, const Eigen::Matrix4d& pose, CloudPtr output){

//   int delta_x = (point.x() - pose(0,3))/voxel_length;
//   int delta_y = (point.y() - pose(1,3))/voxel_length;
//   int delta_z = (point.z() - pose(2,3))/voxel_length;

//   int cur_x = (pose(0,3) - origin.x())/voxel_length;
//   int cur_y = (pose(1,3) - origin.y())/voxel_length;
//   int cur_z = (pose(2,3) - origin.z())/voxel_length;

//   double step_x =  1./delta_x;
//   double step_y =  1./delta_y;
//   double step_z =  1./delta_z;

//   double x_max = step_x; 
//   double y_max = step_y; 
//   double z_max = step_z; 

//   std::stringstream ss;
//   int block =0;
//   while ( abs(x_max) < 1 || abs(y_max) < 1 || abs(z_max) < 1){
//     if( abs(x_max) < abs(y_max)){
//       if( abs(x_max) < abs(z_max)){
//         x_max += step_x;
//         step_x > 0 ? cur_x += 1 : cur_x -= 1;
        
//         VOXEL_LOC position(cur_x, cur_y, cur_z);

//         PointType point;
//         point.x = cur_x * voxel_length - 200 + pose(0,3); 
//         point.y = cur_y * voxel_length - 200 + pose(1,3); 
//         point.z = cur_z * voxel_length - 10 + pose(2,3); 
//         output->points.push_back(point);

//         if( voxel_map.find(position) != voxel_map.end() && voxel_map[position]->num >5 
//                             && 1 - abs(x_max) > abs(step_x) ){
//           block++;
//           ss<<"-"<<voxel_map[position]->num;
//           output->points.push_back(point);
//         }
//       }else{
//         z_max += step_z;
//         step_z > 0 ? cur_z += 1 : cur_z -= 1;
        
//         VOXEL_LOC position(cur_x, cur_y, cur_z);

//         PointType point;
//         point.x = cur_x * voxel_length - 200 + pose(0,3); 
//         point.y = cur_y * voxel_length - 200 + pose(1,3); 
//         point.z = cur_z * voxel_length - 10 + pose(2,3); 
//         output->points.push_back(point);
        
//         if( voxel_map.find(position) != voxel_map.end() && voxel_map[position]->num >5 
//                             && 1 - abs(z_max) > abs(step_z) ){
//           block++;
//           ss<<"-"<<voxel_map[position]->num;
//         }
//       }
//     }else{
//       if( abs(y_max) < abs(z_max) ){
//         y_max += step_y;
//         step_y > 0 ? cur_y += 1 : cur_y -=1 ;
        
//         VOXEL_LOC position(cur_x, cur_y, cur_z);

//         PointType point;
//         point.x = cur_x * voxel_length - 200 + pose(0,3); 
//         point.y = cur_y * voxel_length - 200 + pose(1,3); 
//         point.z = cur_z * voxel_length - 10 + pose(2,3); 
//         output->points.push_back(point);
        
//         if( voxel_map.find(position) != voxel_map.end() &&  voxel_map[position]->num >5 
//                             && 1 - abs(y_max) > abs(step_y) ){
//           block++;
//           ss<<"-"<<voxel_map[position]->num;
//         }
//       }else{
//         z_max += step_z;
//         step_z > 0 ? cur_z += 1 : cur_z -= 1;
        
//         VOXEL_LOC position(cur_x, cur_y, cur_z);

//         PointType point;
//         point.x = cur_x * voxel_length - 200 + pose(0,3); 
//         point.y = cur_y * voxel_length - 200 + pose(1,3); 
//         point.z = cur_z * voxel_length - 10 + pose(2,3); 
//         output->points.push_back(point);
        
//         if( voxel_map.find(position) != voxel_map.end() && voxel_map[position]->num >5 
//                             && 1 - abs(z_max) > abs(step_z) ){
//           block++;
//           ss<<"-"<<voxel_map[position]->num;
//         }
//       }
//     }
//   }

//   LOG(INFO)<<" block "<<block<<" num "<<ss.str()<<" points.size "<<output->points.size();
//   return block>0;
// }

// }
