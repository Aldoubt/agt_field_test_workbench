#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <fstream>
#include <cmath>
namespace fs=std::filesystem;
class Artifact : public rclcpp::Node {
 public:
 Artifact():Node("fastlio_artifact") {
  directory_=declare_parameter<std::string>("output_dir");frame_=declare_parameter<std::string>("world_frame","lidar");
  if(fs::exists(directory_/"map_package")||fs::exists(directory_/"points.raw")||fs::exists(directory_/"trajectory.csv")) throw std::runtime_error("artifact exists");
  fs::create_directories(directory_);
  spool_.open(directory_/"points.raw",std::ios::binary|std::ios::trunc);
  poses_.open(directory_/"trajectory.csv");poses_<<"stamp,x,y,z,qx,qy,qz,qw\n";poses_.precision(17);
  cloud_=create_subscription<sensor_msgs::msg::PointCloud2>("/fastlio2/world_cloud",rclcpp::QoS(20),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
   if(done_) return;
   if(m->header.frame_id!=frame_) {error_="unexpected cloud world frame";return;}
   try {
    sensor_msgs::PointCloud2ConstIterator<float> x(*m,"x"),y(*m,"y"),z(*m,"z");
    for(;x!=x.end();++x,++y,++z) if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)) {
     float v[]={*x,*y,*z};spool_.write(reinterpret_cast<char*>(v),sizeof(v));++points_;
    }
    ++scans_;
   }catch(const std::exception &e){error_=e.what();}
  });
  odom_=create_subscription<nav_msgs::msg::Odometry>("/fastlio2/lio_odom",rclcpp::QoS(50),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
   if(done_)return;const auto &p=m->pose.pose.position;const auto &q=m->pose.pose.orientation;
   poses_<<rclcpp::Time(m->header.stamp).seconds()<<","<<p.x<<","<<p.y<<","<<p.z<<","<<q.x<<","<<q.y<<","<<q.z<<","<<q.w<<"\n";++poses_count_;
  });
  export_=create_service<std_srvs::srv::Trigger>("/mapping/fastlio/export",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){
   if(done_||!points_||!poses_count_||!error_.empty()){r->success=false;r->message="empty, already exported, or invalid: "+error_;return;}
   done_=true;spool_.flush();poses_.flush();
   auto out=directory_/"map_package";fs::create_directory(out);
   std::ofstream pcd(out/"map.pcd",std::ios::binary);
   pcd<<"# FAST-LIO2 world scans; no PGO\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "<<points_<<"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "<<points_<<"\nDATA binary\n";
   std::ifstream input(directory_/"points.raw",std::ios::binary);pcd<<input.rdbuf();pcd.close();
   fs::copy_file(directory_/"trajectory.csv",out/"trajectory.csv");
   r->success=pcd.good();r->message="FASTLIO_NO_PGO scans="+std::to_string(scans_)+" points="+std::to_string(points_);
  });
 }
 private:
 fs::path directory_;std::string frame_,error_;std::ofstream spool_,poses_;bool done_{false};uint64_t points_{0},scans_{0},poses_count_{0};
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Artifact>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}rclcpp::shutdown();}
