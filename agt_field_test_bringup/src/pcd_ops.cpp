#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <iostream>
namespace fs=std::filesystem;
int main(int argc,char **argv) {
 try {
  if(argc!=4) throw std::runtime_error("usage: pcd_ops input.pcd operations.yaml new-output-directory");
  fs::path out(argv[3]); if(fs::exists(out)) throw std::runtime_error("output exists; immutable artifacts required");
  auto spec=YAML::LoadFile(argv[2]);
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  if(pcl::io::loadPCDFile(argv[1],*cloud)<0) throw std::runtime_error("PCD load failed");
  const auto input_count=cloud->width*cloud->height;
  for(const auto &op:spec["operations"]) {
   std::string type=op["type"].as<std::string>();
   pcl::PCLPointCloud2::Ptr next(new pcl::PCLPointCloud2);
   if(type=="crop" || type=="delete") {
    auto lo=op["min"].as<std::vector<float>>(), hi=op["max"].as<std::vector<float>>();
    if(lo.size()!=3 || hi.size()!=3) throw std::runtime_error("ROI needs three bounds");
    for(int i=0;i<3;++i) if(!std::isfinite(lo[i])||!std::isfinite(hi[i])||lo[i]>=hi[i]) throw std::runtime_error("invalid ROI");
    pcl::CropBox<pcl::PCLPointCloud2> f; f.setInputCloud(cloud);
    f.setMin(Eigen::Vector4f(lo[0],lo[1],lo[2],1)); f.setMax(Eigen::Vector4f(hi[0],hi[1],hi[2],1));
    f.setNegative(type=="delete"); f.filter(*next);
   } else if(type=="voxel") {
    float leaf=op["size"].as<float>(); if(!std::isfinite(leaf)||leaf<=0) throw std::runtime_error("invalid voxel size");
    pcl::VoxelGrid<pcl::PCLPointCloud2> f; f.setInputCloud(cloud); f.setLeafSize(leaf,leaf,leaf);f.setDownsampleAllData(true);f.filter(*next);
   } else throw std::runtime_error("unknown operation");
   cloud=next;
  }
  if(cloud->width*cloud->height==0) throw std::runtime_error("operation removed all points");
  fs::create_directories(out);
  if(pcl::PCDWriter().writeBinary((out/"cloud.pcd").string(),*cloud)<0) throw std::runtime_error("PCD write failed");
  if(spec["projection"]) {
   const auto p=spec["projection"];double res=p["resolution"].as<double>();
   auto origin=p["origin"].as<std::vector<double>>();auto range=p["range"].as<std::vector<double>>();
   double zmin=p["z_min"].as<double>(),zmax=p["z_max"].as<double>();int threshold=p["min_points"].as<int>();
   if(!std::isfinite(res)||res<=0||origin.size()!=3||range.size()!=2||threshold<1||zmin>=zmax) throw std::runtime_error("invalid projection");
   for(double x:origin) if(!std::isfinite(x)) throw std::runtime_error("invalid origin");
   int w=static_cast<int>(std::ceil(range[0]/res)),h=static_cast<int>(std::ceil(range[1]/res));
   if(w<=0||h<=0||static_cast<int64_t>(w)*h>16000000) throw std::runtime_error("grid exceeds 16 million cells");
   std::vector<int> hits(w*h,0);std::vector<unsigned char> seen(w*h,0),pixels(w*h,205);
   pcl::PointCloud<pcl::PointXYZ> xyz; pcl::fromPCLPointCloud2(*cloud,xyz);
   double c=std::cos(origin[2]),s=std::sin(origin[2]);
   for(const auto &point:xyz) {
    if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) continue;
    double dx=point.x-origin[0],dy=point.y-origin[1];
    int x=static_cast<int>(std::floor((c*dx+s*dy)/res)),y=static_cast<int>(std::floor((-s*dx+c*dy)/res));
    if(x<0||y<0||x>=w||y>=h) continue; int idx=(h-1-y)*w+x;
    if(point.z>=p["ground_min"].as<double>()&&point.z<zmin) seen[idx]=1;
    if(point.z>=zmin&&point.z<=zmax) ++hits[idx];
   }
   for(int i=0;i<w*h;++i) if(hits[i]>=threshold) pixels[i]=0; else if(seen[i]) pixels[i]=254;
   std::ofstream pgm(out/"map.pgm",std::ios::binary);pgm<<"P5\n"<<w<<" "<<h<<"\n255\n";pgm.write(reinterpret_cast<char*>(pixels.data()),pixels.size());
   YAML::Node map;map["image"]="map.pgm";map["resolution"]=res;map["origin"]=origin;map["negate"]=0;
   map["occupied_thresh"]=p["occupied_thresh"].as<double>();map["free_thresh"]=p["free_thresh"].as<double>();
   std::ofstream(out/"map.yaml")<<map;
  }
  YAML::Node result;result["input_points"]=input_count;result["output_points"]=cloud->width*cloud->height;result["processed_full_resolution"]=true;
  pcl::PointCloud<pcl::PointXYZ> points; pcl::fromPCLPointCloud2(*cloud,points);
  std::vector<double> low(3,INFINITY),high(3,-INFINITY);
  for(const auto &pt:points) { double v[]={pt.x,pt.y,pt.z}; if(!std::isfinite(v[0])||!std::isfinite(v[1])||!std::isfinite(v[2]))continue;
   for(int i=0;i<3;++i){low[i]=std::min(low[i],v[i]);high[i]=std::max(high[i],v[i]);}}
  if(!std::isfinite(low[0]))throw std::runtime_error("cloud has no finite points");
  result["bounds_min"]=low;result["bounds_max"]=high;
  std::ofstream(out/"result.yaml")<<result;
  return 0;
 } catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;return 1;}
}
