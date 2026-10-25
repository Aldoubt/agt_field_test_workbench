// Research adaptation of gisbi-kim/scancontext. See NOTICE.md (CC BY-NC-SA 4.0).
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/io/pcd_io.h>
#include "Scancontext.h"
namespace py = pybind11;

class StaticMapIndex {
  SCManager manager;
  std::unique_ptr<InvKeyTree> tree;
public:
  Eigen::MatrixXd descriptor(const Eigen::MatrixXd& points) {
    if (points.cols()!=3 || points.rows()==0 || !points.allFinite()) throw std::runtime_error("finite Nx3 cloud required");
    pcl::PointCloud<SCPointType> cloud;
    for (int i=0;i<points.rows();++i) {
      if (std::hypot(points(i,0),points(i,1))<1e-6) continue;
      SCPointType p; p.x=points(i,0);p.y=points(i,1);p.z=points(i,2);p.intensity=0;
      cloud.push_back(p);
    }
    return manager.makeScancontext(cloud);
  }
  void load(const std::vector<Eigen::MatrixXd>& descriptions) {
    if (descriptions.empty()) throw std::runtime_error("empty SC database");
    manager.polarcontexts_=descriptions;manager.polarcontext_invkeys_mat_.clear();
    for (auto d:descriptions) {
      if (d.rows()!=20 || d.cols()!=60 || !d.allFinite()) throw std::runtime_error("invalid SC matrix");
      manager.polarcontext_invkeys_mat_.push_back(eig2stdvec(manager.makeRingkeyFromScancontext(d)));
    }
    tree=std::make_unique<InvKeyTree>(20,manager.polarcontext_invkeys_mat_,10);
  }
  std::vector<std::tuple<int,double,double>> query(Eigen::MatrixXd query,int top_k) {
    if (!tree || top_k<1 || query.rows()!=20 || query.cols()!=60 || !query.allFinite()) throw std::runtime_error("invalid SC query/index");
    const size_t n=std::min<size_t>(10,manager.polarcontexts_.size());
    auto key=eig2stdvec(manager.makeRingkeyFromScancontext(query));
    std::vector<size_t> ids(n);std::vector<float> distances(n);
    nanoflann::KNNResultSet<float> nearest(n);nearest.init(ids.data(),distances.data());
    tree->index->findNeighbors(nearest,key.data(),nanoflann::SearchParams(10));
    std::vector<std::tuple<int,double,double>> out;
    for (size_t i=0;i<n;++i) {
      auto d=manager.polarcontexts_.at(ids[i]);auto match=manager.distanceBtnScanContext(query,d);
      if (std::isfinite(match.first)) out.emplace_back(ids[i],match.first,match.second*2.*M_PI/60.);
    }
    std::sort(out.begin(),out.end(),[](const auto&a,const auto&b){return std::get<1>(a)<std::get<1>(b);});
    if (out.size()>size_t(top_k))out.resize(top_k);
    return out;
  }
};
PYBIND11_MODULE(_scancontext,m) {
  m.attr("upstream_commit")="93672835bb85e9c03fedf0fbaf2d0491df12161e";
  py::class_<StaticMapIndex>(m,"Index").def(py::init<>())
    .def("descriptor",&StaticMapIndex::descriptor,py::call_guard<py::gil_scoped_release>())
    .def("load",&StaticMapIndex::load,py::call_guard<py::gil_scoped_release>())
    .def("query",&StaticMapIndex::query,py::call_guard<py::gil_scoped_release>());
  m.def("load_pcd",[](const std::string&path){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPCDFile(path,cloud)!=0)throw std::runtime_error("cannot read PCD: "+path);
    std::vector<Eigen::Vector3d> points;
    for(const auto&p:cloud)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))points.emplace_back(p.x,p.y,p.z);
    Eigen::MatrixXd out(points.size(),3);for(size_t i=0;i<points.size();++i)out.row(i)=points[i];return out;
  });
}
