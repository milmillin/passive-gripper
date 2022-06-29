#pragma once

#include <Eigen/Core>
#include <mutex>
#include <vector>

namespace psg {

class Debugger {
 public:
  Debugger();
  void AddEdge(const Eigen::Vector3d& p0,
               const Eigen::Vector3d& p1,
               const Eigen::Vector3d& color);
  void AddCube(const Eigen::Vector3d& org, const Eigen::Vector3d& size);
  void AddPoint(const Eigen::Vector3d& p, const Eigen::Vector3d& color);

  void Clear();

  void GetEdges(Eigen::MatrixXd& out_P,
                Eigen::MatrixXi& out_E,
                Eigen::MatrixXd& out_C) const;

  void GetMesh(Eigen::MatrixXd& out_V, Eigen::MatrixXi& out_F) const;
  void GetPoints(Eigen::MatrixXd& out_P, Eigen::MatrixXd& out_C) const;

 private:
  Eigen::MatrixXd P_;
  size_t n_P_;
  Eigen::MatrixXi E_;
  size_t n_E_;
  Eigen::MatrixXd C_;
  size_t n_C_;
  Eigen::MatrixXd V_;
  size_t n_V_;
  Eigen::MatrixXi F_;
  size_t n_F_;
  Eigen::MatrixXd PP_;
  size_t n_PP_;
  Eigen::MatrixXd PC_;
  size_t n_PC_;
  mutable std::mutex mtx_;
};

}  // namespace psg
