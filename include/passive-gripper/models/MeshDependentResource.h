#pragma once

#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>

#include "../Constants.h"
#include "../Debugger.h"
#include "../serialization/Serialization.h"

namespace psg {

class MeshDependentResource : psg::serialization::Serializable {
 public:
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd FN;
  Eigen::MatrixXd VN;
  Eigen::MatrixXd EN;
  Eigen::MatrixXi E;
  Eigen::MatrixXi EMAP;
  Eigen::Vector3d center_of_mass;
  Eigen::Vector3d minimum;
  Eigen::Vector3d maximum;
  Eigen::Vector3d size;
  igl::AABB<Eigen::MatrixXd, 3> tree;
  igl::embree::EmbreeIntersector intersector;

  // curvature
  Eigen::MatrixXd PD1, PD2;
  Eigen::VectorXd PV1, PV2;

 private:
  // All-pair shortest path
  // A proxy for geodesic distance
  mutable bool SP_valid_ = false;
  mutable Eigen::MatrixXd SP_;
  mutable Eigen::MatrixXi SP_par_;
  mutable std::mutex SP_mutex_;
  void init_sp() const;

  // Curvature
  /*
  mutable bool curvature_valid_ = false;
  mutable Eigen::VectorXd curvature_;
  mutable std::mutex curvature_mutex_;
  void init_curvature() const;
  */

  bool initialized = false;

 public:
  void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  void init(const MeshDependentResource& other);

  // out_c: closest point
  // out_s: sign
  double ComputeSignedDistance(const Eigen::Vector3d& position,
                               Eigen::RowVector3d& out_c,
                               double& out_s) const;

  void ComputeClosestPoint(const Eigen::Vector3d& position,
                           Eigen::RowVector3d& out_c,
                           int& out_fid) const;

  size_t ComputeClosestFacet(const Eigen::Vector3d& position) const;

  size_t ComputeClosestVertex(const Eigen::Vector3d& position) const;

  // Returns the length of non-intersecting path from A to B
  // minus the displacement from A to B.
  double ComputeRequiredDistance(const Eigen::Vector3d& A,
                                 const Eigen::Vector3d& B,
                                 Debugger* const debugger) const;

  bool Intersects(const Eigen::AlignedBox3d box) const;

  // Getters
  const Eigen::MatrixXd& GetSP() const;
  const Eigen::MatrixXi& GetSPPar() const;
  // const Eigen::VectorXd& GetCurvature() const;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(V);
    SERIALIZE(F);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    Eigen::MatrixXd V_;
    Eigen::MatrixXi F_;
    if (version == 1) {
      DESERIALIZE(V_);
      DESERIALIZE(F_);
      init(V_, F_);
    }
  }
};

}  // namespace psg
