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
  Eigen::MatrixXd V;               // Vertex list
  Eigen::MatrixXi F;               // Face list
  Eigen::MatrixXd FN;              // Per-face normals
  Eigen::MatrixXd VN;              // Per-vertex normals
  Eigen::MatrixXd EN;              // Per-edge normals
  Eigen::MatrixXi E;               // Edge list
  Eigen::MatrixXi EMAP;            // EMAP returned by igl::per_edge_normals
  Eigen::Vector3d center_of_mass;  // Estimated center of mass
  Eigen::Vector3d minimum;         // Minimum coordinate of vertices
  Eigen::Vector3d maximum;         // Maxinum coordinate of vertices
  Eigen::Vector3d size;  // Maxinum coordinate minus minimum coordinate
  igl::AABB<Eigen::MatrixXd, 3> tree;  // AABB tree for the mesh
  igl::embree::EmbreeIntersector
      intersector;  // Embree data structure for the mesh

 private:
  // All-pair shortest path
  // A proxy for geodesic distance
  mutable bool SP_valid_ = false;
  mutable Eigen::MatrixXd SP_;
  mutable Eigen::MatrixXi SP_par_;
  mutable std::mutex SP_mutex_;
  void init_sp() const;

  bool initialized = false;

 public:
  /// <summary>
  /// Initialize MeshDependentResource
  /// </summary>
  /// <param name="V">Mesh vertices</param>
  /// <param name="F">Mesh faces</param>
  void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

  /// <summary>
  /// Initialize MeshDependentResource from another MeshDependentResource
  /// </summary>
  /// <param name="other">Another MeshDependentResource</param>
  void init(const MeshDependentResource& other);

  /// <summary>
  /// Compute the signed distance from a given point.
  /// </summary>
  /// <param name="position">Point</param>
  /// <param name="out_c">Closest point on the mesh</param>
  /// <param name="out_s">-1 or 1</param>
  /// <returns>The signed distance</returns>
  double ComputeSignedDistance(const Eigen::Vector3d& position,
                               Eigen::RowVector3d& out_c,
                               double& out_s) const;

  /// <summary>
  /// Compute the closest point on the mesh from a given point.
  /// </summary>
  /// <param name="position">Point</param>
  /// <param name="out_c">Closest point on the mesh</param>
  /// <param name="out_fid">Face index of the cloest point</param>
  void ComputeClosestPoint(const Eigen::Vector3d& position,
                           Eigen::RowVector3d& out_c,
                           int& out_fid) const;

  /// <summary>
  /// Compute the closest face on the mesh from a given point
  /// </summary>
  /// <param name="position">Point</param>
  /// <returns>Index of the face on which the cloest point lies</returns>
  size_t ComputeClosestFacet(const Eigen::Vector3d& position) const;

  /// <summary>
  /// Compute the closest vertex on the mesh from a given point
  /// </summary>
  /// <param name="position">Point</param>
  /// <returns>Index of the closest vertex</returns>
  size_t ComputeClosestVertex(const Eigen::Vector3d& position) const;

  // Returns the length of non-intersecting path from A to B
  // minus the displacement from A to B.
  double ComputeRequiredDistance(const Eigen::Vector3d& A,
                                 const Eigen::Vector3d& B,
                                 Debugger* const debugger) const;

  /// <summary>
  /// Determine whether the mesh intersects a given bounding box.
  /// </summary>
  /// <param name="box">Bounding box</param>
  /// <returns>Whether the mesh intersects the bounding box</returns>
  bool Intersects(const Eigen::AlignedBox3d box) const;

  // Getters
  const Eigen::MatrixXd& GetSP() const;
  const Eigen::MatrixXi& GetSPPar() const;

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
