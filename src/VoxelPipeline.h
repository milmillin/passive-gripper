#pragma once

#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>
#include <atomic>
#include <string>
#include <vector>

#include "ContactPoint.h"
#include "Gripper.h"
#include "MainUI.h"
#include "VoxelPipelineSettings.h"
#include "Utils.h"

namespace gripper {

class MainUI;

class VoxelPipeline {
 public:
  VoxelPipeline(MainUI* mainUI,
                const Eigen::MatrixXd& mesh_V,
                const Eigen::MatrixXi& mesh_F);

  void UpdateSettings(const VoxelPipelineSettings& settings,
                      bool isInit = false);

  void RunPerformanceTest(VoxelPipelineSettings settings, int lo, int hi, int step);

  inline void WriteDXF(const std::string& filename) const {
    m_gripper.WriteDXF(filename);
  }
  inline void WriteRAPID(const std::string& filename) const {
    m_gripper.WriteRAPID(filename);
  }

  inline const Gripper& GetGripper() const { return m_gripper; }

  void WriteResult(const std::string& filename) const;
  void WriteGripper(const std::string& filename) const;

  inline bool IsReady() { return m_isReady; }

 private:
  void GenerateRotatedMesh(const VoxelPipelineSettings& settings);
  void GenerateOffsetMesh(const VoxelPipelineSettings& settings);
  void CalculateCenterOfMass(const VoxelPipelineSettings& settings);

  void FindTypeAContactPoints(const VoxelPipelineSettings& settings);
  void FindTypeBContactPoints(const VoxelPipelineSettings& settings);

  void FilterFeasibleContactPoints(const VoxelPipelineSettings& settings);

  void FindBestContactPoints(const VoxelPipelineSettings& settings);
  bool CheckManufacturingConstraint(const VoxelPipelineSettings& settings,
                                    const ContactPoint& a,
                                    const ContactPoint& b,
                                    const ContactPoint& c);

  void ExtendTypeAContact(const VoxelPipelineSettings& settings);

  void SetViewerData();

  void GenerateContactPoints();
  static void GeneratePoints(const std::vector<ContactPoint>& points,
                             Eigen::MatrixXd& out_P,
                             Eigen::MatrixXd& out_PC);

  MainUI* m_mainUI;
  std::atomic<bool> m_isReady;
  VoxelPipelineSettings m_settings;

  Eigen::MatrixXd m_mesh_V;
  Eigen::MatrixXi m_mesh_F;
  MeshInfo m_meshInfo;
  igl::embree::EmbreeIntersector m_mesh_intersector;
  Eigen::Vector3d m_centerOfMass;

  Eigen::MatrixXd m_offset_mesh_V;
  Eigen::MatrixXi m_offset_mesh_F;
  Eigen::MatrixXd m_offset_mesh_N;
  MeshInfo m_offset_meshInfo;
  igl::embree::EmbreeIntersector m_offset_mesh_intersector;

  // rotate so that gripper comes in -x direction
  Eigen::MatrixXd m_rotated_mesh_V;
  MeshInfo m_rotated_meshInfo;
  Eigen::Affine3d m_rotation;

  std::vector<ContactPoint> m_contactPoints;
  std::vector<ContactPoint> m_filteredContactPoints;
  std::vector<ContactPoint> m_bestContactPoints;
  std::vector<ContactPoint> m_rawBestContactPoints;

  int m_bestNumA;
  double m_bestAngle;

  Eigen::MatrixXd m_contactPointsA_P;
  Eigen::MatrixXd m_contactPointsB_P;
  Eigen::MatrixXd m_filteredContactPoints_P;
  Eigen::MatrixXd m_filteredContactPoints_PC;
  Eigen::MatrixXd m_bestContactPoints_P;
  Eigen::MatrixXd m_bestContactPoints_PC;

  Eigen::MatrixXd m_contactRay_P;
  Eigen::MatrixXi m_contactRay_E;

  Gripper m_gripper;
};

}  // namespace gripper