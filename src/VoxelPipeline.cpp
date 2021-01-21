#include "VoxelPipeline.h"

#include <igl/offset_surface.h>

#include "Geometry.h"
#include "Gripper.h"

namespace gripper {

VoxelPipeline::VoxelPipeline(
  MainUI* mainUI,
  const Eigen::MatrixXd& mesh_V,
  const Eigen::MatrixXi& mesh_F) :
  m_mainUI(mainUI),
  m_isReady(false),
  m_mesh_V(mesh_V),
  m_mesh_F(mesh_F),
  m_meshInfo(mesh_V, mesh_F)
{ }

// Called from background thread
void VoxelPipeline::UpdateSettings(const VoxelPipelineSettings& settings, bool isInit)
{
  static Eigen::RowVector3d pointColor(1, 1, 1);
  static Eigen::RowVector3d centerOfMassColor(1, 0.5, 1);

  m_isReady = false;
  bool requireUpdate = false;

  // Offset Mesh
  if (isInit || settings.rodDiameter != m_settings.rodDiameter) {
    Eigen::MatrixXd GV;
    Eigen::Matrix<double, -1, 1> S;
    Eigen::RowVector3i side;

    igl::offset_surface(
      m_mesh_V,
      m_mesh_F,
      m_settings.rodDiameter / 2,
      30,
      igl::SignedDistanceType::SIGNED_DISTANCE_TYPE_DEFAULT,
      m_offset_mesh_V,
      m_offset_mesh_F,
      GV, side, S
    );

    m_offset_meshInfo = MeshInfo(m_offset_mesh_V, m_offset_mesh_F);
  }

  // Voxelize
  if (isInit || settings.voxelSize != m_settings.voxelSize) {
    Voxels::Voxelize(m_offset_mesh_V, m_offset_mesh_F, settings.voxelSize, m_voxels, m_allCoords);

    // Filter Supporting Voxels
    m_supportingCoords = m_voxels.FilterSupportingVoxels(m_allCoords, m_meshInfo.minimum.y());
    // Calculate Center of Mass
    m_centerOfMass = m_voxels.GetCenterOfMass(m_allCoords);

    requireUpdate = true;
  }

  // Filter Grab Direction
  if (isInit || requireUpdate || settings.grabAngle != m_settings.grabAngle) {
    Eigen::Vector3f grabDirection = -GetDirectionFromAngle(settings.grabAngle);
    m_filteredCoords = m_voxels.FilterGrabDirection(
      m_supportingCoords,
      m_offset_mesh_V,
      m_offset_mesh_F,
      grabDirection
    );

    requireUpdate = true;
  }

  // Solve Best Contact
  if (isInit || requireUpdate ||
    settings.findBestContact != m_settings.findBestContact ||
    settings.rodDiameter != m_settings.rodDiameter)
  {
    if (settings.findBestContact) m_bestCoords = FindBestContactDumb(m_filteredCoords, m_centerOfMass);
    else m_bestCoords.clear();

    std::vector<Eigen::Vector3d> contactPoints = RefineContactPoint(
      m_mesh_V, m_mesh_F,
      m_voxels, m_bestCoords, settings.rodDiameter
    );

    // Generate Gripper
    m_gripper = Gripper(
      m_mesh_V, m_mesh_F,
      contactPoints, settings.rodDiameter, settings.grabAngle
    );

    requireUpdate = true;
  }


  if (isInit || requireUpdate) {
    // Compute Points
    m_voxels.GeneratePoints(m_allCoords, m_all_P);
    m_voxels.GeneratePoints(m_supportingCoords, m_supporting_P);
    m_voxels.GeneratePoints(m_filteredCoords, m_filtered_P);
    m_voxels.GeneratePoints(m_bestCoords, m_best_P);
  }

  // Compute Mesh
  if (isInit || requireUpdate || settings.voxelScale != m_settings.voxelScale) {
    m_voxels.GenerateMesh(m_allCoords, settings.voxelScale, m_all_V, m_all_F);
    m_voxels.GenerateMesh(m_supportingCoords, settings.voxelScale, m_supporting_V, m_supporting_F);
    m_voxels.GenerateMesh(m_filteredCoords, settings.voxelScale, m_filtered_V, m_filtered_F);
    m_voxels.GenerateMesh(m_bestCoords, settings.voxelScale, m_best_V, m_best_F);

    requireUpdate = true;
  }

  requireUpdate = requireUpdate || settings.showAsPoint != m_settings.showAsPoint;

  // Set Viewer Mesh
  auto setData = [&settings](igl::opengl::ViewerData& data,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& P)
  {
    data.clear();
    if (settings.showAsPoint) {
      data.set_points(P, pointColor);
    }
    else {
      data.set_face_based(true);
      data.set_mesh(V, F);
    }
  };

  if (isInit || requireUpdate) {
    m_mainUI->viewerDataMutex.lock();

    igl::opengl::ViewerData& data_all = m_mainUI->GetViewerData(LayerId::VoxelAll);
    setData(data_all, m_all_V, m_all_F, m_all_P);

    igl::opengl::ViewerData& data_supporting = m_mainUI->GetViewerData(LayerId::VoxelSupporting);
    setData(data_supporting, m_supporting_V, m_supporting_F, m_supporting_P);

    igl::opengl::ViewerData& data_filtered = m_mainUI->GetViewerData(LayerId::VoxelFiltered);
    setData(data_filtered, m_filtered_V, m_filtered_F, m_filtered_P);

    igl::opengl::ViewerData& data_best = m_mainUI->GetViewerData(LayerId::VoxelBest);
    setData(data_best, m_best_V, m_best_F, m_best_P);
    data_best.add_points(m_voxels.GetVoxelCenter<double>(m_centerOfMass).transpose(), centerOfMassColor);

    igl::opengl::ViewerData& data_gripper = m_mainUI->GetViewerData(LayerId::GripperMesh);
    data_gripper.clear();
    data_gripper.set_face_based(true);
    data_gripper.set_mesh(m_gripper.V(), m_gripper.F());

    igl::opengl::ViewerData& data_offset = m_mainUI->GetViewerData(LayerId::Offset);
    data_offset.clear();
    data_offset.set_mesh(m_offset_mesh_V, m_offset_mesh_F);

    m_mainUI->viewerDataMutex.unlock();
  }

  // Done
  m_settings = settings;
  m_isReady = true;
}

}