#include "VoxelPipeline.h"

namespace gripper {

VoxelPipeline::VoxelPipeline(
  MainUI* mainUI,
  const Eigen::MatrixXd& mesh_V,
  const Eigen::MatrixXi& mesh_F) :
  m_mainUI(mainUI),
  m_isReady(false),
  m_mesh_V(mesh_V),
  m_mesh_F(mesh_F)
{ }

// Called from background thread
void VoxelPipeline::UpdateSettings(const VoxelPipelineSettings& settings, bool isInit)
{
  static Eigen::RowVector3d pointColor(1, 1, 1);

  m_isReady = false;
  bool requireUpdate = false;

  // Voxelize
  if (isInit || settings.numDivision != m_settings.numDivision) {
    Voxels::Voxelize(m_mesh_V, m_mesh_F, settings.numDivision, m_voxels, m_allCoords);

    // Filter Supporting Voxels
    m_supportingCoords = m_voxels.FilterSupportingVoxels(m_allCoords);

    requireUpdate = true;
  }
  
  // Filter Grab Direction
  if (isInit || requireUpdate || settings.grabDirection != m_settings.grabDirection) {
    m_filteredCoords = m_voxels.FilterGrabDirection(m_supportingCoords, m_mesh_V, m_mesh_F, settings.grabDirection);

    requireUpdate = true;
  }

  // Compute Mesh
  if (isInit || requireUpdate || settings.voxelScale != m_settings.voxelScale) {
    m_voxels.GenerateMesh(m_allCoords, settings.voxelScale, m_all_V, m_all_F);
    m_voxels.GenerateMesh(m_supportingCoords, settings.voxelScale, m_supporting_V, m_supporting_F);
    m_voxels.GenerateMesh(m_filteredCoords, settings.voxelScale, m_filtered_V, m_filtered_F);
    
    requireUpdate = true;
  }

  // Compute Points
  if (isInit || requireUpdate) {
    m_voxels.GeneratePoints(m_allCoords, m_all_P);
    m_voxels.GeneratePoints(m_allCoords, m_supporting_P);
    m_voxels.GeneratePoints(m_allCoords, m_filtered_P);
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
    setData(data_all, m_all_V, m_all_F, m_all_V);
    
    igl::opengl::ViewerData& data_supporting = m_mainUI->GetViewerData(LayerId::VoxelSupporting);
    setData(data_supporting, m_supporting_V, m_supporting_F, m_supporting_V);

    igl::opengl::ViewerData& data_filtered = m_mainUI->GetViewerData(LayerId::VoxelFiltered);
    setData(data_filtered, m_filtered_V, m_filtered_F, m_filtered_V);

    m_mainUI->viewerDataMutex.unlock();
  }

  // Done
  m_settings = settings;
  m_isReady = true;
}

}