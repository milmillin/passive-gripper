#include "VoxelPipeline.h"

#include <igl/offset_surface.h>
#include <omp.h>

#include "Geometry.h"
#include "Gripper.h"

namespace gripper {

VoxelPipeline::VoxelPipeline(MainUI* mainUI,
                             const Eigen::MatrixXd& mesh_V,
                             const Eigen::MatrixXi& mesh_F)
    : m_mainUI(mainUI),
      m_mesh_V(mesh_V),
      m_mesh_F(mesh_F),
      m_meshInfo(mesh_V, mesh_F) {
  m_mesh_intersector.init(mesh_V.cast<float>(), mesh_F, true);
}

// Called from background thread
void VoxelPipeline::UpdateSettings(const VoxelPipelineSettings& settings,
                                   bool isInit) {
  m_isReady = false;
  bool requireUpdate = false;

  // Calculate Center of Mass
  if (isInit || settings.voxelSize != m_settings.voxelSize) {
    CalculateCenterOfMass(settings);
    requireUpdate = true;
  }

  // Generate Offset Mesh
  if (isInit || requireUpdate ||
      settings.rodDiameter != m_settings.rodDiameter ||
      settings.epsilon != m_settings.epsilon) {
    GenerateOffsetMesh(settings);
    requireUpdate = true;
  }

  // Find Type A Contact
  if (isInit || requireUpdate || settings.grabAngle != m_settings.grabAngle) {
    FindTypeAContactPoints(settings);
    requireUpdate = true;
  }

  // Find Type B Contact
  if (isInit || requireUpdate ||
      settings.gridSpacing != m_settings.gridSpacing) {
    FindTypeBContactPoints(settings);
    requireUpdate = true;
  }

  // Filter feasible contact points
  if (isInit || requireUpdate) {
    FilterFeasibleContactPoints(settings);
  }

  // Solve Best Contact
  if (isInit || requireUpdate ||
      settings.findBestContact != m_settings.findBestContact) {
    if (settings.findBestContact) {
      FindBestContactPoints(settings);
    } else {
      m_bestContactPoints.clear();
    }

    // Generate Gripper
    m_gripper = Gripper(m_mesh_V, m_mesh_F, m_bestContactPoints, settings);

    requireUpdate = true;
  }

  if (isInit || requireUpdate) {
    // Compute Points
    GeneratePoints(m_contactPoints, m_contactPoints_P, m_contactPoints_PC);
    GeneratePoints(
        m_bestContactPoints, m_bestContactPoints_P, m_bestContactPoints_PC);

    // Visualize
    SetViewerData();
  }

  // Done
  m_settings = settings;
  m_isReady = true;
}

void VoxelPipeline::GenerateOffsetMesh(const VoxelPipelineSettings& settings) {
  Eigen::MatrixXd GV;
  Eigen::Matrix<double, -1, 1> S;
  Eigen::RowVector3i side;

  igl::offset_surface(m_mesh_V,
                      m_mesh_F,
                      settings.rodDiameter / 2. + settings.epsilon,
                      30,
                      igl::SignedDistanceType::SIGNED_DISTANCE_TYPE_DEFAULT,
                      m_offset_mesh_V,
                      m_offset_mesh_F,
                      GV,
                      side,
                      S);

  m_offset_meshInfo = MeshInfo(m_offset_mesh_V, m_offset_mesh_F);
  m_offset_mesh_intersector.init(m_offset_mesh_V.cast<float>(),
                                 m_offset_mesh_F);

  // Compute Normals
  m_offset_mesh_N.resize(m_offset_mesh_F.rows(), 3);

#pragma omp parallel for
  for (ssize_t i = 0; i < m_offset_mesh_F.rows(); i++) {
    int a = m_offset_mesh_F.coeff(i, 0);
    int b = m_offset_mesh_F.coeff(i, 1);
    int c = m_offset_mesh_F.coeff(i, 2);
    Eigen::RowVector3d ab = m_offset_mesh_V.row(b) - m_offset_mesh_V.row(a);
    Eigen::RowVector3d ac = m_offset_mesh_V.row(c) - m_offset_mesh_V.row(a);
    m_offset_mesh_N.row(i) = ab.cross(ac).normalized();
  }
}

void VoxelPipeline::CalculateCenterOfMass(
    const VoxelPipelineSettings& settings) {
  Eigen::Matrix<ssize_t, 3, 1> voxelCount =
      (m_meshInfo.size / settings.voxelSize).array().ceil().cast<ssize_t>();
  Eigen::RowVector3f voxelDimension =
      voxelCount.cast<float>() * (float)settings.voxelSize;

  // Make the object centered in the voxels space
  Eigen::RowVector3d origin =
      (m_meshInfo.minimum -
       (voxelDimension.transpose().cast<double>() - m_meshInfo.size) / 2.)
          .array() +
      (settings.voxelSize / 2);

  Eigen::Vector3d massContribution(0, 0, 0);
  ssize_t count = 0;

  // Calculate mass contribution of voxels inside the object
#pragma omp parallel
  {
    Eigen::Vector3d t_massContribution(0, 0, 0);
    ssize_t t_count = 0;
    vector<igl::Hit> t_hits;
    int t_numRays;
    Eigen::RowVector3f t_direction = Eigen::RowVector3f::UnitZ();

#pragma omp for nowait
    for (ssize_t i = 0; i < voxelCount.x(); i++) {
      for (ssize_t j = 0; j < voxelCount.y(); j++) {
        for (ssize_t k = 0; k < voxelCount.z(); k++) {
          Eigen::RowVector3d position =
              origin + Eigen::RowVector3d(i, j, k) * settings.voxelSize;

          m_mesh_intersector.intersectRay(
              position.cast<float>(), t_direction, t_hits, t_numRays);

          if (t_hits.size() % 2 == 1) {
            t_massContribution += position;
            t_count++;
          }
        }
      }
    }

#pragma omp critical
    {
      massContribution += t_massContribution;
      count += t_count;
    }
  }

  m_centerOfMass = massContribution / count;
}

void VoxelPipeline::FindTypeAContactPoints(
    const VoxelPipelineSettings& settings) {
  // Construct Edge List
  // edge (u, v) -> list of another vertex on a face; u < v.
  std::map<std::pair<int, int>, std::vector<int>> edges;
  for (ssize_t i = 0; i < m_offset_mesh_F.rows(); i++) {
    int a = m_offset_mesh_F.coeff(i, 0);
    int b = m_offset_mesh_F.coeff(i, 1);
    int c = m_offset_mesh_F.coeff(i, 2);
    edges[std::minmax(a, b)].push_back(c);
    edges[std::minmax(b, c)].push_back(a);
    edges[std::minmax(c, a)].push_back(b);
  }

  Eigen::Vector3d grabDirection =
      GetDirectionFromAngle(settings.grabAngle).cast<double>();

  // Iterate through edges
  for (const auto& e : edges) {
    Eigen::Vector3d u = m_offset_mesh_V.row(e.first.first).transpose();
    Eigen::Vector3d v = m_offset_mesh_V.row(e.first.second).transpose();

    // Plane Normal
    Eigen::Vector3d N = (v - u).cross(grabDirection);
    if (N.squaredNorm() < 1e-9) continue;
    N.normalize();

    bool hasNeg = false;
    bool hasPos = false;
    for (int wi : e.second) {
      Eigen::Vector3d w = m_offset_mesh_V.row(wi).transpose();
      double side = (w - u).dot(N);
      if (side < 0)
        hasNeg = true;
      else if (side > 0)
        hasPos = true;
    }

    // Type A Contact
    if (hasNeg != hasPos) {
      m_contactPoints.push_back(
          ContactPoint{(u + v) / 2, hasNeg ? N : -N, true});
    }
  }
}

void VoxelPipeline::FindTypeBContactPoints(
    const VoxelPipelineSettings& settings) {
  Eigen::Matrix<ssize_t, 3, 1> voxelCount =
      (m_offset_meshInfo.size / settings.gridSpacing)
          .array()
          .ceil()
          .cast<ssize_t>();
  Eigen::RowVector3f voxelDimension =
      voxelCount.cast<float>() * (float)settings.gridSpacing;

  // Make the object centered in the voxels space
  Eigen::RowVector3f origin =
      ((m_offset_meshInfo.minimum -
        (voxelDimension.transpose().cast<double>() - m_offset_meshInfo.size) /
            2.)
           .array() +
       (settings.voxelSize / 2))
          .cast<float>();

  // Shoot a ray from bottom
#pragma omp parallel
  {
    std::vector<igl::Hit> t_hits;
    int t_numRays;
    Eigen::RowVector3f t_direction = Eigen::RowVector3f::UnitY();
    std::vector<ContactPoint> t_contactPoints;

#pragma omp for nowait
    for (ssize_t i = 0; i < voxelCount.x(); i++) {
      for (ssize_t k = 0; k < voxelCount.z(); k++) {
        Eigen::RowVector3f position =
            origin + Eigen::RowVector3f(i, 0, k) * settings.voxelSize;

        m_offset_mesh_intersector.intersectRay(
            position, t_direction, t_hits, t_numRays);

        for (auto& hit : t_hits) {
          t_contactPoints.push_back(
              ContactPoint{(position + t_direction * hit.t).cast<double>(),
                           m_offset_mesh_N.row(hit.id),
                           false});
        }
      }
    }

#pragma omp critical
    m_contactPoints.insert(
        m_contactPoints.end(), t_contactPoints.begin(), t_contactPoints.end());
  }
}

void VoxelPipeline::FilterFeasibleContactPoints(
    const VoxelPipelineSettings& settings) {
  // Only consider faces that are tilted by more than threshold angle.
  static const double thresholdY =
      -sin(settings.thresholdAngle * DEGREE_TO_RADIAN);

  std::vector<ContactPoint> filteredContactPoints;

#pragma omp parallel
  {
    std::vector<ContactPoint> t_contactPoints;
    igl::Hit t_hit;
    Eigen::RowVector3f t_grabDirection =
        GetDirectionFromAngle(settings.grabAngle).transpose();
#pragma omp for nowait
    for (ssize_t i = 0; i < m_contactPoints.size(); i++) {
      // Check Floor
      if (m_contactPoints[i].position.y() <= m_meshInfo.minimum.y()) continue;
      // Check Supportive
      if (m_contactPoints[i].normal.y() >= thresholdY) continue;
      // Check Reachable
      if (m_offset_mesh_intersector.intersectRay(
              m_contactPoints[i].position.cast<float>().transpose(),
              t_grabDirection,
              t_hit))
        continue;
      t_contactPoints.push_back(m_contactPoints[i]);
    }

#pragma omp critical
    filteredContactPoints.insert(filteredContactPoints.end(),
                                 t_contactPoints.begin(),
                                 t_contactPoints.end());
  }

  m_contactPoints = filteredContactPoints;
}

void VoxelPipeline::FindBestContactPoints(
    const VoxelPipelineSettings& settings) {
  typedef Eigen::Matrix<ssize_t, 3, 1> Index3;
  Index3 bestIndex(-1, -1, -1);
  std::pair<int, double> bestStability = {-1, 100};
  ssize_t numContacts = m_contactPoints.size();

#pragma omp parallel
  {
    Index3 t_bestIndex(-1, -1, -1);
    std::pair<int, double> t_bestStability = {-1, 100};
    std::pair<int, double> t_stability = {-1, 100};

#pragma omp for
    for (ssize_t i = 0; i < numContacts; i++) {
      for (ssize_t j = i + 1; j < numContacts; j++) {
        for (ssize_t k = j + 1; k < numContacts; k++) {
          t_stability = EvaluateContactPoints(m_centerOfMass,
                                              m_contactPoints[i],
                                              m_contactPoints[j],
                                              m_contactPoints[k]);
          if (t_stability > t_bestStability) {
            t_bestStability = t_stability;
            t_bestIndex = Index3(i, j, k);
          }
        }
      }
    }

#pragma omp critical
    if (t_bestStability > bestStability) {
      bestStability = t_bestStability;
      bestIndex = t_bestIndex;
    }
  }

  m_bestContactPoints.clear();
  for (int i = 0; i < 3; i++) {
    if (bestIndex(i) != -1) m_bestContactPoints.push_back(m_contactPoints[bestIndex(i)]);
  }
}

void VoxelPipeline::SetViewerData() {
  static Eigen::RowVector3d centerOfMassColor =
      Eigen::RowVector3d(130, 4, 1) / 255;

  m_mainUI->viewerDataMutex.lock();

  // Candidate Layer
  igl::opengl::ViewerData& data_all =
      m_mainUI->GetViewerData(LayerId::Candidates);
  data_all.clear();
  data_all.set_points(m_contactPoints_P, m_contactPoints_PC);

  // Best Layer
  igl::opengl::ViewerData& data_filtered =
      m_mainUI->GetViewerData(LayerId::BestContacts);
  data_filtered.clear();
  data_filtered.set_points(m_bestContactPoints_P, m_bestContactPoints_PC);

  // Center of Mass Layer
  igl::opengl::ViewerData& data_cm =
      m_mainUI->GetViewerData(LayerId::CenterOfMass);
  data_cm.clear();
  data_cm.set_points(m_centerOfMass.transpose(), centerOfMassColor);

  // Gripper Layer
  igl::opengl::ViewerData& data_gripper =
      m_mainUI->GetViewerData(LayerId::GripperMesh);
  data_gripper.clear();
  data_gripper.set_face_based(true);
  data_gripper.set_mesh(m_gripper.V(), m_gripper.F());
  data_gripper.set_colors(Eigen::RowVector3d::Constant(0.4));

  // Offset Mesh Layer
  igl::opengl::ViewerData& data_offset =
      m_mainUI->GetViewerData(LayerId::Offset);
  data_offset.clear();
  data_offset.set_mesh(m_offset_mesh_V, m_offset_mesh_F);

  m_mainUI->viewerDataMutex.unlock();
}

void VoxelPipeline::GeneratePoints(const std::vector<ContactPoint>& points,
                                   Eigen::MatrixXd& out_P,
                                   Eigen::MatrixXd& out_PC) {
  static Eigen::RowVector3d typeAColor = Eigen::RowVector3d(219, 76, 178) / 255;
  static Eigen::RowVector3d typeBColor = Eigen::RowVector3d(239, 126, 50) / 255;

  out_P.resize(points.size(), 3);
  out_PC.resize(points.size(), 3);

#pragma omp parallel for
  for (ssize_t i = 0; i < points.size(); i++) {
    out_P.row(i) = points[i].position.transpose();
    out_PC.row(i) = points[i].isTypeA ? typeAColor : typeBColor;
  }
}

}  // namespace gripper