// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "TopoOpt.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <vector>

#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/marching_cubes.h>
#include "GeometryUtils.h"
#include "Initialization.h"
#include "PassiveGripper.h"
#include "SweptVolume.h"
#include "models/MeshDependentResource.h"
#include "robots/Robots.h"

namespace psg {

std::vector<Eigen::Vector3i> GetForbiddenVoxels(const Eigen::MatrixXd& V,
                                                const Eigen::MatrixXi& F,
                                                const Eigen::Vector3d& lb,
                                                const Eigen::Vector3d& ub,
                                                double res,
                                                Eigen::Vector3i& out_range) {
  // Holes
  std::vector<Eigen::Vector2d> h_centers;
  std::vector<double> h_radius2;
  std::vector<double> h_height;
  for (int i = 0; i < 4; i++) {
    double ang = (2. * kPi * i) / 4. + (kPi / 4.);
    h_centers.push_back(Eigen::Vector2d(cos(ang) * 0.025, sin(ang) * 0.025));
    h_radius2.push_back(0.0055 * 0.0055);
    h_height.push_back(0.035);
  }
  h_centers.push_back(Eigen::Vector2d(0, 0.025));
  h_radius2.push_back(0.003 * 0.003);
  h_height.push_back(0.01);

  // Intersect neg vol
  igl::embree::EmbreeIntersector intersector;
  intersector.init(V.cast<float>(), F, true);
  std::vector<Eigen::Vector3i> voxels;
  int x = 0, y = 0, z = 0;
  double cx, cy, cz;
  for (x = 0, cx = lb(0); cx < ub(0); x++, cx += res) {
    for (y = 0, cy = lb(1); cy < ub(1); y++, cy += res) {
      for (z = 0, cz = lb(2); cz < ub(2); z++, cz += res) {
        Eigen::Vector3d position(cx, cy, cz);
        position.array() += res / 2;

        bool work = true;
        if (position.z() <= 0.035) {
          for (size_t i = 0; i < h_centers.size(); i++) {
            if (position.z() > h_height[i]) continue;
            if ((Eigen::Vector2d(position.x(), position.y()) - h_centers[i])
                    .squaredNorm() <= h_radius2[i]) {
              voxels.push_back(Eigen::Vector3i(x, y, z));
              work = false;
              break;
            }
          }
        }
        if (!work) continue;

        std::vector<igl::Hit> hits;
        int numRays;
        intersector.intersectRay(
            position.cast<float>(), Eigen::RowVector3f::UnitZ(), hits, numRays);
        if (hits.size() % 2 == 0) {
          voxels.push_back(Eigen::Vector3i(x, y, z));
        }
      }
    }
  }
  out_range = Eigen::Vector3i(x, y, z);
  return voxels;
}

static inline Eigen::Vector3i PointToVoxel(const Eigen::Vector3d& p,
                                           const Eigen::Vector3d& lb,
                                           double res) {
  return ((p - lb) / res).cast<int>();
}

static inline Eigen::Vector3i PointToNode(const Eigen::Vector3d& p,
                                          const Eigen::Vector3d& lb,
                                          double res) {
  return ((p - lb) / res).array().round().cast<int>();
}

static inline Eigen::Vector3d NodeToPoint(const Eigen::Vector3i& p,
                                          const Eigen::Vector3d& lb,
                                          double res) {
  return (p.cast<double>() * res) + lb;
}

static inline Eigen::Vector3d VoxelToPoint(const Eigen::Vector3i& p,
                                           const Eigen::Vector3d& lb,
                                           double res) {
  Eigen::Vector3d a = (p.cast<double>().array() + 0.5) * res;
  return a + lb;
}

static inline int VoxelToElemIndex(const Eigen::Vector3i& v,
                                   const Eigen::Vector3i& range) {
  return v.z() * range.x() * range.y() + v.x() * range.y() +
         (range.y() - v.y() - 1) + 1;
}

static inline int VoxelToNodeIndex(const Eigen::Vector3i& v,
                                   const Eigen::Vector3i& range) {
  return v.z() * (range.x() + 1) * (range.y() + 1) + v.x() * (range.y() + 1) +
         (range.y() - v.y()) + 1;
}

static std::vector<int> ConvertToElemIndices(
    const std::vector<Eigen::Vector3i>& v,
    const Eigen::Vector3i& range) {
  std::vector<int> res(v.size());
  for (size_t i = 0; i < v.size(); i++) {
    res[i] = VoxelToElemIndex(v[i], range);
  }
  std::sort(res.begin(), res.end());
  res.resize(std::unique(res.begin(), res.end()) - res.begin());
  return res;
}

static std::vector<int> ConvertToNodeIndices(
    const std::vector<Eigen::Vector3i>& v,
    const Eigen::Vector3i& range) {
  std::vector<int> res(v.size());
  for (size_t i = 0; i < v.size(); i++) {
    res[i] = VoxelToNodeIndex(v[i], range);
  }
  std::sort(res.begin(), res.end());
  res.resize(std::unique(res.begin(), res.end()) - res.begin());
  return res;
}

static bool VoxelValid(const Eigen::Vector3i& v, const Eigen::Vector3i& range) {
  for (int i = 0; i < 3; i++) {
    if (v(i) < 0) return false;
    if (v(i) >= range(i)) return false;
  }
  return true;
}

static Eigen::Vector3i ClosestEmptySpace(
    const Eigen::Vector3i& p,
    const std::vector<int>& forbidden_indices,
    const Eigen::Vector3i& range) {
  int d = 0;
  while (true) {
    for (int dx = -d; dx <= d; dx++) {
      for (int dy = -d; dy <= d; dy++) {
        for (int dz = -d; dz <= d; dz++) {
          Eigen::Vector3i newP = p + Eigen::Vector3i(dx, dy, dz);
          if (!VoxelValid(newP, range)) continue;
          int index = VoxelToElemIndex(newP, range);
          auto res = std::lower_bound(
              forbidden_indices.begin(), forbidden_indices.end(), index);
          if (res == forbidden_indices.end() || *res != index) {
            return newP;
          }
        }
      }
    }
    d++;
  }
  return Eigen::Vector3i::Constant(-1);
}

template <class T>
static std::string FormatNodeList(const T& v) {
  std::string result;
  for (auto& point : v) {
    if (result.size() != 0) {
      result += ';';
    }
    result += std::to_string(point);
  }
  return result;
}

static void WriteToFile(std::string fileName,
                        const std::map<std::string, std::string>& config) {
  std::ofstream fout(fileName);
  fout << "[ToPy Problem Definition File v2007]" << std::endl;
  for (auto& pair : config) {
    auto& key = pair.first;
    auto& value = pair.second;
    fout << key << ":" << value << std::endl;
  }
}

void GenerateTopyConfig(const PassiveGripper& psg,
                        const Eigen::MatrixXd& neg_V,
                        const Eigen::MatrixXi& neg_F,
                        const std::string& filename,
                        Debugger* debugger) {
  Eigen::Vector3d csv_lb;
  Eigen::Vector3d csv_ub;
  InitializeConservativeBound(psg, csv_lb, csv_ub);

  double csv_volume = (csv_ub - csv_lb).prod();
  double volume = Volume(neg_V, neg_F);
  double vol_frac = (csv_volume * psg.GetTopoOptSettings().vol_frac) / volume;

  Eigen::Vector3d lb = psg.GetTopoOptSettings().lower_bound;
  Eigen::Vector3d ub = psg.GetTopoOptSettings().upper_bound;
  double res = psg.GetTopoOptSettings().topo_res;
  Eigen::Vector3i range;

  std::vector<Eigen::Vector3i> forbidden_voxels =
      GetForbiddenVoxels(neg_V, neg_F, lb, ub, res, range);
  std::vector<int> forbidden_indices =
      ConvertToElemIndices(forbidden_voxels, range);

  if (debugger != nullptr) {
    for (const auto& v : forbidden_voxels) {
      debugger->AddCube(NodeToPoint(v, lb, res),
                        Eigen::Vector3d::Constant(res));
    }
  }

  std::vector<Eigen::Vector3i> attachment_voxels;
  double radius = psg.GetTopoOptSettings().attachment_size / 2;
  double radius2 = radius * radius;

  Eigen::Vector3i attachment_lb =
      PointToNode(Eigen::Vector3d(-radius, -radius, 0), lb, res);
  Eigen::Vector3i attachment_ub =
      PointToNode(Eigen::Vector3d(radius, radius, 0), lb, res);

  for (int x = attachment_lb.x(); x <= attachment_ub.x(); x++) {
    for (int y = attachment_lb.y(); y <= attachment_ub.y(); y++) {
      Eigen::Vector3i v = Eigen::Vector3i(x, y, 0);
      if (NodeToPoint(v, lb, res).squaredNorm() < radius2) {
        attachment_voxels.push_back(v);
      }
    }
  }

  std::vector<int> attachment_indices =
      ConvertToNodeIndices(attachment_voxels, range);

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(psg.GetTrajectory().front()).inverse();
  std::vector<Eigen::Vector3i> contact_voxels;
  for (const auto& point : psg.GetContactPoints()) {
    contact_voxels.push_back(ClosestEmptySpace(
        PointToVoxel(finger_trans_inv * point.position, lb, res),
        forbidden_indices,
        range));
    if (debugger != nullptr) {
      debugger->AddEdge(
          VoxelToPoint(contact_voxels.back(), lb, res),
          VoxelToPoint(contact_voxels.back(), lb, res) + 0.02 * point.normal,
          colors::kRed);
      debugger->AddPoint(finger_trans_inv * point.position, colors::kPurple);
    }
  }
  std::vector<int> contact_indices =
      ConvertToNodeIndices(contact_voxels, range);

  size_t lastdot = filename.rfind('.');
  size_t lastslash = filename.rfind('/');
  if (lastslash == std::string::npos) lastslash = filename.rfind('\\');

  std::map<std::string, std::string> config = kTopyConfig;
  config["PROB_NAME"] =
      filename.substr(lastslash + 1,
                      (lastdot == std::string::npos) ? std::string::npos
                                                     : lastdot - lastslash - 1);
  config["VOL_FRAC"] = std::to_string(vol_frac);
  config["NUM_ELEM_X"] = std::to_string(range(0));
  config["NUM_ELEM_Y"] = std::to_string(range(1));
  config["NUM_ELEM_Z"] = std::to_string(range(2));
  config["FXTR_NODE_X"] = config["FXTR_NODE_Y"] = config["FXTR_NODE_Z"] =
      FormatNodeList(attachment_indices);
  config["LOAD_NODE_X"] = config["LOAD_NODE_Y"] = config["LOAD_NODE_Z"] =
      FormatNodeList(contact_indices);
  std::vector<double> loadX, loadY, loadZ;
  for (const auto& point : psg.GetContactPoints()) {
    Eigen::Vector3d tN = finger_trans_inv.linear() * -point.normal;
    loadX.push_back(tN(0));
    loadY.push_back(tN(1));
    loadZ.push_back(tN(2));
  }
  config["LOAD_VALU_X"] = FormatNodeList(loadX);
  config["LOAD_VALU_Y"] = FormatNodeList(loadY);
  config["LOAD_VALU_Z"] = FormatNodeList(loadZ);
  config["PASV_ELEM"] = FormatNodeList(forbidden_indices);

  WriteToFile(filename, config);
}

bool LoadResultBin(const PassiveGripper& psg,
                   const std::string& filename,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F) {
  if (filename.empty()) return false;
  std::ifstream myfile(filename, std::ios::in | std::ios::binary);
  if (!myfile.is_open()) return false;
  long long rx, ry, rz;
  Eigen::VectorXd values;
  serialization::Deserialize(rx, myfile);
  serialization::Deserialize(ry, myfile);
  serialization::Deserialize(rz, myfile);
  serialization::Deserialize(values, myfile);

  Eigen::Vector3d lb = psg.GetTopoOptSettings().lower_bound;
  double res = psg.GetTopoOptSettings().topo_res;

  // marching cube
  Eigen::Vector3i range(rx + 2, ry + 2, rz + 2);
  long long total_size = range.prod();

  Eigen::VectorXd S(total_size);
  Eigen::MatrixXd P(total_size, 3);

  S.setConstant(0.5);
  for (long long i = 0; i < total_size; i++) {
    long long ex = i % range.x() - 1;
    long long ey = (i / range.x()) % range.y() - 1;
    long long ez = i / ((long long)range.x() * range.y()) - 1;
    P.row(i) = VoxelToPoint(Eigen::Vector3i(ex, ey, ez), lb, res);
  }

  Eigen::VectorXd fltr_values;
  fltr_values.resize(values.rows(), 1);
  for (Eigen::Index i = 0; i < values.rows(); i++) {
    long long ex = i % rx;
    long long ey = (i / rx) % ry;
    long long ez = (i / (rx * ry));

    int count = 0;
    double val = 0;
    for (int ii = -1; ii <= 1; ii++) {
      for (int jj = -1; jj <= 1; jj++) {
        for (int kk = -1; kk <= 1; kk++) {
          long long fx = ex + ii;
          long long fy = ey + jj;
          long long fz = ez + kk;
          if (fx < 0 || fy < 0 || fz < 0 || fx >= rx || fy >= ry || fz >= rz)
            continue;
          long long findex = fz * (rx * ry) + fy * rx + fx;
          val += values(findex);
          count++;
        }
      }
    }
    fltr_values(i) = (count == 0) ? 0 : (val / count);
  }

  for (Eigen::Index i = 0; i < values.rows(); i++) {
    long long ex = i % rx;
    long long ey = (i / rx) % ry;
    long long ez = (i / (rx * ry));
    long long index = (ex + 1ll) + (ey + 1ll) * (long long)range.x() +
                      (ez + 1ll) * (long long)range.x() * range.y();
    S(index) = 0.5 - fltr_values(i);
  }

  igl::copyleft::marching_cubes(
      S, P, range.x(), range.y(), range.z(), 0., out_V, out_F);
  return true;
}

void RefineGripper(const PassiveGripper& psg,
                   const Eigen::MatrixXd& V,
                   const Eigen::MatrixXi& F,
                   const Eigen::MatrixXd& neg_V,
                   const Eigen::MatrixXi& neg_F,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F) {
  Eigen::MatrixXd V1, V2, VR;
  Eigen::MatrixXi F1, F2, FR;

  // std::cout << "Adding contact points" << std::endl;
  Eigen::MatrixX3d CP(psg.GetContactPoints().size(), 3);
  for (size_t i = 0; i < psg.GetContactPoints().size(); i++) {
    CP.row(i) = psg.GetContactPoints()[i].position.transpose();
  }
  CP = (psg.GetFingerTransInv() * CP.transpose().colwise().homogeneous())
           .transpose();

  CreateSpheres(CP, psg.GetTopoOptSettings().contact_point_size, 10, V2, F2);

  igl::copyleft::cgal::mesh_boolean(
      V, F, V2, F2, igl::MESH_BOOLEAN_TYPE_UNION, V1, F1);

  // std::cout << "Adding base" << std::endl;
  constexpr int cyl_res = 16;
  double thickness = psg.GetTopoOptSettings().base_thickness;
  CreateCylinderXY(Eigen::Vector3d::Zero(), 0.0315, thickness, cyl_res, V2, F2);

  igl::copyleft::cgal::mesh_boolean(
      V1, F1, V2, F2, igl::MESH_BOOLEAN_TYPE_UNION, VR, FR);

  // subtracting holes

  Eigen::MatrixXd holes_V(cyl_res * 8, 3);
  Eigen::MatrixXi holes_F((cyl_res * 4 - 4) * 4, 3);

  Eigen::MatrixXd clr_V(cyl_res * 8, 3);
  Eigen::MatrixXi clr_F((cyl_res * 4 - 4) * 4, 3);

  for (int i = 0; i < 4; i++) {
    double ang = (2. * kPi * i) / 4. + (kPi / 4.);
    CreateCylinderXY(Eigen::Vector3d(cos(ang) * 0.025, sin(ang) * 0.025, -0.01),
                     0.0035,
                     thickness + 0.01,
                     cyl_res,
                     V1,
                     F1);
    holes_V.block(i * cyl_res * 2, 0, cyl_res * 2, 3) = V1;
    holes_F.block(i * (cyl_res * 4 - 4), 0, cyl_res * 4 - 4, 3) =
        F1.array() + (i * cyl_res * 2);

    CreateCylinderXY(
        Eigen::Vector3d(cos(ang) * 0.025, sin(ang) * 0.025, thickness),
        0.0055,
        0.025,
        cyl_res,
        V1,
        F1);
    clr_V.block(i * cyl_res * 2, 0, cyl_res * 2, 3) = V1;
    clr_F.block(i * (cyl_res * 4 - 4), 0, cyl_res * 4 - 4, 3) =
        F1.array() + (i * cyl_res * 2);
  }

  igl::copyleft::cgal::mesh_boolean(
      VR, FR, holes_V, holes_F, igl::MESH_BOOLEAN_TYPE_MINUS, V2, F2);
  igl::copyleft::cgal::mesh_boolean(
      V2, F2, clr_V, clr_F, igl::MESH_BOOLEAN_TYPE_MINUS, V1, F1);

  // subtracting pin holes
  CreateCylinderXY(Eigen::Vector3d(0, 0.025, -0.01),
                   0.003,
                   thickness + 0.01,
                   cyl_res,
                   V2,
                   F2);
  igl::copyleft::cgal::mesh_boolean(
      V1, F1, V2, F2, igl::MESH_BOOLEAN_TYPE_MINUS, VR, FR);

  // std::cout << "Subtracting swept volume" << std::endl;
  igl::copyleft::cgal::mesh_boolean(
      VR, FR, neg_V, neg_F, igl::MESH_BOOLEAN_TYPE_INTERSECT, out_V, out_F);
}

}  // namespace psg
