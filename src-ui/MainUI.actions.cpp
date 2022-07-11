// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "MainUI.h"
#include <igl/FileEncoding.h>
#include <igl/writeSTL.h>
#include <igl/remove_duplicate_vertices.h>
#include <passive-gripper/GeometryUtils.h>

namespace psg {
namespace ui {

void MainUI::OnLoadMeshClicked() {
  std::string filename = igl::file_dialog_open();
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }

  std::string extension = filename.substr(last_dot + 1);

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (extension == "obj" || extension == "OBJ") {
    Eigen::MatrixXd corner_normals;
    Eigen::MatrixXi fNormIndices;

    Eigen::MatrixXd UV_V;
    Eigen::MatrixXi UV_F;

    if (!(igl::readOBJ(
            filename, V, UV_V, corner_normals, F, UV_F, fNormIndices))) {
      printf("Error: %s is not a recognized file type.\n", extension.c_str());
      return;
    }
  } else {
    if (!igl::read_triangle_mesh(filename, V, F)) {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n", extension.c_str());
      return;
    }
  }

  Eigen::MatrixXd SV;
  Eigen::VectorXd SVI;
  Eigen::VectorXd SVJ;
  igl::remove_duplicate_vertices(V, 0, SV, SVI, SVJ);
  Eigen::MatrixXi SF = F;
  for (Eigen::Index i = 0; i < SF.size(); i++) {
    SF(i) = SVJ(SF(i));
  }

  if (is_millimeter_) {
    SV /= 1000.;
  }
  if (is_swap_yz_) {
    SV.col(1).swap(SV.col(2));
    SV.col(0) *= -1;
  }

  SV_ = SV;
  SF_ = SF;

  vm_.SetMesh(SV, SF);
  OnAlignCameraCenter();
  optimizer_.Reset();
}

void MainUI::OnSaveMeshClicked() {
  std::string filename = igl::file_dialog_save();
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }
  std::string extension = filename.substr(last_dot + 1);
  if (extension != "stl") {
    std::cerr << "Error: Not an .stl file" << filename << std::endl;
    return;
  }

  igl::writeSTL(filename, SV_, SF_, igl::FileEncoding::Binary);
  std::cout << "Mesh saved to " << filename << std::endl;
}

void MainUI::OnMergeMeshClicked() {
  if (SV_.rows() == 0) {
    std::cout << "Warning: mesh is empty" << std::endl;
    return;
  }

  Eigen::MatrixXd SV;
  Eigen::MatrixXi SF;
  MergeMesh(SV_, SF_, SV, SF);

  SV_.swap(SV);
  SF_.swap(SF);
  vm_.SetMesh(SV_, SF_);
  optimizer_.Reset();
}

void MainUI::OnLoadPSGClicked() {
  std::string filename = igl::file_dialog_open();

  // Sanity check
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }
  std::string extension = filename.substr(last_dot + 1);
  if (extension != "psg") {
    std::cerr << "Error: Not a .psg file" << filename << std::endl;
    return;
  }

  std::ifstream myfile(filename, std::ios::in | std::ios::binary);
  psg::serialization::Deserialize(vm_.PSG(), myfile);
  std::cout << filename << " psg loaded!" << std::endl;

  // try load stl
  std::string stl_fn = filename.substr(0, last_dot) + ".stl";
  if (vm_.LoadGripper(stl_fn)) {
    std::cout << stl_fn << " gripper stl loaded!" << std::endl;
  }
  OnAlignCameraCenter();

  // Try load contact points
  std::string cpx_fn = filename.substr(0, last_dot) + ".cpx";
  std::ifstream cpx_file(cpx_fn, std::ios::in | std::ios::binary);
  if (cpx_file.good()) {
    contact_point_candidates_.clear();
    psg::serialization::Deserialize(contact_point_candidates_, cpx_file);
  }
}

void MainUI::OnSavePSGClicked() {
  std::string filename = igl::file_dialog_save();

  // Sanity check
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }
  std::string extension = filename.substr(last_dot + 1);
  if (extension != "psg") {
    std::cerr << "Error: Not a .psg file" << filename << std::endl;
    return;
  }

  std::ofstream myfile(filename, std::ios::out | std::ios::binary);
  psg::serialization::Serialize(vm_.PSG(), myfile);
  std::cout << "PSG saved to " << filename << std::endl;
}

void MainUI::OnAlignCameraCenter() {
  const auto& mesh = GetLayer(Layer::kMesh);
  viewer->core().align_camera_center(mesh.V);
}

void MainUI::OnExportContactPointCandidates() {
  std::string filename = igl::file_dialog_save();
  if (filename.empty()) return;
  std::ofstream f(filename, std::ios::out | std::ios::binary);
  psg::serialization::Serialize(contact_point_candidates_, f);
}

void MainUI::OnLoadContactPointCandidates() {
  std::string filename = igl::file_dialog_open();
  if (filename.empty()) return;
  std::ifstream f(filename, std::ios::in | std::ios::binary);
  contact_point_candidates_.clear();
  psg::serialization::Deserialize(contact_point_candidates_, f);
}

}
}  // namespace psg
