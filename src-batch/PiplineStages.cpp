#include "PipelineStages.h"

#include <igl/readSTL.h>
#include <igl/remove_duplicate_vertices.h>
#include <Eigen/Core>
#include <fstream>
#include <stdexcept>
#include <chrono>

#include <passive-gripper/Initialization.h>
#include <passive-gripper/PassiveGripper.h>
#include <passive-gripper/models/SettingsOverrider.h>
#include <utils.h>

namespace psg_batch {

void GeneratePSG(const std::string& stl_fn, const std::string& psg_fn) {
  std::ifstream stl_f(stl_fn, std::ios::in | std::ios::binary);
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd N;
  if (!igl::readSTL(stl_f, V, F, N)) {
    throw std::invalid_argument("Failed to read STL file " + stl_fn);
  }

  Eigen::MatrixXd SV;
  Eigen::VectorXd SVI;
  Eigen::VectorXd SVJ;
  igl::remove_duplicate_vertices(V, 0, SV, SVI, SVJ);
  Eigen::MatrixXi SF = F;
  for (size_t i = 0; i < SF.size(); i++) {
    SF(i) = SVJ(SF(i));
  }

  Log() << "Num vertices: " << V.rows() << std::endl;

  std::ofstream psg_f(psg_fn, std::ios::out | std::ios::binary);
  if (!psg_f.is_open()) {
    throw std::invalid_argument("Cannot open " + psg_fn);
  }

  psg::PassiveGripper psg;
  Eigen::MatrixXd SV2;
  Eigen::Affine3d trans;
  psg::InitializeMeshPosition(SV, SV2, trans);
  psg.SetMesh(SV2, SF);

  psg.Serialize(psg_f);
  Log() << "PSG written to: " << psg_fn << std::endl;
}

void GenerateCPX(const psg::PassiveGripper& psg,
                 const std::string& cpx_fn) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto cps = psg::InitializeGCs(
      psg, psg::kNCandidates, psg::kNSeeds);
  auto stop_time = std::chrono::high_resolution_clock::now();
  long long duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                           stop_time - start_time)
                           .count();

  Log() << cps.size() << " candidates generated" << std::endl;
  Log() << "Contact Point Generation took " << duration << " ms." << std::endl;

  std::ofstream cp_f(cpx_fn, std::ios::out | std::ios::binary);
  if (!cp_f.is_open()) {
    throw std::invalid_argument("Cannot open " + cpx_fn);
  }
  psg::serialization::Serialize(cps, cp_f);
  Log() << "Contact point candidate written to: " << cpx_fn << std::endl;
}

}  // namespace psg_batch