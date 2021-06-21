#include "IKUtils.h"
#include <iostream>

void robots::Forward(const std::vector<double>& jointConfig,
                     Eigen::Matrix3d& out_rot,
                     Eigen::Vector3d& out_trans) {
  if (jointConfig.size() != GetNumJoints()) {
    std::cout << "Error: forward kinematics expects 6 values" << std::endl;
    return;
  }

  double eerot[9];
  double eetrans[3];

  ComputeFk(&jointConfig[0], eetrans, eerot);

  for (size_t i = 0; i < 9; i++) {
    out_rot(i) = eerot[i];
  }
  for (size_t i = 0; i < 3; i++) {
    out_trans(i) = eetrans[i];
  }
}

bool robots::Inverse(const Eigen::Matrix3d& rot,
                     const Eigen::Vector3d& trans,
                     std::vector<std::vector<double>>& out_jointConfigs) {
  using namespace ikfast;
  IkSolutionList<IkReal> solutions;
  std::vector<IkReal> vfree(GetNumFreeParameters());

  bool success = ComputeIk(trans.data(),
                           rot.data(),
                           vfree.size() > 0 ? &vfree[0] : nullptr,
                           solutions);

  if (!success) {
    std::cout << "Error: inverse kinematics failed" << std::endl;
    return false;
  }

  size_t numSolutions = solutions.GetNumSolutions();
  std::vector<IkReal> solvalues(GetNumJoints());
  out_jointConfigs.clear();

  for (size_t i = 0; i < numSolutions; i++) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    size_t numSolFreeParams = sol.GetFree().size();
    std::vector<IkReal> vsolfree(numSolFreeParams);
    sol.GetSolution(solvalues, vsolfree);
    out_jointConfigs.push_back(solvalues);
  }

  return true;
}
