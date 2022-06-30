#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "Constants.h"
#include "Debugger.h"
#include "models/GripperParams.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {

struct CostContext {
  Debugger* debugger;
  long long cur_iter;
};

/// <summary>
/// Compute the signed distance given a position.
/// </summary>
/// <param name="p">Position</param>
/// <param name="settings">Cost settings</param>
/// <param name="mdr">Mesh dependent resources</param>
/// <returns>The signed distance</returns>
double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr);

/// <summary>
/// Compute the objective function.
/// </summary>
/// <param name="params">Curent parameters</param>
/// <param name="init_params">Initial parameters used to compute the trajectory
/// complexity</param>
/// <param name="settings">Settings</param>
/// <param name="remeshed_mdr">Mesh dependent resources of the expanded
/// mesh</param>
/// <param name="context">Context (for debugging purposes)</param>
/// <returns>The cost</returns>
double ComputeCost_SP(const GripperParams& params,
                      const GripperParams& init_params,
                      const GripperSettings& settings,
                      const MeshDependentResource& remeshed_mdr,
                      const CostContext& context);

/// <summary>
/// Compute the minimum distance between the skeleton and the object throughout
/// the insert trajectory.
/// </summary>
/// <param name="params">Current parameters</param>
/// <param name="settings">Settings</param>
/// <param name="mdr">Mesh dependent resources</param>
/// <returns>The minimum distance between the skeleton and the object</returns>
double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr);

}  // namespace psg