#pragma once

#include <nlopt.h>
#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <vector>

#include "Constants.h"
#include "CostFunctions.h"
#include "PassiveGripper.h"
#include "serialization/Serialization.h"

namespace psg {

/// <summary>
/// CostDebugInfo contains the parameter and the cost. It is used to keep track
/// of the cost at each iteration.
/// </summary>
struct CostDebugInfo : serialization::Serializable {
  GripperParams param;
  double cost;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(param);
    SERIALIZE(cost);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(param);
      DESERIALIZE(cost);
    }
  }
};

/// <summary>
/// Optimizer manages the insert trajectory optimiztaion.
/// </summary>
class Optimizer {
 public:
  ~Optimizer();

  /// <summary>
  /// Asynchronously starts an optimization
  /// </summary>
  /// <param name="psg">Passive gripper object</param>
  void Optimize(const PassiveGripper& psg);

  /// <summary>
  /// Block until the optimization finishes
  /// </summary>
  void Wait();

  /// <summary>
  /// Cancel the optimization
  /// </summary>
  void Cancel();

  /// <summary>
  /// Restart the optimization with current parameters. This method is
  /// asynchronous.
  /// </summary>
  void Resume();

  /// <summary>
  /// Invalidate the optimizer.
  /// </summary>
  void Reset();

  inline bool IsRunning() { return opt_ != nullptr && is_running_; };
  inline bool IsResultAvailable() {
    return opt_ != nullptr && is_result_available_.load();
  }
  inline double GetCurrentCost() { return g_min_cost_; }
  inline std::chrono::time_point<std::chrono::high_resolution_clock>
  GetStartTime() {
    return start_time_;
  }
  const GripperParams& GetCurrentParams();

  // Internal use
  double ComputeCostInternal(unsigned n, const double* x, double* grad);

  bool debug = false;

 private:
  nlopt_opt opt_ = nullptr;
  int dimension_;

  GripperParams params_;
  GripperParams init_params_;
  GripperParams params_proto_;
  MeshDependentResource mdr_;
  GripperSettings settings_;
  std::unique_ptr<double> x_;
  std::unique_ptr<double> lb_;
  std::unique_ptr<double> ub_;

  std::future<nlopt_result> optimize_future_;
  std::atomic_bool is_running_ = false;
  std::atomic_bool is_resumable_ = false;
  std::atomic_bool is_result_available_ = false;

  long long n_iters_;
  std::atomic<double> g_min_cost_;
  double t_min_cost_;
  std::unique_ptr<double> g_min_x_;
  mutable std::mutex g_min_x_mutex_;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;

  std::vector<CostDebugInfo> costs_dbg_infos_;

 public:
  DECLARE_GETTER(GetIters, n_iters_);

  // not thread-safe
  DECLARE_GETTER(GetCostDebugInfos, costs_dbg_infos_);
};

}  // namespace psg