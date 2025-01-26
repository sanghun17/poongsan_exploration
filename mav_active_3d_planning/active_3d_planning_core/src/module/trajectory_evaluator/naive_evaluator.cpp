#include "active_3d_planning_core/module/trajectory_evaluator/naive_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
namespace trajectory_evaluator {

// Factory Registration
ModuleFactoryRegistry::Registration<NaiveEvaluator>
    NaiveEvaluator::registration("NaiveEvaluator");

NaiveEvaluator::NaiveEvaluator(PlannerI& planner)
    : SimulatedSensorEvaluator(planner) {}

void NaiveEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  // setup parent
  SimulatedSensorEvaluator::setupFromParamMap(param_map);
}

bool NaiveEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
  if (!traj_in->info) {
    traj_in->gain = 0.0;
    return false;
  }

    // Get Information gain
    if (planner_.isReturnPlanningMode()) {
        Eigen::Vector3d base_position = Eigen::Vector3d(0, 0, 1.0);
        Eigen::Vector3d traj_position = traj_in->trajectory.back().position_W;
        float r2b_gain = std::max(0.0, 50.0-(base_position - traj_position).norm());
        traj_in->gain = r2b_gain;
      return true;
    } 
  else{
  // remove all already observed voxels, count number of new voxels
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
  size_t i = 0;
  while (i < info->visible_voxels.size()) {
    if (planner_.getMap().isObserved(info->visible_voxels[i])) {
      info->visible_voxels.erase(info->visible_voxels.begin() + i);
    } else {
      ++i;
    }
  }
  traj_in->gain = info->visible_voxels.size();
  return true;
  }
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
