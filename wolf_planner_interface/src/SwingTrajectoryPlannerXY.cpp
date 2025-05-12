#include "wolf_planner_interface/SwingTrajectoryPlannerXY.h"

#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {
namespace legged_robot {

void SwingTrajectoryPlannerXY::update(const ModeSchedule& modeSchedule,
                                      const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                      const feet_array_t<scalar_array_t>& touchDownHeightSequence,
                                      const feet_array_t<scalar_array_t>& maxHeightSequence) {
  // fallback: no XY displacement
  const size_t N = modeSchedule.modeSequence.size();
  feet_array_t<scalar_array_t> defaultXs, defaultYs;
  for (size_t j = 0; j < numFeet_; ++j) {
    scalar_array_t zeros(N, 0.0);
    defaultXs[j] = zeros;
    defaultYs[j] = zeros;
  }

  updateXY(modeSchedule,
           liftOffHeightSequence,
           touchDownHeightSequence,
           maxHeightSequence,
           defaultXs, defaultXs,
           defaultYs, defaultYs);
}

void SwingTrajectoryPlannerXY::updateXY(
    const ModeSchedule& modeSchedule,
    const feet_array_t<scalar_array_t>& liftOffHeightSequence,
    const feet_array_t<scalar_array_t>& touchDownHeightSequence,
    const feet_array_t<scalar_array_t>& maxHeightSequence,
    const feet_array_t<scalar_array_t>& liftOffXSequence,
    const feet_array_t<scalar_array_t>& touchDownXSequence,
    const feet_array_t<scalar_array_t>& liftOffYSequence,
    const feet_array_t<scalar_array_t>& touchDownYSequence) {

  SwingTrajectoryPlanner::update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);

  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);
  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;

  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    feetXTrajectories_[j].clear();
    feetYTrajectories_[j].clear();

    for (int p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // swing phase
        const int startIdx = startTimesIndices[j][p];
        const int finalIdx = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, startIdx, finalIdx, modeSequence);

        const scalar_t tStart = eventTimes[startIdx];
        const scalar_t tEnd = eventTimes[finalIdx];
        const scalar_t tMid = 0.5 * (tStart + tEnd);
        const scalar_t scaling = swingTrajectoryScaling(tStart, tEnd, config_.swingTimeScale);

        // X trajectory
        const scalar_t liftOffX = liftOffXSequence[j][p];
        const scalar_t touchDownX = touchDownXSequence[j][p];
        const scalar_t midX = 0.5 * (liftOffX + touchDownX);

        CubicSpline::Node liftOffXNode{tStart, liftOffX, 0.0};
        CubicSpline::Node midXNode{tMid, midX, 0.0};
        CubicSpline::Node touchDownXNode{tEnd, touchDownX, 0.0};

        feetXTrajectories_[j].emplace_back(CubicSpline(liftOffXNode, midXNode));
        feetXTrajectories_[j].emplace_back(CubicSpline(midXNode, touchDownXNode));

        // Y trajectory
        const scalar_t liftOffY = liftOffYSequence[j][p];
        const scalar_t touchDownY = touchDownYSequence[j][p];
        const scalar_t midY = 0.5 * (liftOffY + touchDownY);

        CubicSpline::Node liftOffYNode{tStart, liftOffY, 0.0};
        CubicSpline::Node midYNode{tMid, midY, 0.0};
        CubicSpline::Node touchDownYNode{tEnd, touchDownY, 0.0};

        feetYTrajectories_[j].emplace_back(CubicSpline(liftOffYNode, midYNode));
        feetYTrajectories_[j].emplace_back(CubicSpline(midYNode, touchDownYNode));

      } else {  // stance phase
        const scalar_t stanceX = liftOffXSequence[j][p];
        const scalar_t stanceY = liftOffYSequence[j][p];

        CubicSpline::Node node{0.0, stanceX, 0.0};
        feetXTrajectories_[j].emplace_back(CubicSpline(node, node));

        node = CubicSpline::Node{0.0, stanceY, 0.0};
        feetYTrajectories_[j].emplace_back(CubicSpline(node, node));
      }
    }

    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
}

scalar_t SwingTrajectoryPlannerXY::getXpositionConstraint(size_t leg, scalar_t time) const {
  const auto& events = feetHeightTrajectoriesEvents_[leg];
  const auto index = lookup::findIndexInTimeArray(events, time);

  const scalar_t midTime = 0.5 * (events[index] + events[index + 1]);
  if (time <= midTime) {
    return feetXTrajectories_[leg][2 * index].position(time);
  } else {
    return feetXTrajectories_[leg][2 * index + 1].position(time);
  }
}

scalar_t SwingTrajectoryPlannerXY::getYpositionConstraint(size_t leg, scalar_t time) const {
  const auto& events = feetHeightTrajectoriesEvents_[leg];
  const auto index = lookup::findIndexInTimeArray(events, time);

  const scalar_t midTime = 0.5 * (events[index] + events[index + 1]);
  if (time <= midTime) {
    return feetYTrajectories_[leg][2 * index].position(time);
  } else {
    return feetYTrajectories_[leg][2 * index + 1].position(time);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
