#include "safe_cartesian_moveit/CollisionCheckMoveIt.hpp"

// system includes

// library includes
#include <moveit/robot_model_loader/robot_model_loader.h>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
CollisionCheckMoveIt::CollisionCheckMoveIt()
  :m_planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description")),
   m_locked_planning_scene(m_planning_scene_monitor)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  m_planning_scene_monitor->requestPlanningSceneState();
}


collision_detection::CollisionResult
CollisionCheckMoveIt::getCollisionResult(const sensor_msgs::JointState& targetJointsState, bool contacts)
{
  const robot_state::RobotState& current_state = m_locked_planning_scene->getCurrentState();

  robot_state::RobotState target_state(current_state);
  for (size_t jointIdx = 0; jointIdx < targetJointsState.name.size(); jointIdx++) {
    target_state.setJointPositions(targetJointsState.name[jointIdx], &targetJointsState.position[jointIdx]);
  }
  target_state.update();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  // TODO collision_request.group_name = "lwr2";
  collision_request.contacts = contacts;
  collision_request.max_contacts = 100;
  collision_result.clear();
  m_locked_planning_scene->checkCollision(collision_request, collision_result, target_state);

  return collision_result;
}


bool
CollisionCheckMoveIt::hasCollision(const sensor_msgs::JointState& targetJointsState) {
  collision_detection::CollisionResult collision_result = getCollisionResult(targetJointsState);
  return collision_result.collision;
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
