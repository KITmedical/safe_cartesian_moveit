#include "safe_cartesian_moveit/CollisionCheckMoveIt.hpp"

// system includes

// library includes
#include <moveit/robot_model_loader/robot_model_loader.h>

// custom includes
#include <ahbstring.h>


/*---------------------------------- public: -----------------------------{{{-*/
// TODO robot_description vs composite-robot_description
// remap as in joint_topic_merger.py
// use a map, fill it ONCE in some init function
CollisionCheckMoveIt::CollisionCheckMoveIt()
  :m_planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description")),
   m_locked_planning_scene(m_planning_scene_monitor)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (ros::param::get("single_to_composite_joints", m_joint_names_map)) {
    ROS_INFO_STREAM("Using joint mappings: " << ahb::string::toString(m_joint_names_map));
  }

  m_planning_scene_monitor->requestPlanningSceneState();
}


collision_detection::CollisionResult
CollisionCheckMoveIt::getCollisionResult(const sensor_msgs::JointState& targetJointsState, bool contacts)
{
  const robot_state::RobotState& current_state = m_locked_planning_scene->getCurrentState();

  robot_state::RobotState target_state(current_state);
  for (size_t jointIdx = 0; jointIdx < targetJointsState.name.size(); jointIdx++) {
    std::string joint_name;
    if (m_joint_names_map.size() == 0) {
      joint_name = targetJointsState.name[jointIdx];
    } else {
      joint_name = m_joint_names_map[targetJointsState.name[jointIdx]];
    }
    target_state.setJointPositions(joint_name, &targetJointsState.position[jointIdx]);
  }
  target_state.update();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
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
