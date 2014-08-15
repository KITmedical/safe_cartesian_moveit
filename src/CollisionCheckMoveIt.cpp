#include "safe_cartesian_moveit/CollisionCheckMoveIt.hpp"

// system includes

// library includes
#include <moveit/robot_model_loader/robot_model_loader.h>

// custom includes
#include <ahbstring.h>

#define DEBUG_COLLISIONS

/*---------------------------------- public: -----------------------------{{{-*/
CollisionCheckMoveIt::CollisionCheckMoveIt()
  :m_planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"))
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (!ros::service::waitForService("get_planning_scene", ros::Duration(1)))
    ;
  m_planning_scene_monitor->requestPlanningSceneState();

  m_planning_scene_monitor->startSceneMonitor();
  m_planning_scene_monitor->startStateMonitor();
  m_planning_scene_monitor->startWorldGeometryMonitor();

  while (!m_node.hasParam("single_to_composite_joints")) {
    ROS_INFO("Waiting for joint mappings to become available.");
    ros::Duration(1).sleep();
  }
  if (ros::param::get("single_to_composite_joints", m_joint_names_map)) {
    ROS_INFO_STREAM("Using joint mappings: " << ahb::string::toString(m_joint_names_map));
  }
}


collision_detection::CollisionResult
CollisionCheckMoveIt::getCollisionResult(const sensor_msgs::JointState& targetJointsState, bool contacts)
{
  //ROS_INFO_STREAM("Checking collision for:\n" << targetJointsState);

  planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(m_planning_scene_monitor);
  const robot_state::RobotState& current_state = locked_planning_scene->getCurrentState();

  robot_state::RobotState target_state(current_state);
  for (size_t jointIdx = 0; jointIdx < targetJointsState.name.size(); jointIdx++) {
    std::string joint_name;
    if (m_joint_names_map.size() == 0) {
      joint_name = targetJointsState.name[jointIdx];
    } else {
      joint_name = m_joint_names_map[targetJointsState.name[jointIdx]];
    }
    // TODO if joint_name is unknown, report error, do not call setJointPositions (== do not crash)
    target_state.setJointPositions(joint_name, &targetJointsState.position[jointIdx]);
  }
  target_state.update();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = contacts;
#ifdef DEBUG_COLLISIONS
  collision_request.contacts = true;
#endif
  collision_request.max_contacts = 100;
  collision_result.clear();
  locked_planning_scene->checkCollision(collision_request, collision_result, target_state);

#ifdef DEBUG_COLLISIONS
  if (collision_result.collision) {
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it) {
      ROS_INFO("Collision between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }
  }
#endif

  return collision_result;
}


bool
CollisionCheckMoveIt::hasCollision(const sensor_msgs::JointState& targetJointsState) {
  collision_detection::CollisionResult collision_result = getCollisionResult(targetJointsState);
  return collision_result.collision;
}

collision_detection::CollisionResult
CollisionCheckMoveIt::getPathCollisionResult(const std::vector<sensor_msgs::JointState>& targetJointsStateVector, bool contacts)
{
  collision_detection::CollisionResult collision_result;
  for (std::vector<sensor_msgs::JointState>::const_iterator it = targetJointsStateVector.begin(); it != targetJointsStateVector.end(); ++it) {
    collision_result = getCollisionResult(*it, contacts);
    if (collision_result.collision) {
#ifdef DEBUG_COLLISIONS
      ROS_INFO("Collision at %ld of %zd in path.", it - targetJointsStateVector.begin(), targetJointsStateVector.size());
#endif
      return collision_result;
    }
  }

  return collision_result;
}

bool
CollisionCheckMoveIt::hasPathCollision(const std::vector<sensor_msgs::JointState>& targetJointsStateVector)
{
  collision_detection::CollisionResult collision_result = getPathCollisionResult(targetJointsStateVector);
  return collision_result.collision;
}

/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
