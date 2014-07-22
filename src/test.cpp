#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>




int main(int argc, char **argv)
{
  ros::init(argc, argv, "safe_cartesian_moveit_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  planning_scene_monitor->requestPlanningSceneState();

  planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor);
  //robot_model::RobotModelConstPtr robot_model = locked_planning_scene->getRobotModel();

  const robot_state::RobotState& current_state = locked_planning_scene->getCurrentState();
  robot_state::RobotState target_state(current_state);
  // TODO target_state.setJointPositions(...)
  target_state.update();


  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  // TODO collision_request.group_name = "lwr2";
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_result.clear();
  locked_planning_scene->checkCollision(collision_request, collision_result, target_state);
  ROS_INFO_STREAM("Target state is " << (collision_result.collision ? "in" : "not in") << " collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it) {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }


  ros::shutdown();
  return 0;
};
