#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>




int main(int argc, char **argv)
{
  // https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/planning_scene_tutorial.cpp
  ros::init (argc, argv, "safe_cartesian_moveit_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;


  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  /*
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  */


  collision_result.clear();
  //collision_request.group_name = "lwr2";
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  planning_scene.checkCollision(collision_request, collision_result, current_state);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it) {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }


  /* TODO
   * current_state.setJointPositions()
   * planning_scene.checkCollision()
   */


  ros::shutdown();
  return 0;
};
