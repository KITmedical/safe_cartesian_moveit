#ifndef _COLLISION_CHECK_MOVEIT_H_
#define _COLLISION_CHECK_MOVEIT_H_

// system includes

// library includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

// custom includes


// forward declarations


class CollisionCheckMoveIt
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    CollisionCheckMoveIt();

    // overwritten methods

    // methods
    bool hasCollision(const sensor_msgs::JointState& targetJointsState);
    collision_detection::CollisionResult getCollisionResult(const sensor_msgs::JointState& targetJointsState, bool contacts = false);

    // variables


  protected:
    // methods

    // variables
    planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;
    planning_scene_monitor::LockedPlanningSceneRO m_locked_planning_scene;
    std::map<std::string,std::string> m_joint_names_map;


  private:
    // methods

    // variables


};

#endif // _COLLISION_CHECK_MOVEIT_H_
