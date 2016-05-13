//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jimmy Da Silva, jimmy.dasilva@isir.upmc.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software. You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef PICK_N_PLACE_HPP
#define PICK_N_PLACE_HPP

// ROS/MoveIt includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group->h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/attached_body.h>
#include <tf/transform_broadcaster.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>


#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group/capability_names.h>



#include <boost/lexical_cast.hpp>
#include <math.h>

# define M_PI 3.14159265358979323846  /* pi */

typedef move_group_interface::MoveGroup::Plan MoveGroupPlan;

class PickNPlace
{
public:
  // Constructor.
  PickNPlace();

  ////////////////////////// Outer actions /////////////////////////
  // The robot tries to plan a trajectory to its home position
  bool moveToHome();

  // From the current joint pose, the robot moves the requested bin from its location
  // to the target location, and backs away
  bool moveObjectToTarget(int bin_number, double x_target, double y_target, double angle_target, bool is_holding_bin_at_start);
  //////////////////////////////////////////////////////////////////

  //////////////// moveObjectToTarget composite actions ///////////////
  // From the current joint pose, the robot moves to the grasp location for the given bin
  // The robot is not holding a bin during this method
  bool approachObject(int bin_number, double& bin_height);

  // On the assumption that the robot is currently holding a bin, the robot moves the
  // bin to the target place location, but does not release the bin
  bool deliverObject(int bin_number, double x_target, double y_target, double angle_target, double bin_height);
  //////////////////////////////////////////////////////////////////

  ///////////////////////// Traverse actions ///////////////////////

  // Move arm to pose above bin
  bool moveAboveObject(int bin_number, double& bin_height); 

  void getObjectAbovePose(moveit_msgs::CollisionObjectPtr bin_coll_obj, geometry_msgs::Pose& pose,
                       double& bin_height);

  // Move arm to target (X,Y,R), above a place location
  // angle_target is in degrees
  bool carryObjectTo(double x_target, double y_target, double angle_target, double bin_height);
  // finds the pose above a target bin
  // angle_target is in degrees
  void getCarryObjectPose(double x_target, double y_target, double angle_target, double bin_height, 
                       geometry_msgs::Pose& pose);

  // Move arm across freespace to target pose  
  bool traverseMove(geometry_msgs::Pose& pose);
  //////////////////////////////////////////////////////////////////

  ///////////////////////// Vertical actions ///////////////////////

  // Move arm vertically up above bins
  bool ascent(double bin_height);

  // Move arm vertically down to place height
  bool descent(double bin_height);  

  // From current pose, move arm vertically to target z
  bool verticalMove(double target_z);
  //////////////////////////////////////////////////////////////////

  // grasp bin and attach the collision model to the arm
  bool attachObject(int bin_number);  

  // release bin and detach the collision model from the arm
  bool detachObject();

  // Finds out if the robot needs to rotate clockwise or anti-clockwise
  double optimalGoalAngle(double goal_angle, double current_angle);

  // get the collision object corresponding to the bin_number
  moveit_msgs::CollisionObjectPtr getBinCollisionObject(int bin_number);

  // execute a joint trajectory
  bool executeJointTrajectory(MoveGroupPlan& mg_plan, bool check_safety);
  // stop a joint trajectory
  void stopJointTrajectory();

  bool executeGripperAction(bool is_close, bool wait_for_result);

  void getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js_msg);

  void secStoppedCallback(const std_msgs::Bool::ConstPtr& msg);
  void emergStoppedCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::ServiceClient ik_service_client, fk_service_client, cartesian_path_service_;
  moveit_msgs::GetPositionIK::Request ik_srv_req;
  moveit_msgs::GetPositionIK::Response ik_srv_resp;

  ros::Publisher attached_object_publisher;
  ros::Publisher planning_scene_diff_publisher;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor; 
  boost::shared_ptr<tf::TransformListener> tf;
  move_group_interface::MoveGroup group;
  ros::AsyncSpinner spinner;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> excel_ac;
  bool sim;

//   moveit_msgs::JointConstraint rail_constraint, shoulder_constraint,elbow_constraint;
//   double rail_max, rail_min, rail_tolerance;

//   bool vertical_check_safety_, traverse_check_safety_;
//   bool human_unsafe_;
  ros::Subscriber hum_unsafe_sub_, human_pose_sub_;
  ros::Subscriber joint_state_sub_;
  geometry_msgs::Pose human_pose;
  bool use_gripper;

  ros::Subscriber sec_stopped_sub_;
  ros::Subscriber emerg_stopped_sub_;
  bool security_stopped_, emergency_stopped_;

  bool success;
  bool is_still_holding_bin;

  double q_cur[7];
};

#endif