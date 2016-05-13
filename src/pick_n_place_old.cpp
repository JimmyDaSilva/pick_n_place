#include <lwr_pick_n_place/pick_n_place_old.hpp>

//TODO
#define GRIPPING_OFFSET 0.1
#define DZ 0.3


PickNPlace::PickNPlace() : 
  group("arm"), spinner(1)
{
  spinner.start();

  // Initialize planning scene monitor
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
  planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
  planning_scene_monitor = plg_scn_mon;

  // Get params
  ros::NodeHandle nh, nhparam_("~");
  sim = true;
  use_gripper = false;
  nhparam_.getParam("sim", sim);
  nhparam_.getParam("use_gripper", use_gripper);

  // ROS subscribers
  security_stopped_ = false;
  emergency_stopped_ = false;
  success = true;
  is_still_holding_bin = false;
  joint_state_sub_ = nh.subscribe("/joint_states", 1, &PickNPlace::jointStateCallback, this);
  sec_stopped_sub_ = nh.subscribe("/mode_state_pub/is_security_stopped", 1, &PickNPlace::secStoppedCallback,this);
  emerg_stopped_sub_ = nh.subscribe("/mode_state_pub/is_emergency_stopped", 1, &PickNPlace::emergStoppedCallback,this);

  // Params for move group
  group->setPlanningTime(8.0);
  group->allowReplanning(false);
  group->startStateMonitor(1.0);

  // Wait until the required ROS services are available
  ik_service_client = nh.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
  while(!ik_service_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }
  fk_service_client = nh.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
  while(!fk_service_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }
  cartesian_path_service_ = nh.serviceClient<moveit_msgs::GetCartesianPath>(move_group::CARTESIAN_PATH_SERVICE_NAME);
  while(!cartesian_path_service_.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  // Loading planning_scene_monitor //
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();

  // Making sure we can publish attached/unattached objects //
  attached_object_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  while(attached_object_publisher.getNumSubscribers() < 1)
  {
    sleep(1.0);
  }
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep(1.0);
  }

}

bool PickNPlace::moveToHome()
{
  while (ros::ok()) {

    moveit_msgs::PlanningScene planning_scene;
    planning_scene::PlanningScenePtr full_planning_scene;
    getPlanningScene(planning_scene, full_planning_scene);

    double arr[] = {-1.27, -0.33, -1.01, 1.80, 0.29, -1.19, -0.18};
    std::vector<double> joint_vals(arr, arr + sizeof(arr) / sizeof(arr[0]));

    // Fixing joint_0 and joint_6 given by the IK
    joint_vals[0] = this->optimalGoalAngle(joint_vals[0], planning_scene.robot_state.joint_state.position[0]);
    joint_vals[6] = this->optimalGoalAngle(joint_vals[6], planning_scene.robot_state.joint_state.position[6]);

    // TODO
    robot_state::RobotStatePtr cur_state = group->getCurrentState();
    cur_state->update(true);
    cur_state->setJointPositions("table_rail_joint", q_cur);
    group->setStartState(*cur_state);
    // group->getCurrentState()->update(true);

    group->setJointValueTarget(joint_vals);
    int num_tries = 4;
    MoveGroupPlan my_plan;
    // try to plan a few times, just to be safe
    while (ros::ok() && num_tries > 0) {
      if (group->plan(my_plan))
        break;
      num_tries--;
    }

    if (num_tries > 0) {
      // found plan, let's try and execute
      if (executeJointTrajectory(my_plan, false)) {
        ROS_INFO("Home position joint trajectory execution successful");
        return true;
      }
      else {
        ROS_WARN("Home position joint trajectory execution failed");
        ros::Duration(0.5).sleep();
        continue;
      }
    }
    else {
      ROS_ERROR("Home position Motion planning failed");
      continue;
    }
  }
  return true;
}

bool PickNPlace::moveObjectToTarget(int bin_number, double x_target, double y_target, double angle_target, bool is_holding_bin_at_start)
{
  ROS_INFO("Moving bin %d to target (%.3f, %.3f, %f)", bin_number, x_target, y_target, angle_target);

  success = false;
  is_still_holding_bin = false;

  double bin_height;
  
  ////////////////// TYPICAL OPERATION STARTS HERE ///////////////////////////
  if (!is_holding_bin_at_start) {
//     executeGripperAction(false, false); // open gripper, but don't wait
    if(!approachObject(bin_number, bin_height)) {
      ROS_ERROR("Failed to approach bin #%d.", bin_number);
      return false;
    }
    if(!attachObject(bin_number)) {
      ROS_ERROR("Failed to attach bin #%d.", bin_number);
      return false;
    }
  }
  is_still_holding_bin = true;

  ///////////////////////////// HOLDING BIN ///////////////////////////////////
  if(!deliverObject(bin_number, x_target, y_target, angle_target, bin_height)) {
    ROS_ERROR("Failed to deliver bin to target (%.3f, %.3f, %f)", 
        x_target, y_target, angle_target);
    return false;
  }
  if(!detachObject()) {
    ROS_ERROR("Failed to detach bin.");
    return false;
  }
  is_still_holding_bin = false;
  /////////////////////////////////////////////////////////////////////////////
  if(!ascent(bin_height)) {
    ROS_ERROR("Failed to ascend after releasing bin.");
    return false;
  }
  success = true;
  return true;
}

bool PickNPlace::approachObject(int bin_number, double& bin_height)
{
  ROS_INFO("Approaching bin %d", bin_number);
  if(!moveAboveObject(bin_number, bin_height)) {
    ROS_ERROR("Failed to move above bin #%d.", bin_number);
    return false;
  }
  if(!descent(bin_height)) {
    ROS_ERROR("Failed to descend after moving above bin.");
    return false;
  }
  return true;
}

bool PickNPlace::deliverObject(int bin_number, double x_target, double y_target, double angle_target, double bin_height)
{
  ROS_INFO("Delivering to target (%.3f, %.3f, %f)", x_target, y_target, angle_target);

  if(!ascent(bin_height)) {
    ROS_ERROR("Failed to ascend while grasping bin.");
    return false;
  }
  // update planning scene
  if(!carryObjectTo(x_target, y_target, angle_target, bin_height)) {
    ROS_ERROR("Failed to carry bin to target (%.3f, %.3f, %.3f)", 
        x_target, y_target, angle_target);
    return false;
  }
  // update planning scene
  if(!descent(bin_height+0.02)) {
    ROS_ERROR("Failed to descend after moving bin above target place.");
    return false;
  }
  return true;
}

bool PickNPlace::moveAboveObject(int bin_number, double& bin_height)
{
  ROS_INFO("Moving above bin %d", bin_number);
  moveit_msgs::CollisionObjectPtr bin_coll_obj = getBinCollisionObject(bin_number);
  if (!bin_coll_obj) {
    // bin not found
    ROS_ERROR("BIN NOT FOUND");
    return false;
  }

  geometry_msgs::Pose target_pose;
  getObjectAbovePose(bin_coll_obj, target_pose, bin_height);

  return traverseMove(target_pose);
}

bool PickNPlace::traverseMove(geometry_msgs::Pose& pose)
{
  ROS_INFO("Traverse move to position (%.2f, %.2f, %.2f)", 
      pose.position.x, pose.position.y, pose.position.z);
  while (ros::ok()) {

    // TODO
    group->getCurrentState()->update(true);
    // group->setStartStateToCurrentState();
    // sleep(0.3); // Add if jump violation still appears
//     robot_state::RobotStatePtr cur_state = group->getCurrentState();
//     cur_state->update(true);
//     cur_state->setJointPositions("table_rail_joint", q_cur);
//     group->setStartState(*cur_state);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene::PlanningScenePtr full_planning_scene;
    getPlanningScene(planning_scene, full_planning_scene);

    ////////////// Perform IK to find joint goal //////////////
    moveit_msgs::GetPositionIK::Request ik_srv_req;

    // setup IK request
    ik_srv_req.ik_request.group_name = "arm";
    ik_srv_req.ik_request.pose_stamped.header.frame_id = "base_link";
    ik_srv_req.ik_request.pose_stamped.header.stamp = ros::Time::now();
    ik_srv_req.ik_request.avoid_collisions = true;
    ik_srv_req.ik_request.attempts = 30;

    // set pose
    ik_srv_req.ik_request.pose_stamped.pose = pose;

    // set joint constraints
//     double rail_center = pose.position.x;
//     moveit_msgs::JointConstraint special_rail_constraint;
//     special_rail_constraint.joint_name = "table_rail_joint";
//     special_rail_constraint.position = rail_max - rail_center;
//     special_rail_constraint.tolerance_above = std::max(
//         std::min(rail_max - rail_center + rail_tolerance, rail_max) - 
//         (rail_max - rail_center), 0.0);
//     special_rail_constraint.tolerance_below = 
//       std::max((rail_max - rail_center) - 
//           std::max(rail_max - rail_center - rail_tolerance, rail_min), 0.0);
//     special_rail_constraint.weight = 1;
//     ROS_INFO("Special rail constraint: %.3f (+%.3f, -%.3f)", special_rail_constraint.position, 
//         special_rail_constraint.tolerance_above, special_rail_constraint.tolerance_below);
//     ik_srv_req.ik_request.constraints.joint_constraints.push_back(special_rail_constraint);
//     ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
//     //ik_srv_req.ik_request.constraints.joint_constraints.push_back(elbow_constraint);
// 
//     // call IK server
//     ROS_INFO("Calling IK for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
//         ik_srv_req.ik_request.pose_stamped.pose.position.x,
//         ik_srv_req.ik_request.pose_stamped.pose.position.y,
//         ik_srv_req.ik_request.pose_stamped.pose.position.z,
//         ik_srv_req.ik_request.pose_stamped.pose.orientation.x,
//         ik_srv_req.ik_request.pose_stamped.pose.orientation.y,
//         ik_srv_req.ik_request.pose_stamped.pose.orientation.z,
//         ik_srv_req.ik_request.pose_stamped.pose.orientation.w);
    moveit_msgs::GetPositionIK::Response ik_srv_resp;
    ik_service_client.call(ik_srv_req, ik_srv_resp);
    if(ik_srv_resp.error_code.val !=1){
      ROS_ERROR("IK couldn't find a solution (error code %d)", ik_srv_resp.error_code.val);
      return false;
    }
    ROS_INFO("IK returned succesfully");
    ///////////////////////////////////////////////////////////

    // Fixing joint_0 and joint_6 given by the IK
    ik_srv_resp.solution.joint_state.position[0] = this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[0], planning_scene.robot_state.joint_state.position[0]);
    ik_srv_resp.solution.joint_state.position[6] = this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[6], planning_scene.robot_state.joint_state.position[6]);

    // Plan trajectory
    //group->setStartStateToCurrentState();
    //sleep(0.5);
    // TODO
    group->getCurrentState()->update(true);
    // group->setStartStateToCurrentState();
//     cur_state = group->getCurrentState();
//     cur_state->update(true);
//     cur_state->setJointPositions("table_rail_joint", q_cur);
//     group->setStartState(*cur_state);

    group->setJointValueTarget(ik_srv_resp.solution.joint_state);
    int num_tries = 4;
    MoveGroupPlan my_plan;
    // try to plan a few times, just to be safe
    while (ros::ok() && num_tries > 0) {
      if (group->plan(my_plan))
        break;
      num_tries--;
    }

    if (num_tries > 0) {
      // found plan, let's try and execute
      if (executeJointTrajectory(my_plan, false)) {
        ROS_INFO("Traverse joint trajectory execution successful");
        return true;
      }
      else {
        ROS_WARN("Traverse joint trajectory execution failed, going to restart");
        ros::Duration(0.5).sleep();
        continue;
      }
    }
    else {
      ROS_ERROR("Motion planning failed");
      return false;
    }
  }
}

bool PickNPlace::ascent(double bin_height)
{
  ROS_INFO("Ascending");
  return verticalMove(GRIPPING_OFFSET + bin_height + DZ);
}

bool PickNPlace::descent(double bin_height)
{
  ROS_INFO("Descending");
  return verticalMove(GRIPPING_OFFSET + bin_height);
}

bool PickNPlace::verticalMove(double target_z)
{
  ROS_INFO("Vertical move to target z: %f", target_z);

  while (ros::ok()) {
    // update the planning scene to get the robot's state
    moveit_msgs::PlanningScene planning_scene;
    planning_scene::PlanningScenePtr full_planning_scene;
    getPlanningScene(planning_scene, full_planning_scene);

    ////////////// Perform FK to find end effector pose ////////////
    /*
       moveit_msgs::GetPositionFK::Request fk_request;
       moveit_msgs::GetPositionFK::Response fk_response;
       fk_request.header.frame_id = "table_link";
       fk_request.fk_link_names.push_back("ee_link");
       fk_request.robot_state = planning_scene.robot_state;
       fk_service_client.call(fk_request, fk_response);
     */
    ////////////////////////////////////////////////////////////////

#if 0
    ////////////// Perform IK to find joint goal //////////////
    moveit_msgs::GetPositionIK::Request ik_srv_req;

    // setup IK request
    ik_srv_req.ik_request.group_name = "excel";
    ik_srv_req.ik_request.pose_stamped.header.frame_id = "table_link";
    ik_srv_req.ik_request.avoid_collisions = false;
    ik_srv_req.ik_request.attempts = 100;

    // the target pose is the current location with a different z position
    ik_srv_req.ik_request.pose_stamped = group->getCurrentPose();
    // ik_srv_req.ik_request.pose_stamped = fk_response.pose_stamped[0];
    ik_srv_req.ik_request.pose_stamped.pose.position.z = target_z;

    ik_srv_req.ik_request.constraints.joint_constraints.clear();
    // ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
    //ik_srv_req.ik_request.constraints.joint_constraints.push_back(elbow_constraint);
    moveit_msgs::JointConstraint rail_fixed_constraint, shoulder_pan_fixed_constraint,
      wrist_3_fixed_constraint;
    rail_fixed_constraint.joint_name = "table_rail_joint";
    shoulder_pan_fixed_constraint.joint_name = "shoulder_pan_joint";
    wrist_3_fixed_constraint.joint_name = "wrist_3_joint";
    const double *rail_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("table_rail_joint");
    const double *shoulder_pan_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("shoulder_pan_joint");
    const double *wrist_3_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("wrist_3_joint");
    rail_fixed_constraint.position = *rail_current_pose;
    shoulder_pan_fixed_constraint.position = *shoulder_pan_current_pose;
    wrist_3_fixed_constraint.position = *wrist_3_current_pose;

    rail_fixed_constraint.tolerance_above = 0.2;
    rail_fixed_constraint.tolerance_below = 0.2;
    rail_fixed_constraint.weight = 1;
    shoulder_pan_fixed_constraint.tolerance_above = 0.2;
    shoulder_pan_fixed_constraint.tolerance_below = 0.2;
    shoulder_pan_fixed_constraint.weight = 1;
    wrist_3_fixed_constraint.tolerance_above = 0.2;
    wrist_3_fixed_constraint.tolerance_below = 0.2;
    wrist_3_fixed_constraint.weight = 1;
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(rail_fixed_constraint);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_pan_fixed_constraint);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(wrist_3_fixed_constraint);

    ROS_INFO("Calling IK for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        ik_srv_req.ik_request.pose_stamped.pose.position.x,
        ik_srv_req.ik_request.pose_stamped.pose.position.y,
        ik_srv_req.ik_request.pose_stamped.pose.position.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.x,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.y,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.w);
    moveit_msgs::GetPositionIK::Response ik_srv_resp;
    ik_service_client.call(ik_srv_req, ik_srv_resp);
    if(ik_srv_resp.error_code.val !=1){
      ROS_ERROR("IK couldn't find a solution (error code %d)", ik_srv_resp.error_code.val);
      return 0;
    }
    ROS_INFO("IK returned succesfully");

    // Fixing wrist_3 given by the IK
    ik_srv_resp.solution.joint_state.position[1] = 
      this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[1], 
          planning_scene.robot_state.joint_state.position[1]);
    ik_srv_resp.solution.joint_state.position[6] = this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[6],planning_scene.robot_state.joint_state.position[6]);

    group->setJointValueTarget(ik_srv_resp.solution.joint_state);
#endif

    // getting the current  
    // group->setStartStateToCurrentState();
//     sleep(0.3); 
    // TODO
    group->getCurrentState()->update(true);
    // group->setStartStateToCurrentState();
//     robot_state::RobotStatePtr cur_state = group->getCurrentState();
//     cur_state->update(true);
//     cur_state->setJointPositions("table_rail_joint", q_cur);
//     group->setStartState(*cur_state);

    /*
       int num_tries = 4;
       MoveGroupPlan my_plan;
       while(ros::ok() && num_tries > 0) {
       if(group->plan(my_plan))
       return executeJointTrajectory(my_plan, false);
       num_tries--;
       }
     */
    // ROS_WARN("Motion planning failed");

    geometry_msgs::Pose pose1 = group->getCurrentPose().pose;
    geometry_msgs::Pose pose2 = pose1;
    ROS_INFO("Calling cart path from pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        pose1.position.x,
        pose1.position.y,
        pose1.position.z,
        pose1.orientation.x,
        pose1.orientation.y,
        pose1.orientation.z,
        pose1.orientation.w);

    pose1.position.z = (pose1.position.z+target_z)/2.0;
    pose2.position.z = target_z;
    ROS_INFO("for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        pose2.position.x,
        pose2.position.y,
        pose2.position.z,
        pose2.orientation.x,
        pose2.orientation.y,
        pose2.orientation.z,
        pose2.orientation.w);
    // find linear trajectory
    moveit_msgs::RobotTrajectory lin_traj_msg, lin_traj_test_msg;
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(pose1);
    waypoints.push_back(pose2);

    // moveit_msgs::GetPositionIK::Request ik_srv_req;
    // ik_srv_req.ik_request.group_name = "excel";
    // ik_srv_req.ik_request.pose_stamped.header.frame_id = "table_link";
    // ik_srv_req.ik_request.avoid_collisions = true;
    // ik_srv_req.ik_request.attempts = 100;
    // ik_srv_req.ik_request.pose_stamped.pose = waypoints[0];
    // moveit_msgs::GetPositionIK::Response ik_srv_resp;
    // ik_service_client.call(ik_srv_req, ik_srv_resp);
    // if(ik_srv_resp.error_code.val !=1){
    //   ROS_ERROR("IK couldn't find a solution (error code %d)", ik_srv_resp.error_code.val);
    //   return false;
    // }

    moveit_msgs::GetCartesianPath::Request req;
    moveit_msgs::GetCartesianPath::Response res;
    req.group_name = "arm";
    req.header.frame_id = "base_link";
    req.header.stamp = ros::Time::now();
    req.waypoints = waypoints;
    req.max_step = 0.05;
    req.jump_threshold = 0.0;
    req.avoid_collisions = true;
    robot_state::robotStateToRobotStateMsg(*group->getCurrentState(), req.start_state);
    if (!cartesian_path_service_.call(req, res))
      return false;
    if (res.error_code.val != 1) {
      ROS_ERROR("cartesian_path_service_ returned with error code %d", res.error_code.val);
      return false;
    }
    double fraction = res.fraction;
    lin_traj_msg = res.solution;

    // robot_trajectory::RobotTrajectory ret_traj(group->getCurrentState()->getRobotModel(), "excel");
    // ret_traj.setRobotTrajectoryMsg(*group->getCurrentState(), lin_traj_msg);
    // if(!full_planning_scene->isPathValid(ret_traj)) {
    //   ROS_ERROR("INVALID FINAL STATE IN VERTICAL MOVE");
    //   return false;
    // }

    // double fraction_test = group->computeCartesianPath(waypoints, 0.05, 0.0, lin_traj_test_msg, true);
    // std::printf("\n\n\n");
    // std::vector<const moveit::core::AttachedBody*> attached_bodies ;
    // full_planning_scene->getCurrentState().getAttachedBodies(attached_bodies);
    // ROS_WARN("fraction_test %f, numpoints %d, num_attached_objs %d", fraction_test, lin_traj_test_msg.joint_trajectory.points.size(), attached_bodies.size());
    // std::printf("\n\n\n");
    // if (fraction_test == -1.0) {
    //   ROS_ERROR("Cartesian path didn't return valid path");
    //   return false;
    // }

    //ROS_INFO_STREAM("currrent state "<< req.start_state);

    //ROS_INFO_STREAM("1st point for "<< lin_traj_msg.joint_trajectory.joint_names[0] <<" is " << lin_traj_msg.joint_trajectory.points[0].positions[0]);

    // create new robot trajectory object
    robot_trajectory::RobotTrajectory lin_rob_traj(group->getCurrentState()->getRobotModel(), "arm");

    //ROS_INFO_STREAM("first rail point of smooth traj " << lin_rob_traj.getFirstWayPoint().getJointPositions("table_rail_joint"));

    // copy the trajectory message into the robot trajectory object
    lin_rob_traj.setRobotTrajectoryMsg(*group->getCurrentState(), lin_traj_msg);
    //ROS_INFO_STREAM("first rail point of smooth traj " << *(lin_rob_traj.getFirstWayPoint().getJointPositions("table_rail_joint")));

    trajectory_processing::IterativeParabolicTimeParameterization iter_parab_traj_proc;
    if(!iter_parab_traj_proc.computeTimeStamps(lin_rob_traj)) {
      ROS_ERROR("Failed smoothing trajectory");
      return false;
    }
    ///*
    // put the smoothed trajectory back into the message....
    lin_rob_traj.getRobotTrajectoryMsg(lin_traj_msg);
    //*/
    MoveGroupPlan lin_traj_plan;
    lin_traj_plan.trajectory_ = lin_traj_msg;
    ROS_INFO("computeCartesianPath fraction = %f", fraction);
    if(fraction < 0.0) {
      ROS_ERROR("Failed computeCartesianPath");
      return false;
    }

    if (executeJointTrajectory(lin_traj_plan, false)) {
      ROS_INFO("Vertical joint trajectory execution successful");
      return true;
    }
    else {
      ROS_WARN("Vertical joint trajectory execution failed, going to restart");
      continue;
    }
  }
}

bool PickNPlace::attachObject(int bin_number){ 
  ROS_INFO("Attaching bin %d", bin_number);
  // close gripper
//   executeGripperAction(true, true); 

  moveit_msgs::CollisionObjectPtr bin_coll_obj = getBinCollisionObject(bin_number);

  if (bin_coll_obj) {
    ROS_INFO("Attaching the bin");
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper_base_link";
    attached_object.object = *bin_coll_obj;
    attached_object.object.operation = attached_object.object.ADD;
    attached_object_publisher.publish(attached_object);
    return true;
  } else {
    // std::string error_msg = ""+bin_name + " is not in the scene. Aborting !";
    ROS_ERROR("This bin is not in the scene.");
    return false;
  }
}

bool PickNPlace::detachObject(){
  ROS_INFO("Detaching bin");
  // open gripper
//   executeGripperAction(false, true);

  // update the planning scene to get the robot's state
  moveit_msgs::PlanningScene planning_scene;
  planning_scene::PlanningScenePtr full_planning_scene;
  getPlanningScene(planning_scene, full_planning_scene);

  if (planning_scene.robot_state.attached_collision_objects.size()>0){
    moveit_msgs::AttachedCollisionObject attached_object = planning_scene.robot_state.attached_collision_objects[0];
    moveit_msgs::GetPositionFK::Request fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;
    fk_request.header.frame_id = "base_link";
    fk_request.fk_link_names.clear();
    fk_request.fk_link_names.push_back("ati_link");
    fk_request.robot_state = planning_scene.robot_state;
    fk_service_client.call(fk_request, fk_response);

    tf::Quaternion co_quat;
    tf::quaternionMsgToTF(fk_response.pose_stamped[0].pose.orientation, co_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(co_quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM(roll);
    ROS_INFO_STREAM(pitch);
    ROS_INFO_STREAM(yaw);
    tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,yaw);

    attached_object.object.header.frame_id = "base_link";
    attached_object.object.mesh_poses[0].position = fk_response.pose_stamped[0].pose.position;
    attached_object.object.mesh_poses[0].position.z = 0.0;
    attached_object.object.mesh_poses[0].orientation.x = quat.x();
    attached_object.object.mesh_poses[0].orientation.y = quat.y();
    attached_object.object.mesh_poses[0].orientation.z = quat.z();
    attached_object.object.mesh_poses[0].orientation.w = quat.w();

    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene_diff_publisher.publish(planning_scene);
    return 1;
  }else{
    ROS_ERROR("There was no bin attached to the robot");
    return 0;
  }
}

/*--------------------------------------------------------------------
 * optimalGoalAngle()
 * Finds out if the robot needs to rotate clockwise or anti-clockwise
 *------------------------------------------------------------------*/
double PickNPlace::optimalGoalAngle(double goal_angle, double current_angle){
  while( std::abs(std::max(current_angle,goal_angle) - std::min(current_angle,goal_angle))>M_PI){
    //std::cout<<"This is not the shortest path"<<std::endl;
    if (goal_angle>current_angle){
      goal_angle -= 2*M_PI;
    }
    else{
      goal_angle += 2*M_PI;
    }
  }
  if(goal_angle>2*M_PI)
    goal_angle -= 2*M_PI;
  if(goal_angle<-2*M_PI)
    goal_angle += 2*M_PI;
  return goal_angle;
}

/*--------------------------------------------------------------------
 * avoid_human(goal_angle, current_goal, current_pose, goal_pose)
 * Finds out if the robot needs to rotate clockwise or anti-clockwise to avoid the human
 *------------------------------------------------------------------*/
double PickNPlace::avoid_human(double goal_angle, double current_angle, geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose){
  double cur_x, cur_y, goal_x, goal_y;
  cur_x = current_pose.position.x; cur_y = current_pose.position.y;
  goal_x = goal_pose.position.x; goal_y = goal_pose.position.y;

  // Robot on the human table, goal is not
  if( ((cur_x <1.0)&(cur_y>1.0)) & !((goal_x <1.0)&(goal_y>1.0)) ){
    ROS_WARN("Going from A to B");
    if (goal_angle < current_angle){
      goal_angle += 2*M_PI;
    } 

  }else{
    // Robot on not the human table, but goal is
    if( !((cur_x <1.0)&(cur_y>1.0)) & ((goal_x <1.0)&(goal_y>1.0)) ){
      ROS_WARN("Going from B to A");
      if(goal_angle > current_angle){
        goal_angle -= 2*M_PI;
      }
    }
  }
  if(goal_angle>2*M_PI){
    goal_angle -= 2*M_PI;
  }
  if(goal_angle<-2*M_PI){
    goal_angle += 2*M_PI;
  }
  return goal_angle;
}

bool PickNPlace::executeJointTrajectory(MoveGroupPlan& mg_plan, bool check_safety)
{
  std::printf("Start state/current:\n");
  if (security_stopped_ || emergency_stopped_) {
    while (security_stopped_ || emergency_stopped_) {
      ROS_WARN("Waiting for security/emergency stop to be removed");
      if (!ros::ok()) 
        return false;
      ros::Duration(0.3).sleep();
    }
    if(security_stopped_)
      ros::Duration(1.0).sleep();
    if(emergency_stopped_) {
      ROS_WARN("Starting in 5 seconds...");
      ros::Duration(5.0).sleep();
    }
  }
  // for(int i = 0; i < 7; i++)
  //   std::printf("%s, ", mg_plan.start_state_.joint_state.name[i].c_str());
  // for(int i = 0; i < 7; i++)
  //   std::printf("%.5f, ", mg_plan.start_state_.joint_state.position[i]);
  std::printf("\n");
  for(int i = 0; i < 7; i++)
    std::printf("%.5f, ", q_cur[i]);
  std::printf("\n");
  std::printf("\n");
  int num_pts = mg_plan.trajectory_.joint_trajectory.points.size();
  ROS_INFO("Executing joint trajectory with %d knots and duration %f", num_pts, 
      mg_plan.trajectory_.joint_trajectory.points[num_pts-1].time_from_start.toSec());
  if(sim)
    return group->execute(mg_plan);

  // Only sim for now
  
  // Copy trajectory
//   control_msgs::FollowJointTrajectoryGoal excel_goal;
//   excel_goal.trajectory = mg_plan.trajectory_.joint_trajectory;
// 
//   // Ask to execute now
//   excel_goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.15); 
// 
//   // Specify path and goal tolerance
//   //excel_goal.path_tolerance
// 
//   if (check_safety && ros::ok()) {
//     ROS_WARN_THROTTLE(1.0, "Human unsafe condition detected. Waiting for this to clear.");
//     ros::Duration(1.0/30.0).sleep();
//     return false;                        
//   }
//   // Send goal and wait for a result
//   excel_ac.sendGoal(excel_goal);
//   while (ros::ok()) {
//     if (excel_ac.waitForResult(ros::Duration(1.0/30.0))) 
//       break;
//     if (check_safety && human_unsafe_) {
//       ROS_WARN("Human unsafe condition detected. Stopping trajectory");
//       stopJointTrajectory();
//       ros::Duration(0.5).sleep();
//       return false;
//     }
//     if (security_stopped_ || emergency_stopped_) {
//       ROS_WARN("Robot security stopped, stopping trajectory!");
//       stopJointTrajectory();
//       ros::Duration(0.5).sleep();
//       return false;
//     }
//   }
//   actionlib::SimpleClientGoalState end_state = excel_ac.getState();
//   return end_state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void PickNPlace::stopJointTrajectory()
{
  if(sim)
    group->stop();

  ROS_INFO("Stopping joint trajectory");
  excel_ac.cancelGoal();
}

// bool PickNPlace::executeGripperAction(bool is_close, bool wait_for_result)
// {
//   if(is_close)
//     ROS_INFO("Closing gripper");
//   else
//     ROS_INFO("Opening gripper");
//   if(use_gripper) {
//     // send a goal to the action
//     control_msgs::GripperCommandGoal goal;
//     goal.command.position = (is_close) ? 0.0 : 0.08;
//     goal.command.max_effort = 100;
//     gripper_ac.sendGoal(goal);
//     if(wait_for_result)
//       return gripper_ac.waitForResult(ros::Duration(30.0));
//     else
//       return true;
//   }
//   else {
//     ros::Duration(2.0).sleep();
//     return true;
//   }
// }

void PickNPlace::getPlanningScene(moveit_msgs::PlanningScene& planning_scene, 
    planning_scene::PlanningScenePtr& full_planning_scene)
{
  planning_scene_monitor->requestPlanningSceneState();
  full_planning_scene = planning_scene_monitor->getPlanningScene();
  full_planning_scene->getPlanningSceneMsg(planning_scene);
}

moveit_msgs::CollisionObjectPtr PickNPlace::getBinCollisionObject(int bin_number)
{
  std::string bin_name = "bin#" + boost::lexical_cast<std::string>(bin_number); 

  // update the planning scene to get the robot's state
  moveit_msgs::PlanningScene planning_scene;
  planning_scene::PlanningScenePtr full_planning_scene;
  getPlanningScene(planning_scene, full_planning_scene);

  for(int i=0;i<planning_scene.world.collision_objects.size();i++){
    if(planning_scene.world.collision_objects[i].id == bin_name){ 
      return moveit_msgs::CollisionObjectPtr(new moveit_msgs::CollisionObject(planning_scene.world.collision_objects[i]));
    }
  }
  ROS_ERROR("Failed to attach the bin. Attaching an empty collision object");
  return moveit_msgs::CollisionObjectPtr();
}

void PickNPlace::getObjectAbovePose(moveit_msgs::CollisionObjectPtr bin_coll_obj, geometry_msgs::Pose& pose, 
    double& bin_height)
{
  pose = bin_coll_obj->mesh_poses[0];

  // fix height
  bin_height = bin_coll_obj->meshes[0].vertices[0].z;
  pose.position.z = GRIPPING_OFFSET+bin_height+DZ;

  // fix orientation
  tf::Quaternion co_quat(pose.orientation.x, pose.orientation.y, 
      pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(co_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  tf::Quaternion quat = tf::createQuaternionFromRPY(M_PI/2-yaw,M_PI/2,M_PI);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

/*--------------------------------------------------------------------
 * Moves to target location keeping the grasping orientation
 *------------------------------------------------------------------*/
bool PickNPlace::carryObjectTo(double x_target, double y_target, double angle_target, double bin_height)
{
  ROS_INFO("Carrying bin to target (%.3f, %.3f, %f)", x_target, y_target, angle_target);
  geometry_msgs::Pose target_pose;
  getCarryObjectPose(x_target, y_target, angle_target, bin_height, target_pose);
  return traverseMove(target_pose);
}

void PickNPlace::getCarryObjectPose(double x_target, double y_target, double angle_target, double bin_height,
    geometry_msgs::Pose& pose)
{
  tf::Quaternion quat_goal = tf::createQuaternionFromRPY(M_PI/2-angle_target*M_PI/180.0, M_PI/2, M_PI);
  pose.position.x = x_target;
  pose.position.y = y_target;
  pose.position.z = GRIPPING_OFFSET + bin_height + DZ;
  pose.orientation.x = quat_goal.x();
  pose.orientation.y = quat_goal.y();
  pose.orientation.z = quat_goal.z();
  pose.orientation.w = quat_goal.w();
}

void PickNPlace::jointStateCallback(const sensor_msgs::JointState::ConstPtr& js_msg)
{
  const char* joint_names[] = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  for(int i = 0; i < 7; i++)
    for(int j = 0; j < js_msg->position.size(); j++)
      if(js_msg->name[j].compare(joint_names[i]) == 0) {
        q_cur[i] = js_msg->position[j];
        break;
      }
}

void PickNPlace::secStoppedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  security_stopped_ = msg->data;
}
void PickNPlace::emergStoppedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  emergency_stopped_ = msg->data;
}