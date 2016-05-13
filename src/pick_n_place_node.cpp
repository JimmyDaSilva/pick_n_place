#include <lwr_pick_n_place/pick_n_place.hpp>
#include <tf/transform_broadcaster.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_n_place_node");

  PickNPlace pick_n_place;
//   pick_n_place.cleanObjects();
//   usleep(1000000*1);
  
//   geometry_msgs::Pose object_pose;
//   object_pose.position.x = 0.5;
//   object_pose.position.y = 0.0;
//   object_pose.position.z = 0.12;
//   tf::Quaternion q1;
//   q1.setRPY(-M_PI/4.0, 0.0, 0.0);
//   geometry_msgs::Quaternion quat1;
//   tf::quaternionTFToMsg(q1, quat1);  
//   object_pose.orientation = quat1;
//   pick_n_place.addEpingleObject(object_pose);
//   
//   tf::Quaternion q;
//   q.setRPY(-M_PI/2.0+M_PI/4.0, M_PI/4.0, -M_PI/2.0);
//   geometry_msgs::Quaternion quat;
//   tf::quaternionTFToMsg(q, quat);  
//   object_pose.orientation = quat;
//   object_pose.position.x = 0.8;
//   object_pose.position.z = 0.5;
//   pick_n_place.addPlaqueObject(object_pose);

  
  // First: demo with set up already in place
  pick_n_place.moveToStart();
  pick_n_place.moveAboveEpingle("epingle");
  pick_n_place.attachObject("epingle");
  pick_n_place.moveToStart();
  pick_n_place.moveAbovePlaque("plaque");
  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.15;
  tf::Quaternion tf_quat;
  tf_quat.setRPY(M_PI, 0.0, 0.0);
  geometry_msgs::Quaternion geo_quat;
  tf::quaternionTFToMsg(tf_quat, geo_quat);
  pose.orientation = geo_quat;
  pick_n_place.moveToCartesianPose(pose);
  pick_n_place.detachObject();
  pick_n_place.moveToStart();
  
    
//   int run_prg = 1, first =1;
//   while(run_prg && ros::ok()){
// 
//     pick_n_place.moveAboveEpingle("epingle");
//     if (!first){
//       usleep(1000000*1);
//       pick_n_place.detachObject();
//       usleep(1000000*1);
//     }else
//       first = 0;
// 
//     pick_n_place.attachObject("epingle");
//     pick_n_place.moveToStart();
//     pick_n_place.moveAbovePlaque("plaque");
// 
//   }
  ros::shutdown();
  return 0;
}