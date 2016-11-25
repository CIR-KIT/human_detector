

#ifndef TARGET_OBJECT_RECOGNIZER_H
#define TARGET_OBJECT_RECOGNIZER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>

class TargetObject
{
public:
  TargetObject(geometry_msgs::PoseStamped transed_pose,
               jsk_recognition_msgs::BoundingBox box);
  geometry_msgs::Pose getPose();
  void addTargetObject(geometry_msgs::PoseStamped transed_pose,
                       jsk_recognition_msgs::BoundingBox new_box);
  int getCounter();
  void addCounter();
  jsk_recognition_msgs::BoundingBox getBox();
  ~TargetObject();
private:
  jsk_recognition_msgs::BoundingBox box_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Vector3 dimensions_;
  int counter_;
};

class TargetObjectRecognizer
{
public:
  TargetObjectRecognizer(ros::NodeHandle nh);
  ~TargetObjectRecognizer();
  void detectedCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &detected_objects);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
  void run();
private:
  double calcDistance(geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2);
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher recognized_pub_;
  ros::Subscriber detected_sub_;
  ros::Subscriber robotpose_sub_;
  tf::TransformListener tf_;
  geometry_msgs::PoseWithCovarianceStamped latest_robot_pose_;
  std::vector<TargetObject> target_object_candidates_;
  boost::mutex robot_pose_mutex_;
};


#endif /* TARGET_OBJECT_RECOGNIZER_H */
