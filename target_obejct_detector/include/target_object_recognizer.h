

#ifndef TARGET_OBJECT_RECOGNIZER_H
#define TARGET_OBJECT_RECOGNIZER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

class TargetObject
{
public:
  TargetObject(geometry_msgs::Pose pose);
  geometry_msgs::Pose getPose();
  void addPose(geometry_msgs::Pose new_pose);
  int getCounter();
  void addCounter();
  ~TargetObject();
private:
  geometry_msgs::Pose pose_;
  int counter_;
};

class TargetObjectRecognizer
{
public:
  TargetObjectRecognizer(ros::NodeHandle nh);
  ~TargetObjectRecognizer();
  void detectedCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &detected_objects);
  void run();
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher recognized_pub_;
  ros::Subscriber detected_sub_;
  std::vector<TargetObject> target_object_candidates_;
};


#endif /* TARGET_OBJECT_RECOGNIZER_H */
