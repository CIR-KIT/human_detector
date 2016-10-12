#include <target_object_recognizer.h>

TargetObject::TargetObject(geometry_msgs::PoseStamped transed_pose,
                           jsk_recognition_msgs::BoundingBox box)
  : box_(box), counter_(0)
{
  box_.header = transed_pose.header;
  box_.pose = transed_pose.pose;
}

TargetObject::~TargetObject()
{
}

geometry_msgs::Pose TargetObject::getPose()
{
  return box_.pose;
}

jsk_recognition_msgs::BoundingBox TargetObject::getBox()
{
  box_.header.frame_id = "map";
  return box_;
}

void TargetObject::addTargetObject(geometry_msgs::PoseStamped transed_pose,
                                   jsk_recognition_msgs::BoundingBox new_box)
{
  jsk_recognition_msgs::BoundingBox box_buf; geometry_msgs::Pose buf;
  box_buf.pose.position.x = box_.pose.position.x + transed_pose.pose.position.x;
  box_buf.pose.position.y = box_.pose.position.y + transed_pose.pose.position.y;
  box_buf.pose.orientation.x = box_.pose.orientation.x + transed_pose.pose.orientation.x;
  box_buf.pose.orientation.y = box_.pose.orientation.y + transed_pose.pose.orientation.y;
  box_buf.pose.orientation.z = box_.pose.orientation.z + transed_pose.pose.orientation.z;
  box_buf.pose.orientation.w = box_.pose.orientation.w + transed_pose.pose.orientation.w;
  box_buf.dimensions.x = box_.dimensions.x + new_box.dimensions.x;
  box_buf.dimensions.y = box_.dimensions.y + new_box.dimensions.y;
  box_buf.dimensions.z = box_.dimensions.z + new_box.dimensions.z;
  box_.pose.position.x = box_buf.pose.position.x / 2.0;
  box_.pose.position.y = box_buf.pose.position.y / 2.0;
  box_.pose.orientation.x = box_buf.pose.orientation.x / 2.0;
  box_.pose.orientation.y = box_buf.pose.orientation.y / 2.0;
  box_.pose.orientation.z = box_buf.pose.orientation.z / 2.0;
  box_.pose.orientation.w = box_buf.pose.orientation.w / 2.0;
  box_.dimensions.x = box_buf.dimensions.x / 2.0;
  box_.dimensions.y = box_buf.dimensions.y / 2.0;
  box_.dimensions.z = box_buf.dimensions.z / 2.0;
  // box_.pose = transed_pose.pose;
  // box_.header = transed_pose.header;
  // box_.dimensions = new_box.dimensions;
  //  pose_ = new_pose;
}

int TargetObject::getCounter()
{
  return counter_;
}

void TargetObject::addCounter()
{
  counter_++;
}

TargetObjectRecognizer::TargetObjectRecognizer(ros::NodeHandle nh)
  : nh_(nh),
    rate_(10)
{
  detected_sub_ = nh_.subscribe("/clustering_result", 1, &TargetObjectRecognizer::detectedCallback, this);
  recognized_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/recognized_result", 1);
}

TargetObjectRecognizer::~TargetObjectRecognizer()
{
}

void TargetObjectRecognizer::detectedCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &detected_objects)
{
  for (size_t i = 0; i < detected_objects->boxes.size(); ++i) {
    bool is_stored(false);
    geometry_msgs::PoseStamped in_pose;
    in_pose.pose = detected_objects->boxes[i].pose;
    in_pose.header = detected_objects->boxes[i].header;
    geometry_msgs::PoseStamped out_pose;
    try {
      tf_.transformPose("map", in_pose, out_pose);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
    for (size_t j = 0; j < target_object_candidates_.size(); ++j) {
      double dist = this->calcDistance(target_object_candidates_[j].getPose(),
                                       out_pose.pose);
      if (dist < 1.0) {
        target_object_candidates_[j].addCounter();
        target_object_candidates_[j].addTargetObject(out_pose,
                                                     detected_objects->boxes[i]);
        is_stored = true;
      }
    }
    if (! is_stored) {
      target_object_candidates_.push_back(TargetObject(out_pose,
                                                       detected_objects->boxes[i]));
    }
  }
  ROS_INFO_STREAM("target object candidates : " << target_object_candidates_.size());
  jsk_recognition_msgs::BoundingBoxArray box_array;
  for (size_t i = 0; i < target_object_candidates_.size(); ++i) {
    if(target_object_candidates_[i].getCounter() > 5){
      box_array.boxes.push_back(target_object_candidates_[i].getBox());
    }
  }
  box_array.header.stamp = ros::Time::now();
  box_array.header.frame_id = "map";
  recognized_pub_.publish(box_array);
}

void TargetObjectRecognizer::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}

double TargetObjectRecognizer::calcDistance(geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2)
{
  double dist = sqrt(pow(pose_1.position.x - pose_2.position.x, 2)
                     + pow(pose_1.position.y - pose_2.position.y, 2));
  return dist;
}

