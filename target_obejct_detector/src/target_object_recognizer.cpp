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
  return pose_;
}

void TargetObject::addPose(geometry_msgs::Pose new_pose)
{
  pose_ = new_pose;
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
}

TargetObjectRecognizer::~TargetObjectRecognizer()
{
}

void TargetObjectRecognizer::detectedCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &detected_objects)
{
  for (size_t i = 0; i < detected_objects->boxes.size(); ++i) {
    target_object_candidates_.push_back(detected_objects->boxes[i].pose);
  }

}

void TargetObjectRecognizer::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
