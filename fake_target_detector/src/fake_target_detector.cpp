#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <boost/tokenizer.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <ros/package.h>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

class FakeTargetDetector
{
public:
  FakeTargetDetector():
    rate_(10)
  {
    std::string filename;

    ros::NodeHandle n("~");
    n.param<std::string>("waypointsfile", filename,
                         ros::package::getPath("fake_target_detector")
                         + "/targetfiles/targetlist.csv");
    readFakeTargetsList(filename);
    recognized_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/recognized_result", 1);
    tf_frame_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&FakeTargetDetector::tfSendTransformCallback, this, _1));
  }
  int readFakeTargetsList(std::string filename)
  {
    const int rows_num = 2; // x, y
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(filename.c_str());
    std::string line;
    while(ifs.good()){
      getline(ifs, line);
      if(line.empty()){ break; }
      tokenizer tokens(line, sep);
      std::vector<double> data;
      tokenizer::iterator it = tokens.begin();
      for(; it != tokens.end() ; ++it){
        std::stringstream ss;
        double d;
        ss << *it;
        ss >> d;
        data.push_back(d);
      }
      if(data.size() != rows_num){
        ROS_ERROR("Row size is mismatch!!");
        return -1;
      }else{
        jsk_recognition_msgs::BoundingBox box;
        box.header.frame_id ="map";
        box.header.stamp = ros::Time::now();
        box.pose.position.x = data[0];
        box.pose.position.y = data[1];
        box.pose.position.z = 0;
        box.pose.orientation.x = 0;
        box.pose.orientation.y = 0;
        box.pose.orientation.z = 0;
        box.pose.orientation.w = 1;
        box.dimensions.x = 1.0;
        box.dimensions.y = 1.0;
        box.dimensions.z = 1.0;
        box_array_.boxes.push_back(box);
      }
    }
    return 0;
  }
  void tfSendTransformCallback(const ros::TimerEvent&)
  {  
    tf::Transform t;
    ros::Time time = ros::Time::now();

    for (size_t i = 0; i < box_array_.boxes.size(); ++i) {
      std::stringstream s;
      s << "fake_target_" << i;
      t.setOrigin(tf::Vector3(box_array_.boxes[i].pose.position.x,
                              box_array_.boxes[i].pose.position.y,
                              box_array_.boxes[i].pose.position.z));
      t.setRotation(tf::Quaternion(box_array_.boxes[i].pose.orientation.x,
                                   box_array_.boxes[i].pose.orientation.y,
                                   box_array_.boxes[i].pose.orientation.z,
                                   box_array_.boxes[i].pose.orientation.w));
      br_.sendTransform(tf::StampedTransform(t, time, "map", s.str()));
    }
  }

  void fakePublisher(){
    box_array_.header.stamp = ros::Time::now();
    box_array_.header.frame_id = "map";
    recognized_pub_.publish(box_array_);
  }
  void run(){
    while(nh_.ok()){
      this->fakePublisher();
      ros::spinOnce();
      rate_.sleep();
    }
  }
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher recognized_pub_;
  jsk_recognition_msgs::BoundingBoxArray box_array_;
  tf::TransformBroadcaster br_;
  ros::Timer tf_frame_timer_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fake_target_detector");
  
  FakeTargetDetector fake_target_detector;
  fake_target_detector.run();
  return 0;
}
