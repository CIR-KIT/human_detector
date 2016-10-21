#include <target_object_recognizer.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "TargetObjectRecognizerNode");
  ros::NodeHandle nh;

  TargetObjectRecognizer recognizer(nh);
  recognizer.run();

  return 0;
}
