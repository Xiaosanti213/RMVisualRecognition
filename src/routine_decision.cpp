#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"


MYNTEYE_USE_NAMESPACE


using namespace cv;
using namespace std;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "routine_decision");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("camera/messages_test", 1000, chatterCallback);
  ros::spin();
  return 0;
}
