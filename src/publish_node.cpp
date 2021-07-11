#include <ros/ros.h>
#include "toposens_task/parser.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "toposens_publish_node");

  ros::NodeHandle nh;

  std::string fileName;
  nh.getParam("data_file", fileName);

  ros::Rate loop_rate(20);  // 20 Hz

  topo::UssParser parserObj(nh, fileName);

  while (parserObj.readAndPublishFrame())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
