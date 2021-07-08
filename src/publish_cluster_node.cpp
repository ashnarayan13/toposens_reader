#include <ros/ros.h>
#include "toposens_task/cluster_extraction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "toposens_publish_cluster_node");

  ros::NodeHandle nh;
  topo::ClusterExtractor clusterObj(nh);
  ros::spin();
  return 0;
}
