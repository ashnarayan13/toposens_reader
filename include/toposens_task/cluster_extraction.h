#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace topo
{
  class ClusterExtractor
  {
    public:
      ClusterExtractor(ros::NodeHandle nh);
      ~ClusterExtractor() {}

    private:

      void inputCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

      /** \brief The PCL implementation used. */
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> _impl;

      /** \brief The input PointCloud subscriber. */
      ros::Subscriber _sub_input;

      ros::Publisher _pub;
  };
}