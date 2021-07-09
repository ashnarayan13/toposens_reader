#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace topo
{
  /// @brief Class to extract the euclidean clusters from the input pointcloud.
  ///   Uses the PCL euclidean cluster extraction.
  ///   Publishes a pole status flag if it detects an object in front of the sensor.
  class ClusterExtractor
  {
    public:
      ClusterExtractor(ros::NodeHandle nh);
      ~ClusterExtractor() {}

    private:

      /// @brief The clusters are extracted from the input point cloud.
      ///   A flag is published with status true if a cluster is found. False otherwise.
      ///
      /// @param[in] cloud - const reference to the subscribed point cloud.
      void inputCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> _impl; /// The PCL implementation for euclidean cluster extraction.

      ros::Subscriber _sub_input; /// The pointcloud subscriber.

      ros::Publisher _pub; /// The publisher of the bool flag.
  };
}