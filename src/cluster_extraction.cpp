#include "toposens_task/cluster_extraction.h"
#include <std_msgs/Bool.h>
#include <pcl/PointIndices.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>

namespace topo
{
  ClusterExtractor::ClusterExtractor(ros::NodeHandle nh)
  {
    _pub = nh.advertise<std_msgs::Bool>("topo/poleStatus", 1);
    _sub_input = nh.subscribe("/topo/pointCloud", 1, &ClusterExtractor::inputCallback, this);

    float clusterTolerance = 1.0F;
    int minClusterSize = 2;
    int maxClusterSize = 5;

    nh.getParam("cluster_tolerance", clusterTolerance);
    nh.getParam("min_cluster_size", minClusterSize);
    nh.getParam("max_cluster_size", maxClusterSize);
    _impl.setClusterTolerance(clusterTolerance);
    _impl.setMinClusterSize(minClusterSize);
    _impl.setMaxClusterSize(maxClusterSize);
  }

  void ClusterExtractor::inputCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *convertedCloud);
    std::vector<pcl::PointIndices> clusters;

    _impl.setInputCloud(convertedCloud);
    _impl.extract(clusters);

    std::vector<pcl::PointXYZ> pts;
    std_msgs::Bool msg;
    if(clusters.size() > 0)
    {
      ROS_INFO("Cluster size %ld", clusters.size());
      for(int i=0; i<clusters.size(); ++i)
      {
        for(int j=0; j<clusters[i].indices.size(); ++j)
        {
          pts.push_back(convertedCloud->points[clusters[i].indices[j]]);
        }
      }

      // We expect that the object is in front of the sensor!
      // X > 0 && X < 5.0F
      // object is in level of the sensor z +- 0.5F
      // Object is within some FOV y +- 5.0F
      for(int i=0; i<pts.size(); ++i)
      {
        if((pts[i].x > 0.0F) && (pts[i].x < 5.0F))
        {
          if((pts[i].y < 5.0F) && (pts[i].y > -5.0F))
          {
            if((pts[i].z > -1.0F) && (pts[i].z < 1.0F))
            {
              msg.data = true;
            }
          }
        }
      }
    }
    else
    {
      msg.data = false;
      ROS_INFO("NO CLUSTERS FOUND!");
    }


    _pub.publish(msg);
  }
}