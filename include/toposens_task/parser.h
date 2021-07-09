
#include <ros/ros.h>
#include <string>
#include<vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace topo
{
  class UssParser
  {
    public:
      UssParser(ros::NodeHandle nh);
      ~UssParser() {}

      bool readAndPublishFrame();

    private:

      void parse(const std::string& frame);

      float _toNum(auto &i);

      void publishPcl();

      pcl::PointXYZI getConvertedPoint(const pcl::PointXYZI& currPoint);

      ros::Publisher _pubPcl;
      std::string _data;
      int _pos;
      pcl::PointCloud<pcl::PointXYZI> _currScan;
      unsigned long long _seq;
  };
}
