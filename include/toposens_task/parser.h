
#include <ros/ros.h>
#include <string>
#include "toposens_task/TPoint.h"
#include<vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace topo
{
  static int val = 0;
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

      ros::Publisher _pubPcl;
      std::string _data;
      int _pos;
      std::vector<toposens_task::TPoint> _scan;
      unsigned long long _seq;
  };
}
