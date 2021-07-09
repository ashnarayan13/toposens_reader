#include "toposens_task/parser.h"
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include "pcl_conversions/pcl_conversions.h"

namespace topo
{
  UssParser::UssParser(ros::NodeHandle nh)
  {
    _pubPcl = nh.advertise<sensor_msgs::PointCloud2>("/topo/pointCloud", 1);
    std::ifstream _file("/home/nomad/catkin_ros/src/toposens_task/data/data.txt");
    char ch;
    while (_file >> ch)
    {
     _data += ch; // Or whatever
    }
    _pos = 0;
  }

  bool UssParser::readAndPublishFrame()
  {
    _currScan.clear();

    std::string frame;
    frame.clear();
    bool toPush = false;
    for(int i=_pos; i<_data.size(); ++i)
    {
      if(_data[i] == 'S')
      {
        toPush = true;
        // New frame
      }
      if(toPush)
      {
        frame.push_back(_data[i]);
      }
      if(_data[i] == 'E')
      {
        toPush = false;
        _pos = i+1;
        break;
      }
    }

    parse(frame);

    // Publish this frame!
    publishPcl();

    if(_pos >=_data.size())
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  void UssParser::parse(const std::string &frame)
  {
    auto i = frame.begin();

    while (*i != 'S')
    if (++i == frame.end()) return;


    for (i; i < frame.end(); ++i)
    {
      // Find next X-tag in the frame
      while (*i != 'X')
        if (++i == frame.end()) return;

      try
      {
        pcl::PointXYZI pcl_point;
        pcl_point.x = _toNum(++i) / 1000.0;

        if (*(++i) == 'Y')
          pcl_point.y = _toNum(++i) / 1000.0;
        else
          throw std::invalid_argument("Expected Y-tag not found");

        if (*(++i) == 'Z')
          pcl_point.z = _toNum(++i) / 1000.0;
        else
          throw std::invalid_argument("Expected Z-tag not found");

        if (*(++i) == 'V')
          pcl_point.intensity = _toNum(++i) / 100.0;
        else
          throw std::invalid_argument("Expected V-tag not found");

        if (pcl_point.intensity > 0)
        {
          // Conversion to correct coordinate frame
          _currScan.points.push_back(getConvertedPoint(pcl_point));
        } 
      }
      catch (const std::exception &e)
      {
        ROS_INFO("Skipped invalid point in stream");
        ROS_DEBUG("Error: %s in message %s", e.what(), frame.c_str());
      }
    }
  }

  float UssParser::_toNum(auto &i)
  {
    // Size of X, Y, Z, V data is always 5 bytes
    int abs = 0, factor = 1, length = 5;

    // Throw exception if first character is not a number or "-"
    if (*i == '-')
      factor = -1;
    else if (*i != '0')
      throw std::invalid_argument("Invalid value char");

    while (--length)
    {
      int d = *(++i) - '0';
      if (d >= 0 && d <= 9)
        abs = abs * 10 + d;
      else
        throw std::bad_cast();
    }

    return (float)(factor * abs);
  }

  pcl::PointXYZI UssParser::getConvertedPoint(const pcl::PointXYZI& currPoint)
  {
    pcl::PointXYZI pcl_point;
    pcl_point.x = currPoint.z;
    pcl_point.y = -currPoint.x;
    pcl_point.z = -currPoint.y;
    pcl_point.intensity = currPoint.intensity;

    return pcl_point;
  }

  void UssParser::publishPcl()
  {
    sensor_msgs::PointCloud2 pclMsg;

    _currScan.header.stamp = pcl_conversions::toPCL(ros::Time::now());
    _currScan.header.seq = _seq;
    _currScan.header.frame_id = "toposens";
    _currScan.height = 1;
    _seq++;

    _currScan.width = _currScan.points.size();

    pcl::toROSMsg(_currScan, pclMsg);

    _pubPcl.publish(pclMsg);
  }
}