#include "toposens_task/parser.h"
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include "pcl_conversions/pcl_conversions.h"

namespace topo
{
  UssParser::UssParser(ros::NodeHandle nh, const std::string& fileName)
  {
    _pubPcl = nh.advertise<sensor_msgs::PointCloud2>("/topo/pointCloud", 1);

    if(fileName.empty())
    {
      ROS_WARN("No input file to read");
    }
    std::ifstream _file(fileName);
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
    _errors.clear();

    if(_data.empty())
    {
      _errors.push_back(DataFileEmpty);
      return false;
    }

    std::string frame;
    frame.clear();
    bool toPush = false;
    bool foundS = false;
    bool foundE = false;
    for(int i=_pos; i<_data.size(); ++i)
    {
      if(_data[i] == 'S')
      {
        toPush = true;
        foundS = true;
        // New frame
      }
      if(toPush)
      {
        frame.push_back(_data[i]);
      }
      if(_data[i] == 'E')
      {
        toPush = false;
        foundE = true;
        _pos = i+1;
        break;
      }
    }

    if(foundS && !foundE)
    {
      // Missing end. Don't parse
      _errors.push_back(FrameMissingE);
    }
    if(frame.empty())
    {
      // No S found
      _errors.push_back(FrameMissingS);
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
    std::string::const_iterator i = frame.begin();


    for (i; i < frame.end(); ++i)
    {
      // Find next X-tag in the frame
      while (*i != 'X')
      {
        if(*i == 'E')
        {
          // Do nothing. 
        } 
        if (++i == frame.end())
        {
          // No X found.
          _errors.push_back(FrameMissingX);
          return;
        }
      }

      pcl::PointXYZI pcl_point;
      pcl_point.x = _toNum(++i) / 1000.0;

      if (*(++i) == 'Y')
      {
        pcl_point.y = _toNum(++i) / 1000.0;
      }
      else
      {
        // No Y found
        _errors.push_back(FrameMissingY);
        return;
      }

      if (*(++i) == 'Z')
      {
        pcl_point.z = _toNum(++i) / 1000.0;
      }
      else
      {
        // No Z found
        _errors.push_back(FrameMissingZ);
        return;
      }

      if (*(++i) == 'V')
      {
        pcl_point.intensity = _toNum(++i) / 100.0;
      }
      else
      {
        // No V found
        _errors.push_back(FrameMissingV);
        return;
      }

      if (pcl_point.intensity > 0)
      {
        // Conversion to correct coordinate frame
        _currScan.points.push_back(getConvertedPoint(pcl_point));
      }
      else
      {
        // Point is not valid
        _errors.push_back(PointInvalid);
        return;
      }
    }
  }

  float UssParser::_toNum(std::string::const_iterator &i)
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
    // The coordinate frame for the USS is defined.
    // USS coordinate frame -> front (+Z), right (+X)amd down (+Y)
    // ROS coordinate frame -> front (+X), left(+Y) and up (+Z)
    // Hence to map to the ROS coordinate frame.
    // ROS (X) = USS (Z)
    // ROS (Y) = USS (-X)
    // ROS (Z) = USS (-Y)
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

    // Fill the header for the pointcloud.
    _currScan.header.stamp = pcl_conversions::toPCL(ros::Time::now());
    _currScan.header.seq = _seq;
    _currScan.header.frame_id = "toposens";
    _currScan.height = 1;
    _seq++;

    _currScan.width = _currScan.points.size();

    // Convert pointcloud object into ros pointcloud message.
    pcl::toROSMsg(_currScan, pclMsg);

    _pubPcl.publish(pclMsg);
  }

  const std::vector<InternalErrors> UssParser::getErrors() const
  {
    return _errors;
  }
}