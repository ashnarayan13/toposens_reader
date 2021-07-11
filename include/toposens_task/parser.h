
#include <ros/ros.h>
#include <string>
#include<vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace topo
{

  enum InternalErrors
  {
    DataFileEmpty,
    FrameMissingS,
    FrameMissingE,
    FrameMissingX,
    FrameMissingY,
    FrameMissingZ,
    FrameMissingV,
    PointInvalid
  };

  /// @brief Class for reading a data file and publishing the pointcloud message
  class UssParser
  {
    public:

      UssParser(ros::NodeHandle nh, const std::string& fileName);
      ~UssParser() {}

      /// @brief Reads the _data string frame by frame and publishes the pointcloud.
      bool readAndPublishFrame();

      const std::vector<InternalErrors> getErrors() const;

    private:

      /// @brief Given a frame. The method fill the _pubPcl object with 3d points from the frame.
      ///
      /// @param[in] frame - const reference to the current frame string.
      void parse(const std::string& frame);

      /// @brief Given a string iterator. 
      ///   Read the string and get the float number based on the defined format.
      ///
      /// @param[in] i - Reference to the string iterator.
      /// @return float - The float value computed from the string.
      float _toNum(std::string::const_iterator &i);

      void publishPcl();

      /// @brief Convert the inputs toposens point from the toposens coordinate frame
      ///   to the ROS coordinate frame.
      ///
      /// @param[in] currPoint - const ref to the point in the toposens frame.
      /// @return pcl::PointXYZI - pcl point in the ROS coordinate fram.
      pcl::PointXYZI getConvertedPoint(const pcl::PointXYZI& currPoint);

      ros::Publisher _pubPcl; /// Publisher object to publish the point cloud.
      std::string _data; /// The string that is filled from the parsing of the data file.
      int _pos; /// The position of the index in the _data.
      pcl::PointCloud<pcl::PointXYZI> _currScan; /// The point cloud object that is published.
      unsigned long long _seq; /// To store the current sequence number.

      std::vector<InternalErrors> _errors;
  };
}
