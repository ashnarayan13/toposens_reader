#include <ros/ros.h>
#include <gtest/gtest.h>
#include <string>
#include <iostream>

#include "toposens_task/parser.h"
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

class TestParser : public ::testing::Test
{
  public:
   void SetUp()
   {
     ros::NodeHandle nh;
     sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/topo/pointCloud", 1, &TestParser::testCallRes, this);
   }

   void TearDown()
   {
     ros::spinOnce();
   }

  protected:

    void testCallRes(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
      std::string frameId = cloud->header.frame_id;
      EXPECT_EQ(frameId, "toposens");
      EXPECT_EQ(cloud->width, expectedPts_);
      EXPECT_EQ(cloud->height, 1);
    }
    ros::Subscriber sub_; // Subscribe to the point cloud message.
    int expectedPts_; // Expected number of points published in the pointcloud.
};


/// @brief Test behavior when file is empty to read.
///   Expect that parser works but no data is published.
TEST_F(TestParser, TestEmptyFile)
{
  expectedPts_ = 0;
  EXPECT_TRUE(true);
  ros::NodeHandle nh;
  std::string file = "";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());

}


/// @brief Test behavior where a single valid frame is provided.
///   Expect 0 errors in the error queue.
///   Message published successfully.
TEST_F(TestParser, TestSingleFrame)
{
  ros::NodeHandle nh;
  expectedPts_ = 4;

  // Create test data!
  std::string data = "S000000P0000X00083Y-0020Z00986V00031P0000X00130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());

  // No errors!
  const std::vector<topo::InternalErrors>& res = ussParser.getErrors();
  EXPECT_EQ(0, res.size());
}

/// @brief Test single frame where the start character S is missing.
///   Expect error for not finding S.
TEST_F(TestParser, TestSingleFrameFailureNoS)
{
  ros::NodeHandle nh;
  expectedPts_ = 0;

  // Create test data!
  std::string data = "000000P0000X00083Y-0020Z00986V00031P0000X00130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
  EXPECT_EQ(1U, ussParser.getErrors().size());
  bool expectS = false;
  for(int i=0; i<ussParser.getErrors().size(); ++i)
  {
    if(ussParser.getErrors().at(i) == topo::FrameMissingS)
    {
      expectS = true;
      break;
    }
  }
  EXPECT_TRUE(expectS);
}

/// @brief Frame with X missing for 1 point.
///   No error is generated. Since 3 valid points are availble.
TEST_F(TestParser, TestSingleFrameNoFailureMissingX)
{
  ros::NodeHandle nh;
  expectedPts_ = 3;

  // Create test data!
  std::string data = "S000000P0000X00083Y-0020Z00986V00031P0000500130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
  EXPECT_EQ(0U, ussParser.getErrors().size());

  // Until we find a X we will keep searching.
  // So only case where FrameMissingX is possible is if we miss X in the full frame.
}

/// @brief Frame with no X in any point. 
///   Expect error FrameMissingX and no points.
TEST_F(TestParser, TestingFailureForX)
{
  ros::NodeHandle nh;
  expectedPts_ = 0;

  // Create test data!
  std::string data = "S000000P0000500083Y-0020Z00986V00031P0000600130Y00526Z00966V00018P0000900269Y01835Z01518V00029P00002-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());

  const std::vector<topo::InternalErrors>& res = ussParser.getErrors();
  EXPECT_EQ(1, res.size());

  bool expectX = false;
  for(int i=0; i<ussParser.getErrors().size(); ++i)
  {
    if(ussParser.getErrors().at(i) == topo::FrameMissingX)
    {
      expectX = true;
      break;
    }
  }
  EXPECT_TRUE(expectX);
}

/// @brief Frame with missing Y in one point. 
///   Expect error FrameMissingY and 3 points published.
TEST_F(TestParser, TestSingleFrameFailureNoY)
{
  ros::NodeHandle nh;

  // Create test data!
  std::string data = "S000000P0000X00083Y-0020Z00986V00031P0000X00130500526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
  EXPECT_EQ(1U, ussParser.getErrors().size());bool expectX = false;
  bool expectY = false;
  for(int i=0; i<ussParser.getErrors().size(); ++i)
  {
    if(ussParser.getErrors().at(i) == topo::FrameMissingY)
    {
      expectY = true;
      break;
    }
  }
  EXPECT_TRUE(expectY);
}

/// @brief Frame with missing Z in one point. 
///   Expect error FrameMissingZ and 3 points published.
TEST_F(TestParser, TestSingleFrameFailureNoZ)
{
  ros::NodeHandle nh;

  // Create test data!
  std::string data = "S000000P0000X00083Y-0020Z00986V00031P0000X00130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501801996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
  EXPECT_EQ(1U, ussParser.getErrors().size());bool expectX = false;
  bool expectZ = false;
  for(int i=0; i<ussParser.getErrors().size(); ++i)
  {
    if(ussParser.getErrors().at(i) == topo::FrameMissingZ)
    {
      expectZ = true;
      break;
    }
  }
  EXPECT_TRUE(expectZ);
}

/// @brief Frame with missing V in one point. 
///   Expect error FrameMissingV and 3 points published.
TEST_F(TestParser, TestSingleFrameFailureNoV)
{
  ros::NodeHandle nh;

  // Create test data!
  std::string data = "S000000P0000X00083Y-0020Z00986V00031P0000X00130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996500072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
  EXPECT_EQ(1U, ussParser.getErrors().size());bool expectX = false;
  bool expectV = false;
  for(int i=0; i<ussParser.getErrors().size(); ++i)
  {
    if(ussParser.getErrors().at(i) == topo::FrameMissingV)
    {
      expectV = true;
      break;
    }
  }
  EXPECT_TRUE(expectV);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
