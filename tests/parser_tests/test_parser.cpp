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
      EXPECT_EQ(cloud->width, 4);
    }
    ros::Subscriber sub_;
};

TEST_F(TestParser, TestEmptyFile)
{
  EXPECT_TRUE(true);
  ros::NodeHandle nh;
  std::string file = "";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());

}

TEST_F(TestParser, TestSingleFrame)
{
  ros::NodeHandle nh;

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

TEST_F(TestParser, TestSingleFrameFailureNoS)
{
  ros::NodeHandle nh;

  // Create test data!
  std::string data = "000000P0000X00083Y-0020Z00986V00031P0000X00130Y00526Z00966V00018P0000X00269Y01835Z01518V00029P0000X-1825Y-2501Z01996V00072E";
  std::ofstream writeFile("test.txt");
  writeFile << data;
  writeFile.close();

  std::string file = "test.txt";
  topo::UssParser ussParser(nh, file);
  EXPECT_FALSE(ussParser.readAndPublishFrame());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
