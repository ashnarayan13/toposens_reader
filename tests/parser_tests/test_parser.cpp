#include <ros/ros.h>
#include <gtest/gtest.h>

#include "toposens_task/parser.h"

class TestParser : public ::testing::Test
{

};

TEST_F(TestParser, TestFalse)
{
  EXPECT_TRUE(false);
}

TEST_F(TestParser, TestTrue)
{
  EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
