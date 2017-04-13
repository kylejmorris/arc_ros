#include <gtest/gtest.h>
#include <ros/ros.h>
#include "TaskHandler.h"

class TestTaskHandler : public ::testing::Test {
public:
    TaskHandler handler;

    virtual void SetUp() {}

    virtual void TearDown() {}
};

TEST_F(TestTaskHandler, isAcceptableRequest_Should_ReturnFalse_When_TaskRequestsTaskNameIsNull) {
    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;

    //never supplied a request name
    EXPECT_EQ(handler.isAcceptableRequest(req), false);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "test_task_handler");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}