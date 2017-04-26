#include <gtest/gtest.h>
#include <ros/ros.h>
#include "TaskHandler.h"
using namespace XmlRpc;

class TestTaskHandler : public ::testing::Test {
public:
    TaskHandler handler;

    virtual void SetUp() {
        //need parameter server to have some tasks on it to setup actionclients.
        XmlRpcValue task_names("[explore]");
        std::vector<std::string> tasks;
        tasks.push_back("explore");
        tasks.push_back("unguided_clean_debris");

        ros::param::set("task_handler/valid_tasks", tasks);
    }

    virtual void TearDown() {}
};

TEST_F(TestTaskHandler, isAcceptableRequest_Should_ReturnFalse_When_TaskRequestsTaskNameIsNull) {
    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;

    //never supplied a request name
    EXPECT_EQ(handler.isAcceptableRequest(req), false);
}

TEST_F(TestTaskHandler,processRequest_Should_AddTaskToTaskBacklog_When_RequestIsAccepted) {
    int size_before = handler.getBacklogSize();

    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;
    req.task_name = "explore";

    EXPECT_TRUE(handler.isAcceptableRequest(req));

    //accept some task
    handler.processRequest(req);
    EXPECT_EQ(handler.getBacklogSize(),size_before+1);
    EXPECT_TRUE(handler.isTaskInBacklog(req.task_id));

    arc_msgs::TaskRequest req2;
    req2.request_type = 1;
    req2.task_id = 2;
    req2.task_name = "unguided_clean_debris";
    handler.processRequest(req2);
    EXPECT_EQ(handler.getBacklogSize(),size_before+2);
    EXPECT_TRUE(handler.isTaskInBacklog(req2.task_id));
    EXPECT_TRUE(handler.isTaskInBacklog(req.task_id));
}

TEST_F(TestTaskHandler, TaskHandler_Should_RemoveTaskRequestFromBacklog_When_TaskIsStarted) {
    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;
    req.task_name = "explore";

    EXPECT_FALSE(handler.isTaskInBacklog(req.task_id)); //task isn't in backlog yet
    handler.processRequest(req);

    EXPECT_TRUE(handler.isTaskInBacklog(req.task_id)); //now it should be in backlog.
}

TEST_F(TestTaskHandler, TaskHandler_Should_AddTaskToActiveTaskList_When_TaskIsStarted) {
    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;
    req.task_name = "explore";

    EXPECT_FALSE(handler.isTaskActive(req.task_id));
    handler.processRequest(req);
    handler.attemptToBeginNewTasks();
    EXPECT_TRUE(handler.isTaskActive(req.task_id));
}

TEST_F(TestTaskHandler, TaskHandler_Should_RemoveTaskRequestFromActiveList_When_TaskIsCompleted) {
    arc_msgs::TaskRequest req;
    req.request_type = 1;
    req.task_id = 1;
    req.task_name = "explore";

    handler.processRequest(req);
    handler.attemptToBeginNewTasks();

    int original_active_task_list_size = handler.getNumActiveTasks();
    EXPECT_TRUE(handler.isTaskActive(req.task_id));
    handler.markTaskCompleted(req.task_id);
    EXPECT_FALSE(handler.isTaskActive(req.task_id));
    EXPECT_EQ(original_active_task_list_size, handler.getNumActiveTasks()+1);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "test_task_handler");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}