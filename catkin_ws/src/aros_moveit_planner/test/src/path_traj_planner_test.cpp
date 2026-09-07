#include <gtest/gtest.h>

#include "../../include/aros_moveit_planner/path_trajectory_planner.hpp"


using namespace trajectory_planner_moveit;
using namespace std;

int argcc;
char** argvv;

TEST(TestSuite_path_traj_planner, Test_path_traj_goal1)
{
    ros::init(argcc, argvv, "path_traj_planner_test_goal1");
    ros::NodeHandle nh;

    PathTrajectoryPlanner planner(nh);
    planner.setArm("right");

    KinematicsHelper kin_helper(nh);
    moveit_msgs::MotionPlanResponse plan_response;

    geometry_msgs::Pose goal1;

    goal1.position.x = -0.26;
    goal1.position.y = 0.70;
    goal1.position.z = 1.25;
    goal1.orientation.x = 0.755872;
    goal1.orientation.y = -0.612878;
    goal1.orientation.z = -0.0464803;
    goal1.orientation.w = 0.22556;

    bool success=false;

    if(planner.plan(goal1, plan_response)) {
        ROS_INFO("Plan to goal1 found");
        moveit_msgs::RobotState state;
        geometry_msgs::Pose pose;

        state.joint_state.name = plan_response.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = plan_response.trajectory.joint_trajectory.points.back().positions;

        if(kin_helper.computeFK(state, "right_hand_sensor_link", pose)) {
            ROS_INFO("Target pose:     [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", goal1.position.x, goal1.position.y, goal1.position.z, goal1.orientation.x, goal1.orientation.y, goal1.orientation.z, goal1.orientation.w);
            ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
        success=true;
    } else {
        ROS_ERROR("Planning for goal1 failed!");
    }




    EXPECT_TRUE(success);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc,argv);
    argcc=argc;
    argvv=argv;
    return RUN_ALL_TESTS();
}
