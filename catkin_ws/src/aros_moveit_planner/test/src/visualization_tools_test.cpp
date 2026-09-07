#include <gtest/gtest.h>


#include "../../include/aros_moveit_planner/trajectory_planner.hpp"
#include "../../include/aros_moveit_planner/visualization_tools.hpp"


using namespace trajectory_planner_moveit;
using namespace std;

int argcc;
char** argvv;

TEST(TestSuite_vis_tools, Test_vis_tools_goal1)
{
    ros::init(argcc, argvv, "visualization_tools_test_goal1");
    ros::NodeHandle nh;

    TrajectoryPlanner planner(nh);
    VisualizationTools visualizer(nh);
    planner.setArm("right");

    KinematicsHelper kin_helper(nh);
    moveit_msgs::MotionPlanResponse plan_response;

    geometry_msgs::Pose goal1;

    goal1.position.x = -0.26;
    goal1.position.y = 0.50;
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
        visualizer.publish_robot_state(state);
        visualizer.publish_trajectory(state,plan_response.trajectory);

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
