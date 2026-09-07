
#include <moveit/kinematic_constraints/utils.h>
#include "../include/aros_moveit_planner/common.hpp"
#include "../include/aros_moveit_planner/path_trajectory_planner.hpp"

using namespace ros;

namespace trajectory_planner_moveit {

PathTrajectoryPlanner::PathTrajectoryPlanner(NodeHandle &nh):
    traj_planner(nh)
{
    ROS_INFO("Connecting to planning service...");

    string topic = "compute_cartesian_path";
    planning_client = nh.serviceClient<moveit_msgs::GetCartesianPath>(topic);

    // default values
    arm = "right";
    jump_threshold = 0.0;
    eef_step = 0.01;
}

PathTrajectoryPlanner::~PathTrajectoryPlanner()
{

}


void PathTrajectoryPlanner::setJumpThreshold(double value)
{
    jump_threshold = value;
}

double PathTrajectoryPlanner::getJumpThreshold()
{
    return jump_threshold;
}

void PathTrajectoryPlanner::setEEFStep(double value)
{
    eef_step=value;
}

double PathTrajectoryPlanner::getEEFStep()
{
    return eef_step;
}

bool PathTrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution)
{
    // execute planning with empty start state
    sensor_msgs::JointState start_state;
    return plan(goal, solution, start_state);
}

bool PathTrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state)
{
    if (!planning_client.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");
        return false;
    }

    ros::Time start_time = ros::Time::now();

    ROS_INFO("Constructing planning service request...");

    moveit_msgs::GetCartesianPathRequest request;

    request.header.frame_id = FRAME_ID;
    request.header.stamp = ros::Time::now();

    request.group_name = arm + "_arm";
    request.link_name = arm + "_hand_sensor_link";
    request.jump_threshold = jump_threshold;
    request.max_step = eef_step;
    request.avoid_collisions = true;
    request.waypoints.push_back(goal);
    request.start_state.joint_state = start_state;

    ROS_INFO("Initial call to planning service...");

    moveit_msgs::GetCartesianPathResponse response;
    // initialize with default value - gets changed if planning was successful.
    solution.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

    if(!planning_client.call(request, response)) {
        ROS_INFO_STREAM("Call to cartesian path planning service failed with status code '" << solution.error_code.val << "'");
    }

    bool success = false;

    if(!(response.fraction < 1.0)) {
        ROS_INFO("Planning successful - cartesian planning service did the job!");
        solution.trajectory_start = response.start_state;
        solution.trajectory = response.solution;
        solution.error_code = response.error_code;
        success = true;
    } else {
        ROS_WARN("Cartesian planner failed - calling default planner.");
        success = traj_planner.plan(goal, solution, start_state);
    }

    ros::Time end_time = ros::Time::now();
    ros::Duration planning_duration = end_time - start_time;
    solution.planning_time = planning_duration.toSec();

    return success;


}



} // namespace trajectory_planner_moveit
