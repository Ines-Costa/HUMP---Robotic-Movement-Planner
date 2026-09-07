#include "../include/aros_moveit_planner/trajectory_planner.hpp"
#include <moveit/kinematic_constraints/utils.h>

using namespace std;
using namespace ros;
using namespace moveit_planning;

namespace trajectory_planner_moveit {


TrajectoryPlanner::TrajectoryPlanner(NodeHandle nh):
    kin_helper(nh)
{

    ROS_INFO("Connecting to planning service...");
    string topic = "plan_kinematic_path"; // use the service /plan_kinematic_path from the package ompl_planning
    planning_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(topic);

    // default values
    arm = "right";
    plan_time = 5.0;
    plan_attempts = ATTEMPTS;
    max_traj_pts = 50;
    goal_joint_tolerance = 1e-4;
    goal_position_tolerance = 1e-4; // 0.1 mm
    goal_orientation_tolerance = 1e-3; // ~0.1 deg
    plan_id = "";
}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

void TrajectoryPlanner::setPlannerId(const string &planner_id)
{
    plan_id=planner_id;
}

string TrajectoryPlanner::getPlannerId()
{
    return plan_id;
}


void TrajectoryPlanner::setAllowedPlanningTime(double value)
{
    plan_time=value;
}

double TrajectoryPlanner::getAllowedPlanningTime()
{
    return plan_time;
}

void TrajectoryPlanner::setPlanningAttempts(int value)
{
    plan_attempts=value;
}

int TrajectoryPlanner::getPlanningAttempts()
{
    return plan_attempts;
}

void TrajectoryPlanner::setMaxTrajectoryPoints(int value)
{
    max_traj_pts=value;
}

int TrajectoryPlanner::getMaxTrajectoryPoints()
{
    return max_traj_pts;
}


void TrajectoryPlanner::setGoalJointTol(double tol)
{
    goal_joint_tolerance=tol;
}

double TrajectoryPlanner::getGoalJointTol()
{
    return goal_joint_tolerance;
}

void TrajectoryPlanner::setGoalPosTol(double tol)
{
    goal_position_tolerance=tol;
}

double TrajectoryPlanner::getGoalPosTol()
{
    return goal_position_tolerance;
}

void TrajectoryPlanner::setGoalOrTol(double tol)
{
    goal_orientation_tolerance=tol;
}

double TrajectoryPlanner::getGoalOrTol()
{
    return goal_orientation_tolerance;
}

bool TrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution)
{
    // execute planning with empty start state
    sensor_msgs::JointState start_state;
    return plan(goal, solution, start_state);
}


bool TrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state)
{
    if (!planning_client.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");
        return false;
    }


    moveit_msgs::GetMotionPlanRequest get_mp_request;
    moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

    request.group_name = arm + "_arm";
    request.num_planning_attempts = plan_attempts;
    request.allowed_planning_time = plan_time;
    request.planner_id = plan_id;
    request.start_state.joint_state = start_state;

    ROS_DEBUG("Computing possible IK solutions for goal pose");

    vector<string> joint_names;
    getArmJointNames(arm, joint_names);

    // compute a set of ik solutions and construct goal constraint
    for (int i = 0; i < 5; ++i) {
        moveit_msgs::RobotState ik_solution;

        geometry_msgs::PoseStamped pose_goal;
        pose_goal.header.stamp = ros::Time::now();
        pose_goal.header.frame_id = FRAME_ID;
        pose_goal.pose = goal;

        if(kin_helper.computeIK(arm, pose_goal, start_state, ik_solution)) {
            vector<double> values;
            getJointPositionsFromState(joint_names, ik_solution, values);

            moveit_msgs::Constraints c;
            c.joint_constraints.resize(joint_names.size());

            for (size_t j = 0; j < joint_names.size(); ++j) {
                moveit_msgs::JointConstraint &jc = c.joint_constraints[j];
                jc.joint_name = joint_names[j];
                jc.position = values[j];
                jc.tolerance_above = 1e-4;
                jc.tolerance_below = 1e-4;
                jc.weight = 1.0;
            }
            request.goal_constraints.push_back(c);
        }
    }

    if(request.goal_constraints.size() == 0) {
        ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
        return false;
    }

    ROS_DEBUG("Found %d valid IK solutions for given pose goal", (int)request.goal_constraints.size());

    ROS_DEBUG("Calling planning service...");

    moveit_msgs::GetMotionPlanResponse get_mp_response;

    bool success = planning_client.call(get_mp_request, get_mp_response);
    solution = get_mp_response.motion_plan_response;

    if(success) {
        int pts_count = (int)solution.trajectory.joint_trajectory.points.size();
        int error_code = solution.error_code.val;

        if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
            return false;
        }

        if(pts_count > max_traj_pts) {
            ROS_WARN("Valid solution found but contains to many points.");
            return false;
        }

        ROS_DEBUG("Solution found for planning problem .");
        return true;

    } else {
        ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
        return false;
    }


}

}// namespace trajectory_planner_moveit
