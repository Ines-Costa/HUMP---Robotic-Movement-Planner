#include "../include/aros_moveit_planner/visualization_tools.hpp"

using namespace std;
using namespace ros;

namespace trajectory_planner_moveit {


VisualizationTools::VisualizationTools(NodeHandle &nh)
{
    robot_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("aros_robot_state", 1, true);
    trajectory_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("aros_robot_trajectory", 1, true);
}

VisualizationTools::~VisualizationTools()
{

}

void VisualizationTools::publish_robot_state(const moveit_msgs::RobotState &state)
{
    moveit_msgs::DisplayRobotState msg;
    msg.state = state;
    robot_state_pub.publish(msg);
}

void VisualizationTools::publish_trajectory(const moveit_msgs::RobotState &start_state,
                                            const moveit_msgs::RobotTrajectory &trajectory)
{
    vector<moveit_msgs::RobotTrajectory> trajectories;
    trajectories.push_back(trajectory);
    publish_trajectory(start_state, trajectories);
}

void VisualizationTools::publish_trajectory(const moveit_msgs::RobotState &start_state,
                                            const vector<moveit_msgs::RobotTrajectory> &trajectories)
{
    moveit_msgs::DisplayTrajectory msg;
    msg.model_id = "ARoS";
    msg.trajectory_start = start_state;

    for (size_t i = 0; i < trajectories.size(); ++i) {
        msg.trajectory.push_back(trajectories[i]);
    }

    trajectory_pub.publish(msg);
}

} // namespace trajectory_planner_moveit
