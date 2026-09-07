#ifndef VISUALIZATIONTOOLS_H
#define VISUALIZATIONTOOLS_H

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace std;
using namespace ros;

namespace trajectory_planner_moveit {
//! The VisualizationTools class
/**
 * @brief The VisualizationTools class
 * This class defines the functionalities to display one or more trajectories and the robot state
 */
class VisualizationTools
{

private:

    ros::Publisher robot_state_pub; /**< publisher of the robot state */
    ros::Publisher trajectory_pub;/**< publisher of the trajectory */

public:
    /**
     * @brief VisualizationTools, a constructor
     * @param nh
     */
    VisualizationTools(ros::NodeHandle &nh);

    /**
     * @brief ~VisualizationTools, a destructor
     */
    ~VisualizationTools();

    /**
     * @brief publish_robot_state
     * It publishes the robot state
     * @param state
     */
    void publish_robot_state(const moveit_msgs::RobotState &state);
    /**
     * @brief publish_trajectory
     * It publishes the trajectory of the arm
     * @param start_state
     * @param trajectory
     */
    void publish_trajectory(const moveit_msgs::RobotState &start_state,
                            const moveit_msgs::RobotTrajectory &trajectory);

    /**
     * @brief publish_trajectory
     * It publishes the trajectories of the arm
     * @param start_state
     * @param trajectories
     */
    void publish_trajectory(const moveit_msgs::RobotState &start_state,
                            const vector<moveit_msgs::RobotTrajectory> &trajectories);


};

} // namespace trajectory_planner_moveit

#endif // VISUALIZATIONTOOLS_H
