#ifndef PATH_TRAJECTORY_PLANNER_HPP
#define PATH_TRAJECTORY_PLANNER_HPP

#include <moveit_msgs/GetCartesianPath.h>
#include "trajectory_planner.hpp"
#include "base_planner.hpp"

using namespace std;

namespace trajectory_planner_moveit {

//! The PathTrajectoryPlanner class
/**
 * @brief The PathTrajectoryPlanner class
 * This class defines the functionalities of computing a cartesian path
 * following waypoints of the end-effector.
 * It uses the service /compute_cartesian_path
 */
class PathTrajectoryPlanner: public BasePlanner
{

private:

    double jump_threshold; /**< minimum distance allowade as change in the configuration space of the robot */
    double eef_step; /**< distance between two consecutive waypoints of the end-effector */
    TrajectoryPlanner traj_planner; /**< trajectory planner */

    ros::ServiceClient planning_client; /**< ros client for planning operations */


public:

    /**
     * @brief PathTrajectoryPlanner, a constructor
     * @param nh
     */
    PathTrajectoryPlanner(ros::NodeHandle &nh);

    /**
     * @brief ~PathTrajectoryPlanner, a destructor
     */
    ~PathTrajectoryPlanner();

    /**
     * @brief setJumpThreshold
     * It sets the jump threshold
     * @param value
     */
    void setJumpThreshold(double value);

    /**
     * @brief getJumpThreshold
     * It gets the jump threshold
     * @return
     */
    double getJumpThreshold();

    /**
     * @brief setEEFStep
     * It sets the eef step
     * @param value
     */
    void setEEFStep(double value);

    /**
     * @brief getEEFStep
     * It gets the eef step
     * @return
     */
    double getEEFStep();


    /**
     * @brief plan
     * It plans the trajectory towards the goal
     * @param goal
     * @param solution
     * @return
     */
    bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution);

    /**
     * @brief plan
     * It plans the trajectory towards the goal starting from start_state
     * @param goal
     * @param solution
     * @param start_state
     * @return
     */
    bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);




};



} // namespace trajectory_planner_moveit




#endif // PATH_TRAJECTORY_PLANNER_HPP
