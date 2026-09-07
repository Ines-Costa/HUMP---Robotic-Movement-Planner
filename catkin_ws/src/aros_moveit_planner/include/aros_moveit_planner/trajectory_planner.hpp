#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetMotionPlan.h>

#include "kinematics_helper.hpp"
#include "base_planner.hpp"

using namespace std;

namespace trajectory_planner_moveit {

//! The TrajectoryPlanner class
/**
 * @brief The TrajectoryPlanner class
 * This class defines the functionalities of planning a trajectory to a goal pose.
 * It uses the service /plan_kinematic_path
 */
class TrajectoryPlanner: public BasePlanner {

private:

    double plan_time; /**< maximum time allowed for planning [sec] */
    int plan_attempts; /**< numbers of planning attempts */
    string plan_id; /**< identity of the planner */
    string arm; /**< arm that is object of planning*/
    int max_traj_pts; /**< maximum points of the trajectory*/
    double goal_joint_tolerance; /**< tolerance on the goal joint position [rad]*/
    double goal_position_tolerance; /**< tolerance on the goal end-effector position [m] */
    double goal_orientation_tolerance; /**< tolerance on the goal end-effector orientation [rad]*/

    KinematicsHelper kin_helper; /**< kinematic helper for FK and IK operations */
    ros::ServiceClient planning_client;/**< ros client for planning operations */


public:

    /**
     * @brief TrajectoryPlanner, a constructor
     * @param nh
     */
    TrajectoryPlanner(ros::NodeHandle nh);


    /**
     * @brief ~TrajectoryPlanner, a destructor
     */
    ~TrajectoryPlanner();

    /**
     * @brief setPlannerId
     * It sets the identity of the planner
     * @param planner_id
     */
    void setPlannerId(const string &planner_id);

    /**
     * @brief getPlannerId
     * It gets the identity of the planner
     * @return
     */
    string getPlannerId();

    /**
     * @brief setAllowedPlanningTime
     * It sets the maximum allowed time for planning
     * @param value
     */
    void setAllowedPlanningTime(double value);

    /**
     * @brief getAllowedPlanningTime
     * It gets the maximum allowed time for planning
     * @return
     */
    double getAllowedPlanningTime();

    /**
     * @brief setPlanningAttempts
     * It sets the maximum number of attempts for planning
     * @param value
     */
    void setPlanningAttempts(int value);

    /**
     * @brief getPlanningAttempts
     * It gets the maximum number of attempts for planning
     * @return
     */
    int getPlanningAttempts();

    /**
     * @brief setMaxTrajectoryPoints
     * It sets the maximum number of points in the trajectory
     * @param value
     */
    void setMaxTrajectoryPoints(int value);

    /**
     * @brief getMaxTrajectoryPoints
     * It gets the maximum number of points in the trajectory
     * @return
     */
    int getMaxTrajectoryPoints();


    /**
     * @brief setGoalJointTol
     * It sets the tolerance on the goal joint position [rad]
     * @param tol
     */
    void setGoalJointTol(double tol);

    /**
     * @brief getGoalJointTol
     * It gets the tolerance on the goal joint position [rad]
     * @return
     */
    double getGoalJointTol();

    /**
     * @brief setGoalPosTol
     * It sets the tolerance on the goal end-effector position [m]
     * @param tol
     */
    void setGoalPosTol(double tol);

    /**
     * @brief getGoalPosTol
     * It gets the tolerance on the goal end-effector position [m]
     * @return
     */
    double getGoalPosTol();

    /**
     * @brief setGoalOrTol
     * It sets the tolerance on the goal end-effector orientation [rad]
     * @param tol
     */
    void setGoalOrTol(double tol);

    /**
     * @brief getGoalOrTol
     * It gets the tolerance on the goal end-effector orientation [rad]
     * @return
     */
    double getGoalOrTol();

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

#endif // TRAJECTORY_PLANNER_HPP
