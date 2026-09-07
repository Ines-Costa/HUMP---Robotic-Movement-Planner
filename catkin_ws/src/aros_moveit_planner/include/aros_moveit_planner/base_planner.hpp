#ifndef BASE_PLANNER_HPP
#define BASE_PLANNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MotionPlanResponse.h>

using namespace std;

namespace trajectory_planner_moveit {

//! The BasePlanner class
/**
 * @brief The BasePlanner class
 * This class is abstract and defines the main features of a planner
 */
class BasePlanner {

protected:

    string arm; /**< selected arm (e.g. "right") */

public:

    virtual ~BasePlanner() {}

    /**
     * @brief getName
     * Return the name of the planner
     * @return
     */
    //virtual const string getName() = 0;

    /**
     * @brief plan
     * This method plans the trajectory towards the goal
     * @param goal
     * @param solution
     * @return
     */
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution) = 0;

    /**
     * @brief plan
     * This method plans the trajectory towards the goal starting from the start:state
     * @param goal
     * @param solution
     * @param start_state
     * @return
     */
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state) = 0;

    /**
     * @brief setArm
     * This method sets the arm that is object of the planning
     * @param _arm
     */
    void setArm(const string &_arm) { arm = _arm; }

    /**
     * @brief getArm
     * This method gets the arm that is object of the planning
     * @return
     */
    string getArm() { return arm; }

};

}


#endif // BASE_PLANNER_HPP
