#ifndef MOVEIT_COMMON_HPP
#define MOVEIT_COMMON_HPP


#define ATTEMPTS 5
#define TIMEOUT 0.1
#define CAN_LOOK false
#define ALLOW_REPLAN false
#define FRAME_ID "base_link"
#define SUPPORT_SURFACE "Table"

/** definitions for printing output */
#define SEP ", "
#define METERS " [m]"
#define MILLIMETERS " [mm]"
#define DEG " [deg]"
#define RAD " [rad]"
#define COLUMN ":"
#define SPACE " "
#define XposSTR "Xpos = "
#define YposSTR "Ypos = "
#define ZposSTR "Zpos = "
#define RollSTR "Roll = "
#define PitchSTR "Pitch = "
#define YawSTR "Yaw = "
#define XsizeSTR "Xsize = "
#define YsizeSTR "Ysize = "
#define ZsizeSTR "Zsize = "

#include <sstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>



using namespace std;
using namespace moveit_msgs;
using namespace Eigen;

namespace moveit_planning {

const double AP = 20.0*static_cast<double>(M_PI)/180; /**< aperture of the fingers when approaching to pick */

/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    std::string config;/**< current configuration of the planner in the file ompl_planning.yaml*/
    std::vector<double> pre_grasp_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance form the target*/
    std::vector<double> post_grasp_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance form the target*/
    std::vector<double> pre_place_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance form the target*/
    std::vector<double> post_place_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance form the target*/
    std::string support_surface; /**< support surface for pick and place movements */
    std::vector<std::string> allowed_touch_objects; /**< list of allowed object to touch for pick and place movements */
    int arm_code; /**< the code of the arm: 0 = both arms, 1 = right arm, 2 = left arm */
    int hand_code;/**< the code of the hand: 0 = human hand, 1 = barrett hand */
    //int griptype; /**< the type of the grip */
    string mov_infoline; /**< description of the movement */
    double dHO;/**< distanche hand-target*/
    std::vector<double> finalHand;/**< final posture of the hand */
    std::vector<double> target;/**< target to reach: tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(6)=yaw,*/
    std::string obj_name; /**< object involved in the movement */
    double tol_stop; /**< this tolerance defines the error between the norm of the final posture and the norm the current posture.
                    It has to be set to stop the movement when the final posture is reached. A tipical value is 0.1  */
    bool use_move_plane; /**< true to constrain the end-effector to move on a plane in move movements, false otherwise*/
    std::vector<double> plane_params; /**< plane cartesian parameters in move movements: a*x+b*y+c*z+d=0. a=plane_params(0), b=plane_params(1), c=plane_params(2), d=plane_params(3) */
    std::vector<double> plane_point1; /**< point1 that belongs to the plane */
    std::vector<double> plane_point2; /**< point2 that belongs to the plane */
    std::vector<double> plane_point3; /**< point3 that belongs to the plane */
    bool approach;/**< true to use the approach options, false otherwise  */
    bool retreat;/**< true to use the retreat options, false otherwise */
} moveit_params;

/**
 * @brief The PlanningResult struct
 * This structure lists the result of planning
 */
typedef struct{
        int type;/**< type of the trajectory */
        int status;/**< status code of the planning */
        string status_msg;/**< status message of the planning */
        string object_id;/**< identity of the object involved on the movement */
        RobotState start_state;/**< start state of the robot*/
        vector<RobotTrajectory> trajectory_stages;/**< sequence of robot trajectories */
        vector<string> trajectory_descriptions;/**< description of the trajectories */
}PlanningResult;


/**
 * @brief getJointPositionFromState *
 * This method retrieves the position of the joint with name "joint" in the robot state
 * @param joint
 * @param state
 * @return
 */
double getJointPositionFromState(const string &joint, const moveit_msgs::RobotState &state);

/**
 * @brief getJointPositionsFromState
 * This method retrieves all the values of the joints in the robot state
 * @param joints
 * @param state
 * @param values
 */
void getJointPositionsFromState(const vector<string> &joints, const moveit_msgs::RobotState &state, vector<double>&values);

/**
 * @brief getArmJointNames
 * This method retrieves the names of the joints in the arm selected
 * @param arm
 * @param names
 */
void getArmJointNames(const string &arm, vector<string> &names);

/**
 * @brief getArmValuesFromState
 * This method retrieves the values of the joints in the arm selected in the robot state
 * @param arm
 * @param state
 * @param values
 */
void getArmValuesFromState(const string &arm, const moveit_msgs::RobotState &state, vector<double> &values);

/**
 * @brief printJointsValues
 * This method prints the values of the joints of the arm selected in the robot state
 * @param state
 * @param arm
 */
void printJointsValues(const moveit_msgs::RobotState &state, const string &arm);

} // namespace moveit_planning



#endif // MOVEIT_COMMON_HPP
