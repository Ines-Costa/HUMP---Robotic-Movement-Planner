#ifndef HUMANOID_PLANNER_HPP
#define HUMANOID_PLANNER_HPP

#include <ros/ros.h>
#include <string>

#include <boost/scoped_ptr.hpp>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include "common.hpp"
#include "kinematics_helper.hpp"

using namespace std;
using namespace moveit_msgs;
using namespace sensor_msgs;

namespace moveit_planning {

typedef boost::shared_ptr<PlanningResult> PlanningResultPtr; /**< pointer to a PlanningResult struct */


//! The HumanoidPlanner class
/**
 * @brief The HumanoidPlanner class
 * This class
 */
class HumanoidPlanner{

private:

    string group_name_arm;/**< arm planning group */
    string group_name_hand;/**< hand planning group */
    string group_name_arm_hand;/**< arm hand planning group */
    string end_effector;/**< group of the end effector */
    string end_effector_link;/**< end effector that is linking to the planning group */
    double planning_time;/**< maximum allowed planning time */
    int planning_attempts;/**< number of attempts of planning */
    double goal_joint_tolerance;/**< tolerance on the goal joint position [rad]*/
    double goal_position_tolerance;/**< tolerance on the goal end-effector position [m] */
    double goal_orientation_tolerance;/**< tolerance on the goal end-effector orientation [rad]*/
    string planner_id;/**< identity of the planner */
    string planner_name; /**< the name of the planner */
    string scenario_path; /**< scenario of the path */

    ros::NodeHandle nh;/**< ros node handle */
    ros::Publisher pub;/**< ros publisher for pickup messages */
    ros::ServiceClient execution_client;/**< ros service client for execution of the trajectory */
    ros::Publisher attached_object_pub;/**< ros publisher of the attached collision object */
    ros::ServiceClient get_planning_scene_client;/**< ros service client for getting the planning scene */
    ros::ServiceClient apply_planning_scene_client; /**< ros service client to apply the planning scene */
    robot_model::RobotModelConstPtr robot_model;/**< model of the loaded robot*/
    const robot_model::JointModelGroup *joint_model_group;/**< joint group of the planning group */

    boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client; /**< pickup action client */
    boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client; /**< place action client */
    boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client;/**< move group action client */
    //boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > execute_action_client;/**< execute action client */

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     */
    void RPY_matrix(std::vector<double>rpy, Matrix3d &Rot);

    /**
     * @brief setJoints_barrettHand
     * @param finalHand
     * @param goal_posture_ext
     */
    void setJoints_barrettHand(std::vector<double> finalHand, std::vector<double>& goal_posture_ext);

    /**
     * @brief openBarrettHand
     * @param finalHand
     * @param posture
     */
    void openBarrettHand(std::vector<double> finalHand, trajectory_msgs::JointTrajectory& posture);

    /**
     * @brief closedBarrettHand
     * @param finalHand
     * @param posture
     */
    void closedBarrettHand(std::vector<double> finalHand, trajectory_msgs::JointTrajectory& posture);

    /**
     * @brief execute
     * It executes the robot trajectory
     * @param trajectory
     * @return
     */
    bool execute(const RobotTrajectory &trajectory);

    /**
     * @brief attachObject
     * This method attaches the object from the end effector link
     * @param object
     * @return
     */
    bool attachObject(const string &object);

    /**
     * @brief detachObject
     * This method detaches the object from the end effector link
     * @param object
     * @return
     */
    bool detachObject(const string &object);

public:

    static const int SUCCESS = 1; /**< success status code */
    static const int FAILURE = 0;/**< failure status code */
    static const int PICK = 0;/**< pick type code */
    static const int PLACE = 1;/**< place type code */
    static const int GOAL = 2;/**< goal type code */

    /**
     * @brief HumanoidPlanner, a constructor
     * @param name
     */
    HumanoidPlanner(const string &name);


    /**
     * @brief HumanoidPlanner, a constructor
     * @param name
     * @param path
     */
    HumanoidPlanner(const string &name,const string &path);

    /**
     * @brief HumanoidPlanner, a copy constructor
     * @param hp
     */
    HumanoidPlanner(const HumanoidPlanner& hp);

    /**
     * @brief ~HumanoidPlanner, a destructor
     */
    ~HumanoidPlanner();

    /**
     * @brief init
     * Initialization of the constructor
     */
    void init();


    /**
     * @brief pick
     * @param params
     * @return
     */
    PlanningResultPtr pick(moveit_params& params);

    /**
     * @brief plan_pick
     * @param object_id
     * @param support_surf
     * @param allowed_touch_objects
     * @param grasps
     * @return
     */
    PlanningResultPtr plan_pick(const string &object_id, const string &support_surf, const std::vector<string> &allowed_touch_objects,const vector<Grasp> &grasps);

    /**
     * @brief place
     * @param params
     * @return
     */
    PlanningResultPtr place(moveit_params& params);

    /**
     * @brief plan_place
     * @param object_id
     * @param support_surf
     * @param allowed_touch_objects
     * @param locations
     * @return
     */
    PlanningResultPtr plan_place(const string &object_id, const string &support_surf, const std::vector<string> &allowed_touch_objects, const vector<PlaceLocation> &locations);


    //PlanningResultPtr plan_move(const std::string group_arm, const std::string group_hand, const geometry_msgs::Pose &pose_goal, int hand_code, std::vector<double> hand_posture);


    /**
     * @brief move
     * @param params
     * @param goal_posture
     * @return
     */
    PlanningResultPtr move(moveit_params& params, const std::vector<double> goal_posture);

    /**
     * @brief move
     * @param params
     * @return
     */
    PlanningResultPtr move(moveit_params& params);

    /**
     * @brief plan_move
     * Plan the movement to reach the given posture
     * @param posture
     * @param group
     * @return
     */
    PlanningResultPtr plan_move(const std::string group, const std::vector<double> posture);

    /**
     * @brief plan_move
     * @param group
     * @param posture
     * @param traj_c
     * @return
     */
    PlanningResultPtr plan_move(const std::string group, const std::vector<double> posture, const moveit_msgs::TrajectoryConstraints traj_c);

    /**
     * @brief plan_arm_to_default
     * Plan the trajectory towards the default posture of the arm
     * @param def_posture
     * It must be "park" or "home"
     * @return
     */
    PlanningResultPtr plan_arm_to_default(const std::string def_posture);

    /**
     * @brief plan_hand_to_default
     * Plan the trajectory towards the default posture of the hand
     * @param def_posture
     * It must be "park" or "home"
     * @return
     */
    PlanningResultPtr plan_hand_to_default(const std::string def_posture);

    /**
     * @brief plan_arm_hand_to_default
     * Plan the trajectory towards the default posture of the arm_hand
     * @param def_posture
     * It must be "park" or "home"
     * @return
     */
    PlanningResultPtr plan_arm_hand_to_default(const std::string def_posture);

    /**
     * @brief execute
     * It executes the planned trajectory
     * @param plan
     * @return
     */
    bool execute(const PlanningResultPtr &plan);

    /**
     * @brief setPlanningGroupName
     * It sets the name of the planning group
     * @param name
     */
    void setPlanningGroupName(const string &name);

    /**
     * @brief getPlanningGroupName
     * It gets the name of the planning group
     * @return
     */
    string getPlanningGroupName();

    /**
     * @brief setPlannerId
     * It sets the identity of the planner
     * @param id
     */
    void setPlannerId(const string &id);

    /**
     * @brief getPlannerId
     * It gets the identity of the planner
     * @return
     */
    string getPlannerId();

    /**
     * @brief setAllowedPlanningTime
     * It sets the maximum allowed planning time
     * @param value
     */
    void setAllowedPlanningTime(double value);

    /**
     * @brief getAllowedPlanningTime
     * It gets the maximum allowed planning time
     * @return
     */
    double getAllowedPlanningTime();

    /**
     * @brief setPlanningAttempts
     * It sets the attempts of planning
     * @param value
     */
    void setPlanningAttempts(int value);

    /**
     * @brief getPlanningAttempts
     * It gets the attempts of planning
     * @return
     */
    int getPlanningAttempts();


};


} // namespace moveit_planning

#endif // HUMANOID_PLANNER_HPP
