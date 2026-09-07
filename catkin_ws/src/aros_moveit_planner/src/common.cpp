#include "../include/aros_moveit_planner/common.hpp"

namespace moveit_planning {


double getJointPositionFromState(const string &joint, const moveit_msgs::RobotState &state)
{
    double value = 0.0;

    for(size_t i = 0; i < state.joint_state.name.size(); ++i) {

        if(joint == state.joint_state.name[i]) {
            value = state.joint_state.position[i];
            break;
        }

    }
    return value;
}

void getJointPositionsFromState(const vector<string> &joints, const moveit_msgs::RobotState &state, vector<double> &values)
{

    for (size_t i = 0; i < joints.size(); ++i) {
        double value = getJointPositionFromState(joints[i], state);
        values.push_back(value);
    }

}

void getArmJointNames(const string &arm, vector<string> &names)
{

    for(int i = 1; i <= 7; ++i) {
        stringstream ss;
        ss << arm << "_joint" << i;
        names.push_back(ss.str());
    }

}

void getArmValuesFromState(const string &arm, const moveit_msgs::RobotState &state, vector<double> &values) {

    vector<string> joints;
    getArmJointNames(arm,joints);
    getJointPositionsFromState(joints, state, values);

}

void printJointsValues(const moveit_msgs::RobotState &state, const string &arm)
{
    vector<string> joints;
    getArmJointNames(arm, joints);

    vector<double> values;
    getJointPositionsFromState(joints, state, values);

    for(size_t i = 0; i < joints.size(); ++i) {
        ROS_INFO("%s: %1.4f", joints[i].c_str(), values[i]);
    }
}

}
