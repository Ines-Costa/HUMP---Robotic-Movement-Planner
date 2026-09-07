#include <gtest/gtest.h>



#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "../../include/aros_moveit_planner/humanoid_moveit_planner.hpp"

int argcc;
char** argvv;

using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_msgs;
using namespace moveit_planning;

namespace aros_pick_place {

class PickPlace
{

public:

    boost::shared_ptr<PlanningSceneInterface> planning_scene_interface;
    boost::shared_ptr<HumanoidPlanner>  h_planner;

    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose goal_pose;

    ros::Publisher pub_attach_coll_obj;

    PickPlace(ros::NodeHandle &nh, const string &planner_id)
    {


        planning_scene_interface.reset(new PlanningSceneInterface());
        // Create PlanningHelper for one of the planning groups
        h_planner.reset(new HumanoidPlanner("title"));

        h_planner->setAllowedPlanningTime(5);
        h_planner->setPlanningAttempts(5);
        h_planner->setPlanningGroupName("right_arm");
        //h_planner->setSupportSurfaceName("table");
        h_planner->setPlannerId(planner_id);

        pub_attach_coll_obj = nh.advertise<AttachedCollisionObject>("attached_collision_object", 10);


        // Let everything load
        ros::Duration(5.0).sleep();

    }

    ~PickPlace() {}

    void openGripper(trajectory_msgs::JointTrajectory& posture)
    {

      posture.joint_names.resize(6);
      posture.joint_names[0] = "right_finger1_joint1";
      posture.joint_names[1] = "right_finger1_joint2";
      posture.joint_names[2] = "right_finger2_joint1";
      posture.joint_names[3] = "right_finger2_joint2";
      posture.joint_names[4] = "right_finger3_joint1";
      posture.joint_names[5] = "right_finger3_joint2";

      posture.points.resize(1);
      posture.points[0].positions.resize(6);
      posture.points[0].positions[0] = 0.0;
      posture.points[0].positions[1] = 0.0;
      posture.points[0].positions[2] = 0.0;
      posture.points[0].positions[3] = 0.0;
      posture.points[0].positions[4] = 0.0;
      posture.points[0].positions[5] = 0.0;
    }

    void closedGripper(trajectory_msgs::JointTrajectory& posture){

      posture.joint_names.resize(6);
      posture.joint_names[0] = "right_finger1_joint1";
      posture.joint_names[1] = "right_finger1_joint2";
      posture.joint_names[2] = "right_finger2_joint1";
      posture.joint_names[3] = "right_finger2_joint2";
      posture.joint_names[4] = "right_finger3_joint1";
      posture.joint_names[5] = "right_finger3_joint2";

      posture.points.resize(1);
      posture.points[0].positions.resize(6);
      posture.points[0].positions[0] = 1.22;
      posture.points[0].positions[1] = 0.41;
      posture.points[0].positions[2] = 1.22;
      posture.points[0].positions[3] = 0.41;
      posture.points[0].positions[4] = 1.22;
      posture.points[0].positions[5] = 0.41;
    }

    bool pick(const string &name)
    {
        ROS_WARN_STREAM_NAMED("", "picking object "<< name);

        // Pick grasp
        std::vector<Grasp> grasps;
        Grasp g;

      geometry_msgs::PoseStamped p;
      p.header.frame_id = "base_link";
      p.pose.position.x = -0.47;
      p.pose.position.y = 0.615;
      p.pose.position.z = 1.118;
      p.pose.orientation.x = -0.0655;
      p.pose.orientation.y = -0.7018;
      p.pose.orientation.z = -0.07045;
      p.pose.orientation.w = 0.70577;
      g.grasp_pose = p;

      //approach
      g.pre_grasp_approach.direction.header.frame_id = "right_hand_sensor_link";
      g.pre_grasp_approach.direction.vector.x = 0.0;
      g.pre_grasp_approach.direction.vector.y = 0.0;
      g.pre_grasp_approach.direction.vector.z = 1.0;
      g.pre_grasp_approach.min_distance = 0.1;
      g.pre_grasp_approach.desired_distance = 0.2;

      // retreat
      g.post_grasp_retreat.direction.header.frame_id = "base_link";
      g.post_grasp_retreat.direction.vector.x = 0.0;
      g.post_grasp_retreat.direction.vector.y = 0.0;
      g.post_grasp_retreat.direction.vector.z = 1.0;
      g.post_grasp_retreat.min_distance = 0.1;
      g.post_grasp_retreat.desired_distance = 0.2;

      openGripper(g.pre_grasp_posture);

      closedGripper(g.grasp_posture);

      grasps.push_back(g);

        PlanningResultPtr plan = h_planner->plan_pick(name,"table",{""},grasps);
        if(plan->status == HumanoidPlanner::SUCCESS) {
            ROS_INFO("Planning pickup phase sucessfully completed!");
        } else {
            ROS_WARN("Planning pickup phase failed!");
            return false;
        }

        cout << ("Should the planned trajectory be executed (y/n)? ");
        string input;
        //cin >> input;
        input = "y";
        if(input == "y") {
            ROS_INFO("Executing pickup...");
            return h_planner->execute(plan);
        } else {
            ROS_WARN("Execution stopped by user!");
            return false;
        }
    }

    bool place(std::string name)
    {
        ROS_WARN_STREAM_NAMED("pick_place", "Placing "<< name);

        std::vector<PlaceLocation> loc;

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "base_link";
        p.pose.position.x = -0.564;
        p.pose.position.y = 1.025;
        p.pose.position.z = 1.118;
        p.pose.orientation.x =0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z =0;
        p.pose.orientation.w = 1;

        moveit_msgs::PlaceLocation g;
        g.place_pose = p;


        // approach
        g.pre_place_approach.direction.header.frame_id = "base_link";
        g.pre_place_approach.direction.vector.x = 0.0;
        g.pre_place_approach.direction.vector.y = 0.0;
        g.pre_place_approach.direction.vector.z = -1.0;
        g.pre_place_approach.min_distance = 0.1;
        g.pre_place_approach.desired_distance = 0.2;

        // retreat
        g.post_place_retreat.direction.header.frame_id = "right_hand_sensor_link";
        g.post_place_retreat.direction.vector.x = 0.0;
        g.post_place_retreat.direction.vector.y = 0.0;
        g.post_place_retreat.direction.vector.z = -1.0;
        g.post_place_retreat.min_distance = 0.1;
        g.post_place_retreat.desired_distance = 0.2;

        openGripper(g.post_place_posture);

        loc.push_back(g);

        PlanningResultPtr plan = h_planner->plan_place(name,"table",{""},loc);

        if(plan->status == HumanoidPlanner::SUCCESS) {
            ROS_INFO("Planning placement phase sucessfully completed!");
        } else {
            ROS_WARN("Planning placement phase failed!");
            return false;
        }

        cout << ("Should the planned trajectory be executed (y/n)? ");
        string input;
        //cin >> input;
        input="y";
        if(input == "y") {
            ROS_INFO("Executing placement...");
            return h_planner->execute(plan);
        } else {
            ROS_WARN("Execution stopped by user!");
            return false;
        }

    }

    bool start()
    {
        // -----------------------------------------------------------
        // create obstacle and object to move...
        createEnvironment();

        bool found = false;
        while (!found && ros::ok()) {

            if (!pick("part")) {
                ROS_ERROR_STREAM_NAMED("pick_place", "Pick failed. Retrying.");
                //cleanupACO(OBJ_ID);
            } else {
                ROS_INFO_STREAM_NAMED("pick_place",	"Done with pick!");
                found = true;
            }
        }

        ROS_INFO_STREAM_NAMED("simple_pick_place", "Waiting to put...");
        ros::Duration(5.5).sleep();



        bool placed = false;
        while (!placed && ros::ok()) {
            if (!place("part")) {
                ROS_ERROR_STREAM_NAMED("pick_place", "Place failed.");
            } else {
                ROS_INFO_STREAM_NAMED("pick_place", "Done with place");
                placed = true;
            }
        }

        ROS_INFO_STREAM_NAMED("pick_place", "Pick and place cycle complete");

        ros::Duration(5).sleep();



        ROS_INFO_STREAM_NAMED("pick_place", "Going to park position");



       PlanningResultPtr plan = h_planner->plan_arm_hand_to_default("park");
       if(plan->status == HumanoidPlanner::SUCCESS) {
            ROS_INFO("Motion plan computed - executing trajectory...");
            h_planner->execute(plan);
        } else {
            ROS_WARN("Planning was not successful!");
        }

        ROS_INFO("Finished");



        return true;
    }
    void createEnvironment()
    {

      moveit_msgs::CollisionObject co;
      co.header.stamp = ros::Time::now();
      co.header.frame_id = "base_link";

      vector<string> object_ids;
      vector<moveit_msgs::CollisionObject> collision_objects;

      // remove pole
      co.id = "pole";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      object_ids.push_back(co.id);
      planning_scene_interface->removeCollisionObjects(object_ids);

      // add pole
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
      co.primitive_poses.resize(1);      
      co.primitive_poses[0].position.x = -0.64;
      co.primitive_poses[0].position.y = 0.82;
      co.primitive_poses[0].position.z = 1.43;
      co.primitive_poses[0].orientation.x = 0.0;
      co.primitive_poses[0].orientation.y = 0.0;
      co.primitive_poses[0].orientation.z = 0.0;
      co.primitive_poses[0].orientation.w = 1.0;
      collision_objects.push_back(co);
      ROS_INFO("Add the pole into the world");
      planning_scene_interface->addCollisionObjects(collision_objects);



      // remove table
      co.id = "table";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      object_ids.push_back(co.id);
      planning_scene_interface->removeCollisionObjects(object_ids);

      // add table
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.5;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = -0.56;
      co.primitive_poses[0].position.y = 0.06;
      co.primitive_poses[0].position.z = 0.77;
      co.primitive_poses[0].orientation.x = 0.0;
      co.primitive_poses[0].orientation.y = 0.0;
      co.primitive_poses[0].orientation.z = 0.0;
      co.primitive_poses[0].orientation.w = 1.0;
      collision_objects.push_back(co);
      ROS_INFO("Add the table into the world");
      planning_scene_interface->addCollisionObjects(collision_objects);


      // remove part
      co.id = "part";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      object_ids.push_back(co.id);
      planning_scene_interface->removeCollisionObjects(object_ids);

      // add part
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = -0.6;
      co.primitive_poses[0].position.y = 0.63;
      co.primitive_poses[0].position.z = 1.10;
      co.primitive_poses[0].orientation.x = 0.0;
      co.primitive_poses[0].orientation.y = 0.0;
      co.primitive_poses[0].orientation.z = 0.0;
      co.primitive_poses[0].orientation.w = 1.0;
      collision_objects.push_back(co);
      ROS_INFO("Add the part into the world");
      planning_scene_interface->addCollisionObjects(collision_objects);


    }

};

} //aros_pick_place

TEST(TestSuite_humanoid_plan, Test_hp_goal1)
{
    ros::init(argcc, argvv, "test_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    string planner = "RRTConnectkConfigDefault";
    if(argcc > 1) {
        planner = argvv[1];
    }

    aros_pick_place::PickPlace t(nh, planner);
    t.start();
    EXPECT_TRUE(true);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc,argv);
    argcc=argc;
    argvv=argv;
    return RUN_ALL_TESTS();
}

