/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>

#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <time.h>
#include "../include/motion_manager/qnode.hpp"
#include <vrep_common/simRosLoadScene.h>
#include <vrep_common/simRosCloseScene.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>
#include <vrep_common/simRosGetFloatSignal.h>
#include <vrep_common/simRosGetIntegerSignal.h>
#include <vrep_common/simRosGetStringSignal.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <vrep_common/simRosEnableSubscriber.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosReadProximitySensor.h>
#include <vrep_common/simRosSetObjectParent.h>
#include <vrep_common/JointSetStateData.h>
#include<vrep_common/simRosSetObjectIntParameter.h>
#include<vrep_common/simRosSetJointForce.h>
#include<vrep_common/simRosSetJointPosition.h>

#include <geometric_shapes/solid_primitive_dims.h>

#include "../include/motion_manager/v_repConst.hpp"



namespace motion_manager {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
    nodeName = "motion_manager";
    TotalTime = 0.0;

    #if HAND == 0 || HAND == 1
      //******Humanoid Hand and Barrett Hand******//
      right_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
      left_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
      right_2hand_pos.assign(3,0.0f);
      right_2hand_vel.assign(3,0.0f);
      right_2hand_force.assign(3,0.0f);
      left_2hand_pos.assign(3,0.0f);
      left_2hand_vel.assign(3,0.0f);
      left_2hand_force.assign(3,0.0f);
    #endif

    #if HAND==1
      //******Barrett Hand******//
      firstPartLocked.assign(3,false);
      needFullOpening.assign(3,0);
      closed.assign(3,false);
    #else
      closed = false;
    #endif
    this->hand_closed = false;

    got_scene = false;
    obj_in_hand = false;

    this->simulationTimePaused=0.0;
    this->simulationTime=0.0;
    this->simulationRunning=false;
    this->simulationPaused=false;
    this->sim_robot = true;

    // logging
    init();
    logging::add_common_attributes();

}

QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown();
    }
    wait();
}

bool QNode::on_init()
{
    ros::init(init_argc,init_argv,"motion_manager");
  if ( ! ros::master::check() ) {
    return false;
  }
    ros::start();
    start();
  return true;
}

bool QNode::on_init_url(const std::string &master_url, const std::string &host_url)
{
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;

  ros::init(remappings,"motion_manager");

  if ( ! ros::master::check() ) {
    return false;
  }

  ros::start();
  start();
  return true;
}

void QNode::on_end()
{
}

bool QNode::getTypeHands(){

    ros::NodeHandle n;
    std::string Hand_Right;
    std::string Hand_Left;

    vrep_common::simRosGetStringSignal srvs;
    bool succ = true;

    //Start simulation
    this->startSim();
    sleep(1);

    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("Hand_Right");
    add_client.call(srvs);
    if(srvs.response.result == 1){
      Hand_Right = srvs.response.signalValue;
      auto it = hand_map.find(Hand_Right);
      if(it != hand_map.end())
          this->hand_code_right = it->second;
      else{
          succ = false;
          log(QNode::Error, string("Hand type doesn't exist. Please include it in the list or change the scenario."));
      }
    }else{
        succ = false;
        log(QNode::Error, string("Error while retrieving the robot right hand type. Please load the scenario again."));
    }

    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("Hand_Left");
    add_client.call(srvs);
    if(srvs.response.result == 1){
      Hand_Left = srvs.response.signalValue;
      auto it = hand_map.find(Hand_Left);
      if(it != hand_map.end())
          this->hand_code_left = it->second;
      else{
          succ = false;
          log(QNode::Error, string("Hand type doesn't exist. Please include it in the list or change the scenario."));
      }
    }else{
        succ = false;
        log(QNode::Error, string("Error while retrieving the robot left hand type. Please load the scenario again."));
    }

    this->stopSim();
    return succ;
}

bool  QNode::loadScenario(const std::string& path,int id)
{

    ros::NodeHandle n;

    // stop simulations
    add_client = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);

    // close the old scene
    add_client = n.serviceClient<vrep_common::simRosCloseScene>("/vrep/simRosCloseScene");
    vrep_common::simRosCloseScene srvc;
    add_client.call(srvc);

    // load the new scene
    add_client = n.serviceClient<vrep_common::simRosLoadScene>("/vrep/simRosLoadScene");
    vrep_common::simRosLoadScene srv;
    srv.request.fileName = path;
    add_client.call(srv);

    int res = srv.response.result;

    bool succ = getTypeHands();

    if (res == 1 && succ){

        subInfo = n.subscribe("/vrep/info",1, &QNode::infoCallback,this);

        //Simulated Robot Joints States
        subJoints_state = n.subscribe("/vrep/joints_state",1, &QNode::JointsCallback, this); // vrep joints state
        //Real Robot Joints States
        subJoints_state_real = n.subscribe("/RAMBO/joints_state",1, &QNode::JointsRealCallback, this); // ARoS real joints state

        subRightProxSensor = n.subscribe("/vrep/right_prox_sensor",1,&QNode::rightProxCallback,this);
        subLeftProxSensor = n.subscribe("/vrep/left_prox_sensor",1,&QNode::leftProxCallback,this);
        subRightHandPos = n.subscribe("/vrep/right_hand_pose",1,&QNode::rightHandPosCallback,this);
        subRightHandVel = n.subscribe("/vrep/right_hand_vel",1,&QNode::rightHandVelCallback,this);
        subLeftHandPos = n.subscribe("/vrep/left_hand_pose",1,&QNode::leftHandPosCallback,this);
        subLeftHandVel = n.subscribe("/vrep/left_hand_vel",1,&QNode::leftHandVelCallback,this);


        switch(id){
        case 0:
            // **** Scene with RAMBO and QbSoftHand **** //
            subRolosCozinha = n.subscribe("/vrep/Rolos_cozinha_pose",1,&QNode::RolosCozinhaCallback,this);
            subTargetsRolosCozinha = n.subscribe("/vrep/Rolos_cozinha_Targets_info",1,&QNode::targetsRolosCozinhaCallback,this);
            subPastilhasMaqLoica14 = n.subscribe("/vrep/Pastilhas_maq_loica14_pose",1,&QNode::PastilhasMaqLoicaCallback,this);
            subTargetsPastilhasMaqLoica14 = n.subscribe("/vrep/Pastilhas_maq_loica_14_Targets_info",1,&QNode::targetsPastilhasMaqLoica14Callback,this);
            subDetergenteMaqLoica10 = n.subscribe("/vrep/detergente_maq_loica10_pose",1,&QNode::DetergenteMaqLoicaCallback,this);
            subTargetsDetergenteMaqLoica10 = n.subscribe("/vrep/detergente_maq_loica_10_Targets_info",1,&QNode::targetsDetergenteMaqLoica10Callback,this);

            subShelf1 = n.subscribe("/vrep/Shelf1_pose", 1, &QNode::Shelf1Callback, this);
            subShelf2 = n.subscribe("/vrep/Shelf2_pose", 1, &QNode::Shelf2Callback, this);
            subShelf3 = n.subscribe("/vrep/Shelf3_pose", 1, &QNode::Shelf3Callback, this);
            subShelf4 = n.subscribe("/vrep/Shelf4_pose", 1, &QNode::Shelf4Callback, this);
            subTable = n.subscribe("/vrep/TableRight_pose", 1, &QNode::TableCallback, this);

            break;
        case 1:
            // **** Scene to test the representation of objects as spheres or ellipsoids **** //
            subCup = n.subscribe("/vrep/Cup_pose",1,&QNode::CupCallback, this);


            subShelf1 = n.subscribe("/vrep/Shelf1_pose", 1, &QNode::Shelf1Callback, this);
            subShelf2 = n.subscribe("/vrep/Shelf2_pose", 1, &QNode::Shelf2Callback, this);
            //subShelf3 = n.subscribe("/vrep/Shelf3_pose", 1, &QNode::Shelf3Callback, this);
            //subShelf4 = n.subscribe("/vrep/Shelf4_pose", 1, &QNode::Shelf4Callback, this);

            subTable = n.subscribe("/vrep/Table_pose", 1, &QNode::TableCallback, this);

            //Environment2
            subCup1_4 = n.subscribe("/vrep/Cup1_4_pose",1,&QNode::Cup1_4_Callback,this);
            subCup1_8 = n.subscribe("/vrep/Cup1_8_pose",1,&QNode::Cup1_8_Callback,this);
            subCup1_12 = n.subscribe("/vrep/Cup1_12_pose",1,&QNode::Cup1_12_Callback,this);
            subCup1_16 = n.subscribe("/vrep/Cup1_16_pose",1,&QNode::Cup1_16_Callback,this);
            subCup1_19 = n.subscribe("/vrep/Cup1_19_pose",1,&QNode::Cup1_19_Callback,this);
            subCup2_4 = n.subscribe("/vrep/Cup2_4_pose",1,&QNode::Cup2_4_Callback,this);
            subCup2_8 = n.subscribe("/vrep/Cup2_8_pose",1,&QNode::Cup2_8_Callback,this);
            subCup2_12 = n.subscribe("/vrep/Cup2_12_pose",1,&QNode::Cup2_12_Callback,this);
            subCup2_16 = n.subscribe("/vrep/Cup2_16_pose",1,&QNode::Cup2_16_Callback,this);
            subCup2_19 = n.subscribe("/vrep/Cup2_19_pose",1,&QNode::Cup2_19_Callback,this);
            /*subCup3_4 = n.subscribe("/vrep/Cup3_4_pose",1,&QNode::Cup3_4_Callback,this);
            subCup3_8 = n.subscribe("/vrep/Cup3_8_pose",1,&QNode::Cup3_8_Callback,this);
            subCup3_12 = n.subscribe("/vrep/Cup3_12_pose",1,&QNode::Cup3_12_Callback,this);
            subCup3_16 = n.subscribe("/vrep/Cup3_16_pose",1,&QNode::Cup3_16_Callback,this);
            subCup3_19 = n.subscribe("/vrep/Cup3_19_pose",1,&QNode::Cup3_19_Callback,this);*/

            //Environment3
            subCup1 = n.subscribe("/vrep/Cup1_pose",1,&QNode::Cup1Callback, this);
            subCup2 = n.subscribe("/vrep/Cup2_pose",1,&QNode::Cup2Callback, this);
            subCup3 = n.subscribe("/vrep/Cup3_pose",1,&QNode::Cup3Callback, this);

            //Environment4
            subPerson = n.subscribe("/vrep/Person_pose",1,&QNode::PersonCallback,this);

            break;
        }
#if MOVEIT==1
        // planning scene of RViZ
        planning_scene_interface_ptr.reset(new moveit::planning_interface::PlanningSceneInterface());
#endif
        return true;
    }else{
        return false;
    }
}

void QNode::resetSimTime()
{

    this->TotalTime=0.0;
    this->simulationTime = 0.0;
}

void QNode::resetGlobals()
{
    #if HAND == 0 || HAND == 1
    // **** Human and Barrett Hand **** //
        for (int i =0; i < 3; ++i){
            closed.at(i)=false;
            needFullOpening.at(i)=0;
            firstPartLocked.at(i)=false;
        }
    #else
        closed = false;
    #endif

    obj_in_hand = false;
}

#if HAND != 0 || HAND != 1
    void QNode::init_HandPoses(qbsofthand& Robot_hand){
        Robot_hand.FullHandPoses = std::vector<HandPoses>(20);

        Robot_hand.FullHandPoses[0].dFF         = 100.0;
        Robot_hand.FullHandPoses[0].synergy     = 0.229360;
        Robot_hand.FullHandPoses[0].hand_angles = {45.0, 18.0, 11.0, 0.0, 5.0, 5.0, 10.0, 0.0, 3.0, 5.0, 3.0, 0.0, 5.0, 5.0, 7.0, -2.0, 12.0, 3.0, 22.0};

        Robot_hand.FullHandPoses[1].dFF         = 95.0;
        Robot_hand.FullHandPoses[1].synergy     = 0.246353;
        Robot_hand.FullHandPoses[1].hand_angles = {50.0, 18.0, 11.0, 0.0, 5.0, 6.0, 12.0, 0.0, 4.0, 5.0, 3.0, 0.0, 6.0, 6.0, 7.0, -2.0, 12.0, 5.0, 24.0};

        Robot_hand.FullHandPoses[2].dFF         = 90.0;
        Robot_hand.FullHandPoses[2].synergy     = 0.263537;
        Robot_hand.FullHandPoses[2].hand_angles = {55.0, 18.0, 11.0, 0.0, 5.0, 6.0, 15.0, 0.0, 4.0, 5.0, 3.0, 0.0, 6.0, 7.0, 6.0, -3.0, 12.0, 7.0, 27.0};

        Robot_hand.FullHandPoses[3].dFF         = 85.0;
        Robot_hand.FullHandPoses[3].synergy     = 0.280912;
        Robot_hand.FullHandPoses[3].hand_angles = {60.0, 18.0, 11.0, 0.0, 5.0, 7.0, 18.0, 0.0, 5.0, 5.0, 3.0, 0.0, 7.0, 8.0, 6.0, -3.0, 12.0, 9.0, 29.0};

        Robot_hand.FullHandPoses[4].dFF         = 80.0;
        Robot_hand.FullHandPoses[4].synergy     = 0.298480;
        Robot_hand.FullHandPoses[4].hand_angles = {66.0, 18.0, 11.0, 0.0, 7.0, 7.0, 18.0, 0.0, 5.0, 5.0, 4.0, 0.0, 7.0, 9.0, 7.0, -4.0, 13.0, 9.0, 30.0};

        Robot_hand.FullHandPoses[5].dFF         = 75.0;
        Robot_hand.FullHandPoses[5].synergy     = 0.316245;
        Robot_hand.FullHandPoses[5].hand_angles = {73.0, 18.0, 11.0, 0.0, 8.0, 7.0, 19.0, 0.0, 5.0, 5.0, 4.0, 0.0, 7.0, 9.0, 9.0, -4.0, 13.0, 9.0, 32.0};

        Robot_hand.FullHandPoses[6].dFF         = 70.0;
        Robot_hand.FullHandPoses[6].synergy     = 0.334209;
        Robot_hand.FullHandPoses[6].hand_angles = {78.0, 18.0, 11.0, 0.0, 10.0, 7.0, 19.0, 0.0, 5.0, 5.0, 5.0, 0.0, 7.0, 10.0, 10.0, -5.0, 14.0, 9.0, 33.0};

        Robot_hand.FullHandPoses[7].dFF         = 65.0;
        Robot_hand.FullHandPoses[7].synergy     = 0.352372;
        Robot_hand.FullHandPoses[7].hand_angles = {81.0, 18.0, 11.0, 0.0, 12.0, 7.0, 18.0, 0.0, 8.0, 5.0, 5.0, 0.0, 11.0, 11.0, 8.0, -5.0, 17.0, 9.0, 32.0};

        Robot_hand.FullHandPoses[8].dFF         = 60.0;
        Robot_hand.FullHandPoses[8].synergy     = 0.370739;
        Robot_hand.FullHandPoses[8].hand_angles = {85.0, 18.0, 11.0, 0.0, 13.0, 8.0, 17.0, 0.0, 10.0, 6.0, 5.0, 0.0, 14.0, 12.0, 7.0, -6.0, 19.0, 10.0, 31.0};

        Robot_hand.FullHandPoses[9].dFF         = 55.0;
        Robot_hand.FullHandPoses[9].synergy     = 0.389310;
        Robot_hand.FullHandPoses[9].hand_angles = {88.0, 18.0, 11.0, 0.0, 15.0, 8.0, 17.0, 0.0, 11.0, 7.0, 5.0, 0.0, 16.0, 12.0, 6.0, -6.0, 21.0, 10.0, 31.0};

        Robot_hand.FullHandPoses[10].dFF         = 50.0;
        Robot_hand.FullHandPoses[10].synergy     = 0.408089;
        Robot_hand.FullHandPoses[10].hand_angles = {90.0, 18.0, 11.0, 0.0, 16.0, 8.0, 17.0, 0.0, 11.0, 6.0, 5.0, 0.0, 16.0, 12.0, 6.0, -6.0, 20.0, 10.0, 30.0};

        Robot_hand.FullHandPoses[11].dFF         = 45.0;
        Robot_hand.FullHandPoses[11].synergy     = 0.427077;
        Robot_hand.FullHandPoses[11].hand_angles = {90.0, 19.0, 24.0, 0.0, 17.0, 5.0, 17.0, 0.0, 12.0, 9.0, 5.0, 0.0, 16.0, 14.0, 7.0, -6.0, 28.0, 0.0, 36.0};

        Robot_hand.FullHandPoses[12].dFF         = 40.0;
        Robot_hand.FullHandPoses[12].synergy     = 0.446278;
        Robot_hand.FullHandPoses[12].hand_angles = {90.0, 20.0, 26.0, 1.0, 18.0, 5.0, 17.0, 0.0, 14.0, 9.0, 4.0, 0.0, 16.0, 15.0, 9.0, -5.0, 27.0, 1.0, 38.0};

        Robot_hand.FullHandPoses[13].dFF         = 35.0;
        Robot_hand.FullHandPoses[13].synergy     = 0.465692;
        Robot_hand.FullHandPoses[13].hand_angles = {90.0, 20.0, 26.0, 2.0, 19.0, 6.0, 23.0, 0.0, 17.0, 10.0, 3.0, 0.0, 15.0, 17.0, 12.0, -3.0, 26.0, 3.0, 42.0};

        Robot_hand.FullHandPoses[14].dFF         = 30.0;
        Robot_hand.FullHandPoses[14].synergy     = 0.485324;
        Robot_hand.FullHandPoses[14].hand_angles = {90.0, 20.0, 26.0, 2.0, 19.0, 8.0, 24.0, 0.0, 17.0, 10.0, 3.0, 0.0, 16.0, 17.0, 11.0, -3.0, 26.0, 6.0, 41.0};

        Robot_hand.FullHandPoses[15].dFF         = 25.0;
        Robot_hand.FullHandPoses[15].synergy     = 0.505174;
        Robot_hand.FullHandPoses[15].hand_angles = {90.0, 20.0, 26.0, 3.0, 20.0, 7.0, 28.0, 0.0, 18.0, 11.0, 4.0, 0.0, 17.0, 19.0, 9.0, -3.0, 24.0, 14.0, 38.0};

        Robot_hand.FullHandPoses[16].dFF         = 20.0;
        Robot_hand.FullHandPoses[16].synergy     = 0.525246;
        Robot_hand.FullHandPoses[16].hand_angles = {90.0, 20.0, 26.0, 3.0, 21.0, 8.0, 33.0, 0.0, 19.0, 11.0, 5.0, 0.0, 19.0, 20.0, 7.0, -3.0, 23.0, 23.0, 35.0};

        Robot_hand.FullHandPoses[17].dFF         = 15.0;
        Robot_hand.FullHandPoses[17].synergy     = 0.545542;
        Robot_hand.FullHandPoses[17].hand_angles = {90.0, 20.0, 26.0, 3.0, 24.0, 7.0, 33.0, 0.0, 20.0, 12.0, 5.0, -1.0, 21.0, 21.0, 8.0, -4.0, 23.0, 23.0, 36.0};

        Robot_hand.FullHandPoses[18].dFF         = 10.0;
        Robot_hand.FullHandPoses[18].synergy     = 0.566065;
        Robot_hand.FullHandPoses[18].hand_angles = {90.0, 20.0, 26.0, 3.0, 28.0, 7.0, 33.0, 0.0, 21.0, 12.0, 5.0, -1.0, 23.0, 21.0, 10.0, -5.0, 23.0, 24.0, 37.0};

        Robot_hand.FullHandPoses[19].dFF         = 5.0;
        Robot_hand.FullHandPoses[19].synergy     = 0.586817;
        Robot_hand.FullHandPoses[19].hand_angles = {90.0, 20.0, 26.0, 3.0, 30.0, 6.0, 33.0, 0.0, 21.0, 13.0, 5.0, -2.0, 24.0, 22.0, 11.0, -5.0, 23.0, 24.0, 37.0};

        Robot_hand.PrecHandPoses = std::vector<HandPoses>(15);

        Robot_hand.PrecHandPoses[0].dFF         = 100;
        Robot_hand.PrecHandPoses[0].synergy     = 0;
        Robot_hand.PrecHandPoses[0].hand_angles = {60.0, 19.0, 12.0, 0.0, 5.0, 5.0, 2.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[1].dFF         = 95;
        Robot_hand.PrecHandPoses[1].synergy     = 0.04842105;
        Robot_hand.PrecHandPoses[1].hand_angles = {70.0, 19.0, 12.0, 0.0, 5.0, 4.0, 6.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[2].dFF         = 90;
        Robot_hand.PrecHandPoses[2].synergy     = 0.07153;
        Robot_hand.PrecHandPoses[2].hand_angles = {75.0, 19.0, 12.0, 0.0, 6.0, 5.0, 7.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[3].dFF         = 85;
        Robot_hand.PrecHandPoses[3].synergy     = 0.09831579;
        Robot_hand.PrecHandPoses[3].hand_angles = {80.0, 19.0, 12.0, 0.0, 6.0, 6.0, 9.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[4].dFF         = 80;
        Robot_hand.PrecHandPoses[4].synergy     = 0.14831579;
        Robot_hand.PrecHandPoses[4].hand_angles = {82.0, 19.0, 12.0, 0.0, 9.0, 7.0, 11.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[5].dFF         = 75;
        Robot_hand.PrecHandPoses[5].synergy     = 0.15205;
        Robot_hand.PrecHandPoses[5].hand_angles = {83.0, 19.0, 12.0, 0.0, 9.0, 7.0, 12.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[6].dFF         = 70;
        Robot_hand.PrecHandPoses[6].synergy     = 0.15728;
        Robot_hand.PrecHandPoses[6].hand_angles = {83.0, 19.0, 12.0, 0.0, 10.0, 7.0, 12.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[7].dFF         = 65;
        Robot_hand.PrecHandPoses[7].synergy     = 0.1625;
        Robot_hand.PrecHandPoses[7].hand_angles = {84.0, 19.0, 12.0, 0.0, 1.0, 8.0, 13.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[8].dFF         = 60;
        Robot_hand.PrecHandPoses[8].synergy     = 0.16826316;
        Robot_hand.PrecHandPoses[8].hand_angles = {85.0, 19.0, 12.0, 0.0, 13.0, 8.0, 14.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[9].dFF         = 55;
        Robot_hand.PrecHandPoses[9].synergy     = 0.18198;
        Robot_hand.PrecHandPoses[9].hand_angles = {87.0, 19.0, 14.0, 0.0, 14.0, 8.0, 14.0, 0.0, 4.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[10].dFF         = 50;
        Robot_hand.PrecHandPoses[10].synergy     = 0.19768421;
        Robot_hand.PrecHandPoses[10].hand_angles = {90.0, 20.0, 17.0, 0.0, 15.0, 7.0, 14.0, 0.0, 6.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[11].dFF         = 40;
        Robot_hand.PrecHandPoses[11].synergy     = 0.22752632;
        Robot_hand.PrecHandPoses[11].hand_angles = {90.0, 22.0, 19.0, 0.0, 17.0, 8.0, 18.0, 0.0, 9.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[12].dFF         = 30;
        Robot_hand.PrecHandPoses[12].synergy     = 0.24742105;
        Robot_hand.PrecHandPoses[12].hand_angles = {90.0, 22.0, 20.0, 0.0, 19.0, 8.0, 25.0, 0.0, 9.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[13].dFF         = 15;
        Robot_hand.PrecHandPoses[13].synergy     = 0.26736842;
        Robot_hand.PrecHandPoses[13].hand_angles = {90.0, 22.0, 23.0, 0.0, 24.0, 8.0, 32.0, 0.0, 11.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        Robot_hand.PrecHandPoses[14].dFF         = 5;
        Robot_hand.PrecHandPoses[14].synergy     = 0.29731579;
        Robot_hand.PrecHandPoses[14].hand_angles = {90.0, 22.0, 23.0, 0.0, 29.0, 8.0, 28.0, 0.0, 14.0, 2.0, 6.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0};

        for (auto& poses : Robot_hand.FullHandPoses) {
                std::transform(poses.hand_angles.begin(), poses.hand_angles.end(), poses.hand_angles.begin(),
                            [](double angle_deg) { return angle_deg * M_PI / 180.0; });
            }

        for (auto& poses : Robot_hand.PrecHandPoses) {
                std::transform(poses.hand_angles.begin(), poses.hand_angles.end(), poses.hand_angles.begin(),
                            [](double angle_deg) { return angle_deg * M_PI / 180.0; });
            }
    }
#endif

int QNode::getHandCodeRight(){
    return this->hand_code_right;
}

int QNode::getHandCodeLeft(){
    return this->hand_code_left;
}

void QNode::updateTargetsPosOr(string obj_name){
    if(obj_name == "Rolos_cozinha"){
        pos position;
        orient orientation;

        //Right Target
        targetPtr target = curr_scene->getTarget(obj_name + "_Right");

        position.Xpos = static_cast<double>(received_targets_Rolos_Cozinha_data[0]*1000);  //[mm]
        position.Ypos = static_cast<double>(received_targets_Rolos_Cozinha_data[1]*1000); //[mm]
        position.Zpos = static_cast<double>(received_targets_Rolos_Cozinha_data[2]*1000); //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Rolos_Cozinha_data[3]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Rolos_Cozinha_data[4]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Rolos_Cozinha_data[5]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //Left Target
        target = curr_scene->getTarget(obj_name + "_Left");

        position.Xpos = static_cast<double>(received_targets_Rolos_Cozinha_data[6]*1000); //[mm]
        position.Ypos = static_cast<double>(received_targets_Rolos_Cozinha_data[7]*1000); //[mm]
        position.Zpos = static_cast<double>(received_targets_Rolos_Cozinha_data[8]*1000); //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Rolos_Cozinha_data[9]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Rolos_Cozinha_data[10]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Rolos_Cozinha_data[11]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //UpRight Target
        target = curr_scene->getTarget(obj_name + "_UpRight");

        position.Xpos = static_cast<double>(received_targets_Rolos_Cozinha_data[12]*1000); //[mm]
        position.Ypos = static_cast<double>(received_targets_Rolos_Cozinha_data[13]*1000); //[mm]
        position.Zpos = static_cast<double>(received_targets_Rolos_Cozinha_data[14]*1000); //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Rolos_Cozinha_data[15]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Rolos_Cozinha_data[16]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Rolos_Cozinha_data[17]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);
    }else if (obj_name == "Pastilhas_maq_loica14") {
        pos position;
        orient orientation;

        //Right Target
        targetPtr target = curr_scene->getTarget(obj_name + "_Right");

        position.Xpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[0]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[1]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[2]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[3]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[4]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[5]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //Left Target
        target = curr_scene->getTarget(obj_name + "_Left");

        position.Xpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[6]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[7]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[8]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[9]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[10]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[11]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //UpRight Target
        target = curr_scene->getTarget(obj_name + "_UpRight");

        position.Xpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[12]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[13]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[14]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[15]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[16]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Pastilhas_Maq_Loica_14_data[17]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);
    }else if (obj_name == "detergente_maq_loica10") {
        pos position;
        orient orientation;

        //Right Target
        targetPtr target = curr_scene->getTarget(obj_name + "_Right");

        position.Xpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[0]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[1]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[2]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[3]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[4]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[5]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //Left Target
        target = curr_scene->getTarget(obj_name + "_Left");

        position.Xpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[6]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[7]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[8]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[9]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[10]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[11]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //UpRight Target
        target = curr_scene->getTarget(obj_name + "_UpRight");

        position.Xpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[12]) * 1000; //[mm]
        position.Ypos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[13]) * 1000; //[mm]
        position.Zpos = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[14]) * 1000; //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[15]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[16]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Detergente_Maq_Loica_10_data[17]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);
    }else if(obj_name == "Cup"){
        pos position;
        orient orientation;

        //Right Target
        targetPtr target = curr_scene->getTarget(obj_name + "_Right");

        position.Xpos = static_cast<double>(received_targets_Cup_data[0]*1000);  //[mm]
        position.Ypos = static_cast<double>(received_targets_Cup_data[1]*1000); //[mm]
        position.Zpos = static_cast<double>(received_targets_Cup_data[2]*1000); //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Cup_data[3]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Cup_data[4]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Cup_data[5]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);

        //Left Target
        target = curr_scene->getTarget(obj_name + "_Left");

        position.Xpos = static_cast<double>(received_targets_Cup_data[6]*1000); //[mm]
        position.Ypos = static_cast<double>(received_targets_Cup_data[7]*1000); //[mm]
        position.Zpos = static_cast<double>(received_targets_Cup_data[8]*1000); //[mm]
        target->setPos(position);

        orientation.roll = static_cast<double>(received_targets_Cup_data[9]) * static_cast<double>(M_PI/180);
        orientation.pitch = static_cast<double>(received_targets_Cup_data[10]) * static_cast<double>(M_PI/180);
        orientation.yaw = static_cast<double>(received_targets_Cup_data[11]) * static_cast<double>(M_PI/180);
        target->setOr(orientation);
    }
}

scenarioPtr QNode::getscene(){
    return this->curr_scene;
}

bool QNode::getElements(scenarioPtr scene)
{

    ros::NodeHandle n;

    // start the simulation
    this->startSim();
    sleep(1);

    int n_objs; // total number of objects in the scenario
    int n_poses; // total number of poses in the scenario
    int cnt_obj = 0; // index of the object being loaded
    int cnt_pose = 0; // index of the pose being loaded
    std::string signPrefix = ""; // prefix of each object
    std::string infoLine; // info line in the list of elements
    std::string signTarRight = "_targetRight";
    std::string signTarLeft = "_targetLeft";
    std::string signEngage ="_engage";
    bool succ = true;

    // **** Object Info **** //
    pos obj_pos;// position of the object
    orient obj_or;// orientation of the object
    dim obj_size;// size of the object
    std::string obj_info_str;
    std::vector<double> obj_info_vec;
    std::vector<std::string> objs_prefix;

    // **** Target Info **** //
    pos tarRight_pos;// target right position
    orient tarRight_or;// target right orientation
    pos tarRight1_pos; // target right 1 position
    orient tarRight1_or; // target right 1 orientation
    pos tarLeft_pos;// target left position
    orient tarLeft_or;// target left orientation

    // **** Engage Info **** //
    pos engage_pos; // engage point position
    orient engage_or;// engage point orientation

    // **** Humanoid/ Robot Info **** //
    pos humanoid_pos; // position of the humanoid
    orient humanoid_or; // orientation of the humanoid
    dim humanoid_size; // size of the humanoid
#if HAND == 0 || HAND == 1
    arm humanoid_arm_specs; // specs of the arms
#else
    arm humanoid_arm_right_specs; //specs of the right arm
    arm humanoid_arm_left_specs; //specs of the right arm
#endif

    std::string Hname;

#if HAND != 0 && HAND != 1
    std::string Hand_Right;
    std::string Hand_Left;
#endif

    // **** Pose Info **** //
    std::string pose_info_str;
    std::vector<double> pose_info_vec;
    pos pose_pos;// position of the pose
    orient pose_or;// orientation of the pose
    std::vector<std::string> poses_prefix; // names of the poses
    std::vector<bool> poses_rel; // relations of the poses
    std::vector<int> poses_obj_id; // id of the related object

    vrep_common::simRosGetIntegerSignal srvi;
    vrep_common::simRosGetFloatSignal srvf;
    vrep_common::simRosGetStringSignal srvs;
    vrep_common::simRosGetObjectHandle srv_get_handle;
    ros::ServiceClient client_getHandle;

    // **** DH Parameters **** //
    int floatCount;
#if HAND == 0 || HAND == 1
    std::vector<double> DH_params_vec;
    string DH_params_str;
#else
    string DH_params_right_str, DH_params_left_str;
    std::vector<double> DH_params_right_vec, DH_params_left_vec;
#endif

    // **** Transformation Matrices **** //
    // arm: matrices from world to right and left references
    std::string mat_arms_str;
    std::string mat_right_arm_str;
    std::string mat_left_arm_str;
    std::vector<double> mat_arms_vec;
    std::vector<double> mat_right_arm_vec;
    std::vector<double> mat_left_arm_vec;
    Matrix4d mat_right;
    Matrix4d mat_left;
    // hand: matrices from the last joint of the arm to the palm of the hand
    std::string r_mat_hand_str;
    std::string l_mat_hand_str;
    std::vector<double> r_mat_hand_vec;
    std::vector<double> l_mat_hand_vec;
    Matrix4d mat_r_hand;
    Matrix4d mat_l_hand;

    #if HAND==0
        // **** Human Hand Parameters **** //
        double max_human_Ap; //[m]
        human_hand jarde_hand;
        jarde_hand.fingers = std::vector<human_finger>(4);
        human_finger fing1 = jarde_hand.fingers.at(0);
        human_finger fing2 = jarde_hand.fingers.at(1);
        human_finger fing3 = jarde_hand.fingers.at(2);
        human_finger fing4 = jarde_hand.fingers.at(3);
        human_thumb thumb = jarde_hand.thumb;
    #elif HAND==1
        // **** Barrett Hand Parameters **** //
        barrett_hand humanoid_hand_specs; // specs of the barret hand
        double maxAp; //[m]
        double Aw; // [m]
        double A1; // [m]
        double A2; // [m]
        double A3; // [m]
        double D3; // [m]
        double phi2; // [rad]
        double phi3; // [rad]
    #else
        double maxAp_right, maxAp_left; // [mm]

        // **** Electric Gripper Parameters **** //
        electric_gripper Humanoid_gripper_specs; //specs of the electric gripper
        double minAp_left, minAp_right; // [mm]
        double A1_left, A1_right; //[mm]
        double D3_left, D3_right; // [mm]

        // **** QbSoftHand **** //
        qbsofthand Robot_hand_right;
        Robot_hand_right.fingers = std::vector<qbsofthand_finger>(5);
        qbsofthand_finger Thumb_right = Robot_hand_right.fingers.at(0);
        qbsofthand_finger Index_right = Robot_hand_right.fingers.at(1);
        qbsofthand_finger Middle_right = Robot_hand_right.fingers.at(2);
        qbsofthand_finger Ring_right = Robot_hand_right.fingers.at(3);
        qbsofthand_finger Little_right = Robot_hand_right.fingers.at(4);
        init_HandPoses(Robot_hand_right);

        qbsofthand Robot_hand_left;
        Robot_hand_left.fingers = std::vector<qbsofthand_finger>(5);
        qbsofthand_finger Thumb_left = Robot_hand_left.fingers.at(0);
        qbsofthand_finger Index_left = Robot_hand_left.fingers.at(1);
        qbsofthand_finger Middle_left = Robot_hand_left.fingers.at(2);
        qbsofthand_finger Ring_left = Robot_hand_left.fingers.at(3);
        qbsofthand_finger Little_left = Robot_hand_left.fingers.at(4);
        init_HandPoses(Robot_hand_left);
    #endif

    // **** Torso Parameters **** //
    humanoid_part torso;  // parameters of the torso (ARoS and Jarde)
    std::string torso_str;
    std::vector<double> torso_vec;

    // **** Home Postures **** //
#if HAND != 1
    std::vector<double> rposture = std::vector<double>(JOINTS_ARM); // right
    std::vector<double> lposture = std::vector<double>(JOINTS_ARM); // left

    // **** Joint Limits **** //
    std::vector<double> min_rlimits = std::vector<double>(JOINTS_ARM); // minimum right limits
    std::vector<double> max_rlimits = std::vector<double>(JOINTS_ARM); // maximum right limits
    std::vector<double> min_llimits = std::vector<double>(JOINTS_ARM); // minimum left limits
    std::vector<double> max_llimits = std::vector<double>(JOINTS_ARM); // maximum left limits
#else
    std::vector<double> rposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // right
    std::vector<double> lposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // left

    // **** Joint Limits **** //
    std::vector<double> min_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum right limits
    std::vector<double> max_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum right limits
    std::vector<double> min_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum left limits
    std::vector<double> max_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum left limits
#endif

    // **** Others **** //
    int rows;

    const string NOBJECTS = string("n_objects");
    const string NPOSES = string("n_poses");

    int scenarioID  = scene->getID();
    if (scenarioID == 0){
        // error, no scenario
        throw string("No scenario");
    }else if(scenarioID == 1){
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        // **** Load Number of Objects **** //
        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if(srvi.response.result == 1){
          n_objs = srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // **** Load Number of Poses **** //
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if(srvi.response.result == 1){
          n_poses = srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        //get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");

        // **** Objects **** //
        //obj_id = 0 (Rolos_cozinha)
        objs_prefix.push_back("Rolos_cozinha");

        //obj_id = 1 (Pastilhas_maq_loica14)
        objs_prefix.push_back("Pastilhas_maq_loica14");

        //obj_id = 2 (detergente_maq_loica)
        objs_prefix.push_back("detergente_maq_loica10");

        //obj_id = 4 (Shelf1)
        objs_prefix.push_back("Shelf1");

        //obj_id = 5 (Shelf2)
        objs_prefix.push_back("Shelf2");

        //obj_id = 6 (Shelf3)
        objs_prefix.push_back("Shelf3");

        //obj_id = 7 (Shelf4)
        objs_prefix.push_back("Shelf4");

        //obj_id = 8 (Table)
        objs_prefix.push_back("TableRight");
        //*******************//

        std::vector<float>targetValues;
        while(cnt_obj < n_objs){
            targetValues.clear();
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if(succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for(int k = 0; k < floatCount; ++k){
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));
                }

                //position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                //orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180;   //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180;  //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;    //[rad]
                //size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]

                //position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000; targetValues.push_back(obj_info_vec.at(9));//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000; targetValues.push_back(obj_info_vec.at(10));//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000; targetValues.push_back(obj_info_vec.at(11));//[mm]
                //orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(12)); //[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(13)); //[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(14));   //[rad]

                //position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000; targetValues.push_back(obj_info_vec.at(15));//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000; targetValues.push_back(obj_info_vec.at(16));//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000; targetValues.push_back(obj_info_vec.at(17));//[mm]
                //orientation of the target right
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(18)); //[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(19));//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(20));//[rad]

                //position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000; //[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000; //[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000; //[mm]
                //orientation of the target right
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;   //[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;  //[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;    //[rad]

                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                        new Target(signPrefix + signTarRight, tarRight_pos, tarRight_or),
                                        new Target(signPrefix + signTarLeft, tarLeft_pos, tarLeft_or),
                                        new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                Pose* ps = new Pose(signPrefix+string("_home"), tarRight_pos,tarRight_or,true,cnt_obj);

                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);

                //handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                //add the object to the scenario
                scene->addObject(objectPtr(ob));
                //add the pose to the scenario
                scene->addPose(posePtr(ps));
                cnt_obj++;

                if(signPrefix != "Shelf1" && signPrefix != "Shelf2" && signPrefix != "Shelf3" && signPrefix != "Shelf4" && signPrefix != "TableRight"){
                    //position of the target right 1
                    tarRight1_pos.Xpos = obj_info_vec.at(27)*1000; targetValues.push_back(obj_info_vec.at(27));//[mm]
                    tarRight1_pos.Ypos = obj_info_vec.at(28)*1000; targetValues.push_back(obj_info_vec.at(28));//[mm]
                    tarRight1_pos.Zpos = obj_info_vec.at(29)*1000; targetValues.push_back(obj_info_vec.at(29));//[mm]
                    //orientation of the target right 1
                    tarRight1_or.roll = obj_info_vec.at(30)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(30));  //[rad]
                    tarRight1_or.pitch = obj_info_vec.at(31)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(31)); //[rad]
                    tarRight1_or.yaw = obj_info_vec.at(32)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(32));   //[rad]

                    Target* tg_right = new Target(signPrefix+string("_Right"), tarRight_pos,tarRight_or);
                    Target* tg_left = new Target(signPrefix+string("_Left"), tarLeft_pos,tarLeft_or);
                    Target* tg_right1 = new Target(signPrefix+string("_UpRight"), tarRight1_pos,tarRight1_or);

                    Q_EMIT newTarget(tg_right->getName());
                    Q_EMIT newTarget(tg_left->getName());
                    Q_EMIT newTarget(tg_right1->getName());
                    //add the pose to the scenario
                    scene->addTarget(targetPtr(tg_right));
                    scene->addTarget(targetPtr(tg_left));
                    scene->addTarget(targetPtr(tg_right1));
                }

                if(signPrefix == "Rolos_cozinha"){
                    this->received_targets_Rolos_Cozinha_data = targetValues;
                }else if(signPrefix == "Pastilhas_maq_loica14"){
                    this->received_targets_Pastilhas_Maq_Loica_14_data = targetValues;
                }else if(signPrefix == "detergente_maq_loica10"){
                    this->received_targets_Detergente_Maq_Loica_10_data = targetValues;
                }
            }else{throw string("Error while retrieving the objects of the scenario");}
        }

        // **** Poses **** //
        // ---- Rolos Cozinha ---- //
        // ---- Movement 1 ---- //
        poses_prefix.push_back("Pose_approach1_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Pos_Pick_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1_Pos_Pick_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1_Pos_Drop_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose2_Pos_Drop_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1_right_Pre_Grasp_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        //-----------------
        poses_prefix.push_back("Pose_Pre_place_left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Pre_place_right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_place_left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_place_right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose0_Pos_place_left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Pos_place_right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Pos_place_left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        //-------------------------------------------------
        // ---- Movement 4 ---- //
        /*poses_prefix.push_back("Pose_Pre_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Pre_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1_Pre_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1_Pre_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose2_Pre_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose2_Pre_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose3_Pre_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose3_Pre_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose0_Pos_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pos_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pos_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pos_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pos_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pos_Place4_Left_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pos_Place4_Right_RC1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);*/
        //-------------------------------------------------

        // ---- Pastilhas 1 ----
        // ---- Grasp Vertical ----
        /*poses_prefix.push_back("Pose_approach1_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pos_Pick1_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pos_Pick1_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_right_Pos_Drop_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_right_Pos_Drop_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_right_Pre_Grasp_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);
        //------------ Place1 ----------------------

        poses_prefix.push_back("Pose_Pre_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pre_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pre_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pre_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pre_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pre_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose0_Pos_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pos_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose_Pos_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pos_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose1_Pos_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pos_Place1_Left_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);

        poses_prefix.push_back("Pose2_Pos_Place1_Right_P1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(1);*/
        //------------------------------------------

        // ---- Detergente 1 ----
        /*poses_prefix.push_back("Approach1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pos_Pick");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pos_Pick1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pos_Drop");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pos_Drop1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pre_Grasp");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);*/
        //------------------------------------------
        //----------- Place1 -----------------------
        /*poses_prefix.push_back("Pose_Pre_place1_left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pre_place1_right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_place1_left");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_place1_right");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose0_Pos_place1_left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pos_place1_left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pos_place1_right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);*/
        //------------------------------------------
        //----------- Place3 -----------------------
        /*poses_prefix.push_back("Pose_Pre_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pre_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose1_Pre_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose1_Pre_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose2_Pre_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose2_Pre_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose3_Pre_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose3_Pre_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pos_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose_Pos_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose1_Pos_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose1_Pos_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose2_Pos_Place3_Left_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);

        poses_prefix.push_back("Pose2_Pos_Place3_Right_D1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(3);*/
        //------------------------------------------
        //------------------------------------------

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if(srvs.response.result == 1){
              pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if(succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for(int k = 0; k < floatCount; ++k)
                pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                //position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                //orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                //add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }

        //get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if(srvs.response.result == 1){
          Hname = srvs.response.signalValue;
        }else{succ = false;}

        //get the handles of both arms
        succ = getArmsHandles(2);

        //transformation matrix for both arms
        add_client =  n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        //right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
          mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
          floatCount = mat_right_arm_str.size()/sizeof(float);
        for(int k = 0; k <floatCount; ++k)
          mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k){
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
        }

        rows = 0;
        for(int i = 0; i < 4 ; ++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        //Arms
        //Right Arm
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_right_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            DH_params_right_str = srvs.response.signalValue;
        }else{succ =  false; throw string("Error: Couldn't get the DH parameters of the arms");}

        floatCount = DH_params_right_str.size()/sizeof(float);
        if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));

        humanoid_arm_right_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.theta = std::vector<double>(7);
        for(int i = 0; i < 7; ++i){
            humanoid_arm_right_specs.arm_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;
            humanoid_arm_right_specs.arm_specs.a.at(i) = DH_params_right_vec.at(i+7)*1000;
            humanoid_arm_right_specs.arm_specs.d.at(i) = DH_params_right_vec.at(i+14)*1000;
            humanoid_arm_right_specs.arm_specs.theta.at(i) = DH_params_right_vec.at(i+21)*static_cast<double>(M_PI)/180;
        }

        //Left Arm
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_left_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            DH_params_left_str = srvs.response.signalValue;
        }else{succ =  false; throw string("Error: Couldn't get the DH parameters of the arms");}

        floatCount = DH_params_left_str.size()/sizeof(float);
        if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));

        humanoid_arm_left_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.theta = std::vector<double>(7);
        for(int i = 0; i < 7; ++i){
            humanoid_arm_left_specs.arm_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;
            humanoid_arm_left_specs.arm_specs.a.at(i) = DH_params_left_vec.at(i+7)*1000;
            humanoid_arm_left_specs.arm_specs.d.at(i) = DH_params_left_vec.at(i+14)*1000;
            humanoid_arm_left_specs.arm_specs.theta.at(i) = DH_params_left_vec.at(i+21)*static_cast<double>(M_PI)/180;
        }

        // **** Hands **** //
        // **** QbSoftHands **** //
        if(this->hand_code_right == 3){
            rposture.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);
            min_rlimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);
            max_rlimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}

            // **** Right Hand **** //
            // Thumb
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Thumb_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Thumb");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Thumb_right = Robot_hand_right.fingers.at(0);
            Thumb_right.finger_name = "Thumb";
            Thumb_right.finger_specs.alpha = std::vector<double>(7);
            Thumb_right.finger_specs.a = std::vector<double>(7);
            Thumb_right.finger_specs.d = std::vector<double>(7);
            Thumb_right.finger_specs.theta = std::vector<double>(7);
            for (int i = 0; i < 7 ; ++i) {
                Thumb_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;     //[rad]
                Thumb_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+7)*1000;                                //[mm]
                Thumb_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+14)*1000;                               //[mm]
                Thumb_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+21)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(0) = Thumb_right;

            // Index
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Index_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Index");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
             }
            Index_right = Robot_hand_right.fingers.at(1);
            Index_right.finger_name = "Index";
            Index_right.finger_specs.alpha = std::vector<double>(10);
            Index_right.finger_specs.a = std::vector<double>(10);
            Index_right.finger_specs.d = std::vector<double>(10);
            Index_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Index_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Index_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Index_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Index_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(1) = Index_right;

            // Middle
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Middle_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Middle");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Middle_right = Robot_hand_right.fingers.at(2);
            Middle_right.finger_name = "Middle";
            Middle_right.finger_specs.alpha = std::vector<double>(10);
            Middle_right.finger_specs.a = std::vector<double>(10);
            Middle_right.finger_specs.d = std::vector<double>(10);
            Middle_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Middle_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Middle_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Middle_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Middle_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(2) = Middle_right;

            // Ring
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Ring_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Ring");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Ring_right = Robot_hand_right.fingers.at(3);
            Ring_right.finger_name = "Ring";
            Ring_right.finger_specs.alpha = std::vector<double>(10);
            Ring_right.finger_specs.a = std::vector<double>(10);
            Ring_right.finger_specs.d = std::vector<double>(10);
            Ring_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Ring_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Ring_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Ring_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Ring_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(3) = Ring_right;

            // Little
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Little_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Little");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Little_right = Robot_hand_right.fingers.at(4);
            Little_right.finger_name = "Little";
            Little_right.finger_specs.alpha = std::vector<double>(10);
            Little_right.finger_specs.a = std::vector<double>(10);
            Little_right.finger_specs.d = std::vector<double>(10);
            Little_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Little_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Little_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Little_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Little_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(4) = Little_right;
            // ******************** //
        }else if(this->hand_code_right == 2){
            rposture.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            min_rlimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            max_rlimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            this->hand_code_right = 2;

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("minAperture_info");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                minAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("A1_info");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                A1_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("D3_info");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                D3_right = srvf.response.signalValue*1000;
            }else{succ = false;}
        }

        // **** Left Hand **** //
        if(this->hand_code_left == 3){

            // **** Home Postures **** //
            lposture.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // left

            // **** Joint Limits **** //
            min_llimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // minimum left limits
            max_llimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // maximum left limits

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}

            // Thumb
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Thumb_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Thumb");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Thumb_left = Robot_hand_left.fingers.at(0);
            Thumb_left.finger_name = "Thumb";
            Thumb_left.finger_specs.alpha = std::vector<double>(7);
            Thumb_left.finger_specs.a = std::vector<double>(7);
            Thumb_left.finger_specs.d = std::vector<double>(7);
            Thumb_left.finger_specs.theta = std::vector<double>(7);
            for (int i = 0; i < 7 ; ++i) {
                Thumb_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;     //[rad]
                Thumb_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+7)*1000;                                //[mm]
                Thumb_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+14)*1000;                               //[mm]
                Thumb_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+21)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(0) = Thumb_left;

            // Index
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Index_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Index");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Index_left = Robot_hand_left.fingers.at(1);
            Index_left.finger_name = "Index";
            Index_left.finger_specs.alpha = std::vector<double>(10);
            Index_left.finger_specs.a = std::vector<double>(10);
            Index_left.finger_specs.d = std::vector<double>(10);
            Index_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Index_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Index_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Index_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Index_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(1) = Index_left;

            // Middle
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Middle_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Middle");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Middle_left = Robot_hand_left.fingers.at(2);
            Middle_left.finger_name = "Middle";
            Middle_left.finger_specs.alpha = std::vector<double>(10);
            Middle_left.finger_specs.a = std::vector<double>(10);
            Middle_left.finger_specs.d = std::vector<double>(10);
            Middle_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Middle_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Middle_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Middle_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Middle_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(2) = Middle_left;

            // Ring
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Ring_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Ring");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Ring_left = Robot_hand_left.fingers.at(3);
            Ring_left.finger_name = "Ring";
            Ring_left.finger_specs.alpha = std::vector<double>(10);
            Ring_left.finger_specs.a = std::vector<double>(10);
            Ring_left.finger_specs.d = std::vector<double>(10);
            Ring_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Ring_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Ring_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Ring_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Ring_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(3) = Ring_left;

            // Little
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Little_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Little");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Little_left = Robot_hand_left.fingers.at(4);
            Little_left.finger_name = "Little";
            Little_left.finger_specs.alpha = std::vector<double>(10);
            Little_left.finger_specs.a = std::vector<double>(10);
            Little_left.finger_specs.d = std::vector<double>(10);
            Little_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Little_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Little_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Little_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Little_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(4) = Little_left;
            //**********************//
        }else if (this->hand_code_left == 2){
            // **** Home Postures **** //
            lposture.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // left

            // **** Joint Limits **** //
            min_llimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // minimum left limits
            max_llimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // maximum left limits

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("minAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                minAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("A1_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                A1_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("D3_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                D3_left = srvf.response.signalValue*1000;
            }else{succ = false;}
        }
        //______________________//

        //Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if(!torso_vec.empty()){torso_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000; //[mm]
        torso.Ypos = torso_vec.at(1)*1000; //[mm]
        torso.Zpos = torso_vec.at(2)*1000; //[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180;; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180;; //[rad]
        torso.Xsize = torso_vec.at(6)*1000; //[mm]
        torso.Ysize = torso_vec.at(7)*1000; //[mm]
        torso.Zsize = torso_vec.at(8)*1000; //[mm]

        //right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        for(size_t i = 0; i < rposture.size() ; i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if(srvf.response.result == 1){
               rposture.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }
        //minimum right limits
        for(size_t i = 0; i < min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if(srvf.response.result == 1){
               min_rlimits.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }
        //maximum right limits
        for(size_t i = 0; i < max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if(srvf.response.result == 1){
               max_rlimits.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }

        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        if(succ){
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll = torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            if(this->hand_code_right == 2){
                Humanoid_gripper_specs.maxAperture = maxAp_right;
                Humanoid_gripper_specs.minAperture = minAp_right;
                Humanoid_gripper_specs.A1 = A1_right;
                Humanoid_gripper_specs.D3 = D3_right;
            }else if(this->hand_code_right == 3)
                Robot_hand_right.maxAp = maxAp_right;

            if(this->hand_code_left == 2){
                Humanoid_gripper_specs.maxAperture = maxAp_left;
                Humanoid_gripper_specs.minAperture = minAp_left;
                Humanoid_gripper_specs.A1 = A1_left;
                Humanoid_gripper_specs.D3 = D3_left;
            }else if(this->hand_code_left == 3)
                Robot_hand_left.maxAp = maxAp_left;

            Humanoid *hptr = new Humanoid(Hname,humanoid_pos, humanoid_or,humanoid_size,humanoid_arm_right_specs, humanoid_arm_left_specs, this->hand_code_right, this->hand_code_left, rposture, lposture,
                                          min_rlimits, max_rlimits, min_llimits,max_llimits);

            if(this->hand_code_right == 2){
                vector<int> rkk;
                rkk.push_back(-1.0);
                rkk.push_back(1.0);

                hptr->setElectricGripper(Humanoid_gripper_specs);
                hptr->setRK(rkk);
            }else if(this->hand_code_right == 3)
                hptr->setRightQbSoftHand(Robot_hand_right);


            if(this->hand_code_left == 2){
                vector<int> rkk;
                rkk.push_back(-1.0);
                rkk.push_back(1.0);

                hptr->setElectricGripper(Humanoid_gripper_specs);
                hptr->setRK(rkk);
            }else if(this->hand_code_left == 3)
                hptr->setLeftQbSoftHand(Robot_hand_left);

            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            //get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            if(this->hand_code_right == 2){
                for(size_t i = 0; i < rightp.size(); i++){
                    if(i < rightp.size()-1){
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    }else{
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)).toStdString());
                    }
                    Q_EMIT newJoint(rj.at(i));
                }
            }else if(this->hand_code_right == 3){
                for(size_t i = 0; i < rightp.size(); i++){
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    Q_EMIT newJoint(rj.at(i));
                }
            }

            std::vector<string> lj = std::vector<string>(leftp.size());
            if(this->hand_code_left == 2){
                for(size_t i = 0; i < leftp.size(); i++){
                    if(i < leftp.size()-1){
                        lj.at(i) = string("left_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    }else{
                        lj.at(i) = string("left_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(leftp.at(i)).toStdString());
                    }
                    Q_EMIT newJoint(lj.at(i));
                }
            }else if(this->hand_code_left == 3){
                for (size_t i=0; i<leftp.size(); i++ ){
                    lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                           QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    Q_EMIT newJoint(lj.at(i));
                }
            }

            //display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));

        }else{
            throw string("Error while retrieving elements from the scenario.");
        }
    }else if(scenarioID == 2){
        add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");

        // **** Load Number of Objects **** //
        srvi.request.signalName = NOBJECTS;
        add_client.call(srvi);
        if(srvi.response.result == 1){
          n_objs = srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        // **** Load Number of Poses **** //
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if(srvi.response.result == 1){
          n_poses = srvi.response.signalValue;
        }else{succ = false; throw string("Communication error");}

        //get the object handle
        client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");

        // **** Objects **** //
        //obj_id = 0 (Cup)
        objs_prefix.push_back("Cup");

        //obj_id = 1 (Shelf1)
        objs_prefix.push_back("Shelf1");

        //obj_id = 2 (Shelf2)
        objs_prefix.push_back("Shelf2");

        //obj_id = 3 (Shelf3)
        //objs_prefix.push_back("Shelf3");

        //obj_id = 4 (Table)
        objs_prefix.push_back("Table");

        //Environment2
        objs_prefix.push_back("Cup1_4");
        objs_prefix.push_back("Cup1_8");
        objs_prefix.push_back("Cup1_12");
        objs_prefix.push_back("Cup1_16");
        objs_prefix.push_back("Cup1_19");
        objs_prefix.push_back("Cup2_4");
        objs_prefix.push_back("Cup2_8");
        objs_prefix.push_back("Cup2_12");
        objs_prefix.push_back("Cup2_16");
        objs_prefix.push_back("Cup2_19");
        /*objs_prefix.push_back("Cup3_4");
        objs_prefix.push_back("Cup3_8");
        objs_prefix.push_back("Cup3_12");
        objs_prefix.push_back("Cup3_16");
        objs_prefix.push_back("Cup3_19");*/

        //Environment3
        objs_prefix.push_back("Cup1");
        objs_prefix.push_back("Cup2");
        objs_prefix.push_back("Cup3");

        //Environment4
        objs_prefix.push_back("Person");
        //*******************//

        std::vector<float>targetValues;
        while(cnt_obj < n_objs){
            targetValues.clear();
            signPrefix = objs_prefix[cnt_obj];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                obj_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if(succ){
                floatCount = obj_info_str.size()/sizeof(float);
                if(!obj_info_vec.empty()){obj_info_vec.clear();}
                for(int k = 0; k < floatCount; ++k){
                    obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));
                }

                //position of the object
                obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
                obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
                obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
                //orientation of the object
                obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180;   //[rad]
                obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180;  //[rad]
                obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;    //[rad]
                //size of the object
                obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
                obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
                obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]

                //position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9)*1000; targetValues.push_back(obj_info_vec.at(9));//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10)*1000; targetValues.push_back(obj_info_vec.at(10));//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11)*1000; targetValues.push_back(obj_info_vec.at(11));//[mm]
                //orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(12)); //[rad]
                tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(13)); //[rad]
                tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(14));   //[rad]

                //position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15)*1000; targetValues.push_back(obj_info_vec.at(15));//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16)*1000; targetValues.push_back(obj_info_vec.at(16));//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17)*1000; targetValues.push_back(obj_info_vec.at(17));//[mm]
                //orientation of the target right
                tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(18)); //[rad]
                tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(19));//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180; targetValues.push_back(obj_info_vec.at(20));//[rad]

                //position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21)*1000; //[mm]
                engage_pos.Ypos = obj_info_vec.at(22)*1000; //[mm]
                engage_pos.Zpos = obj_info_vec.at(23)*1000; //[mm]
                //orientation of the target right
                engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;   //[rad]
                engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;  //[rad]
                engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;    //[rad]

                Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                        new Target(signPrefix + signTarRight, tarRight_pos, tarRight_or),
                                        new Target(signPrefix + signTarLeft, tarLeft_pos, tarLeft_or),
                                        new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

                Pose* ps = new Pose(signPrefix+string("_home"), tarRight_pos,tarRight_or,true,cnt_obj);

                infoLine = ob->getInfoLine();
                Q_EMIT newElement(infoLine);
                Q_EMIT newObject(ob->getName());
                Q_EMIT newPose(ps->getName());

                //handle of the object
                srv_get_handle.request.objectName = signPrefix;
                client_getHandle.call(srv_get_handle);
                ob->setHandle(srv_get_handle.response.handle);

                //handle of the visible object
                srv_get_handle.request.objectName = signPrefix+string("_body");
                client_getHandle.call(srv_get_handle);
                ob->setHandleBody(srv_get_handle.response.handle);

                //add the object to the scenario
                scene->addObject(objectPtr(ob));
                //add the pose to the scenario
                scene->addPose(posePtr(ps));
                cnt_obj++;

                if(signPrefix != "Shelf1" && signPrefix != "Shelf2" && signPrefix != "Shelf3" && signPrefix != "Table" &&
                        signPrefix != "Cup1_4" && signPrefix != "Cup1_8" && signPrefix != "Cup1_12" && signPrefix != "Cup1_16" && signPrefix != "Cup1_19" &&
                        signPrefix != "Cup2_4" && signPrefix != "Cup2_8" && signPrefix != "Cup2_12" && signPrefix != "Cup2_16" && signPrefix != "Cup2_19" &&
                        signPrefix != "Cup3_4" && signPrefix != "Cup3_8" && signPrefix != "Cup3_12" && signPrefix != "Cup3_16" && signPrefix != "Cup3_19" &&
                        signPrefix != "Cup1" && signPrefix != "Cup2" && signPrefix != "Cup3" &&
                        signPrefix != "Person"){
                    Target* tg_right = new Target(signPrefix+string("_Right"), tarRight_pos,tarRight_or);
                    Target* tg_left = new Target(signPrefix+string("_Left"), tarLeft_pos,tarLeft_or);

                    Q_EMIT newTarget(tg_right->getName());
                    Q_EMIT newTarget(tg_left->getName());
                    //add the pose to the scenario
                    scene->addTarget(targetPtr(tg_right));
                    scene->addTarget(targetPtr(tg_left));
                }

                if(signPrefix == "Cup"){
                    this->received_targets_Cup_data = targetValues;
                }
            }else{throw string("Error while retrieving the objects of the scenario");}
        }

        // **** Poses **** //
        // **** Placement 1 **** //
        poses_prefix.push_back("Pose0");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose1");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose5");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose6");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);

        poses_prefix.push_back("Pose7");
        poses_rel.push_back(true);
        poses_obj_id.push_back(0);
        // ********************* //

        while(cnt_pose < n_poses){
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if(srvs.response.result == 1){
              pose_info_str = srvs.response.signalValue;
            }else{succ = false;}

            if(succ){
                floatCount = pose_info_str.size()/sizeof(float);
                if(!pose_info_vec.empty()){pose_info_vec.clear();}
                for(int k = 0; k < floatCount; ++k)
                pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                //position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                //orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                //add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }else{
                throw string("Error while retrieving the poses of the scenario");
            }
        }
        //get the info of the Humanoid
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

        srvs.request.signalName = string("HumanoidName");
        add_client.call(srvs);
        if(srvs.response.result == 1){
          Hname = srvs.response.signalValue;
        }else{succ = false;}

        //get the handles of both arms
        succ = getArmsHandles(2);

        //transformation matrix for both arms
        add_client =  n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        //right arm
        srvs.request.signalName = string("mat_right_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
          mat_right_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_right_arm_vec.empty()){mat_right_arm_vec.clear();}
          floatCount = mat_right_arm_str.size()/sizeof(float);
        for(int k = 0; k <floatCount; ++k)
          mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));
        // left arm
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1){
             mat_left_arm_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the transformation matrix of the arms");}
        if(!mat_left_arm_vec.empty()){mat_left_arm_vec.clear();}
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k){
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
        }

        rows = 0;
        rows = 0;
        for(int i = 0; i < 4 ; ++i){
            for(int j=0;j<4;++j){
                if(i==3 && j<3){
                    mat_right(i,j) = 0;
                    mat_left(i,j) = 0;
                }else if(i==3 && j==3){
                    mat_right(i,j) = 1;
                    mat_left(i,j) = 1;
                }else if(i<3 && j==3){
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
                }else{
                    mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
                }
            }
            ++rows;
        }

        //Arms
        //Right Arm
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_right_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            DH_params_right_str = srvs.response.signalValue;
        }else{succ =  false; throw string("Error: Couldn't get the DH parameters of the arms");}

        floatCount = DH_params_right_str.size()/sizeof(float);
        if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));

        humanoid_arm_right_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_right_specs.arm_specs.theta = std::vector<double>(7);
        for(int i = 0; i < 7; ++i){
            humanoid_arm_right_specs.arm_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;
            humanoid_arm_right_specs.arm_specs.a.at(i) = DH_params_right_vec.at(i+7)*1000;
            humanoid_arm_right_specs.arm_specs.d.at(i) = DH_params_right_vec.at(i+14)*1000;
            humanoid_arm_right_specs.arm_specs.theta.at(i) = DH_params_right_vec.at(i+21)*static_cast<double>(M_PI)/180;
        }

        //Left Arm
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("DH_params_left_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            DH_params_left_str = srvs.response.signalValue;
        }else{succ =  false; throw string("Error: Couldn't get the DH parameters of the arms");}

        floatCount = DH_params_left_str.size()/sizeof(float);
        if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));

        humanoid_arm_left_specs.arm_specs.alpha = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.a = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.d = std::vector<double>(7);
        humanoid_arm_left_specs.arm_specs.theta = std::vector<double>(7);
        for(int i = 0; i < 7; ++i){
            humanoid_arm_left_specs.arm_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;
            humanoid_arm_left_specs.arm_specs.a.at(i) = DH_params_left_vec.at(i+7)*1000;
            humanoid_arm_left_specs.arm_specs.d.at(i) = DH_params_left_vec.at(i+14)*1000;
            humanoid_arm_left_specs.arm_specs.theta.at(i) = DH_params_left_vec.at(i+21)*static_cast<double>(M_PI)/180;
        }

        // **** Hands **** //
        // **** QbSoftHands **** //
        if(this->hand_code_right == 3){
            rposture.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);
            min_rlimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);
            max_rlimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand);

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}

            // **** Right Hand **** //
            // Thumb
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Thumb_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Thumb");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Thumb_right = Robot_hand_right.fingers.at(0);
            Thumb_right.finger_name = "Thumb";
            Thumb_right.finger_specs.alpha = std::vector<double>(7);
            Thumb_right.finger_specs.a = std::vector<double>(7);
            Thumb_right.finger_specs.d = std::vector<double>(7);
            Thumb_right.finger_specs.theta = std::vector<double>(7);
            for (int i = 0; i < 7 ; ++i) {
                Thumb_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;     //[rad]
                Thumb_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+7)*1000;                                //[mm]
                Thumb_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+14)*1000;                               //[mm]
                Thumb_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+21)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(0) = Thumb_right;

            // Index
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Index_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Index");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
             }
            Index_right = Robot_hand_right.fingers.at(1);
            Index_right.finger_name = "Index";
            Index_right.finger_specs.alpha = std::vector<double>(10);
            Index_right.finger_specs.a = std::vector<double>(10);
            Index_right.finger_specs.d = std::vector<double>(10);
            Index_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Index_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Index_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Index_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Index_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(1) = Index_right;

            // Middle
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Middle_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Middle");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Middle_right = Robot_hand_right.fingers.at(2);
            Middle_right.finger_name = "Middle";
            Middle_right.finger_specs.alpha = std::vector<double>(10);
            Middle_right.finger_specs.a = std::vector<double>(10);
            Middle_right.finger_specs.d = std::vector<double>(10);
            Middle_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Middle_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Middle_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Middle_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Middle_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(2) = Middle_right;

            // Ring
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Ring_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Ring");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Ring_right = Robot_hand_right.fingers.at(3);
            Ring_right.finger_name = "Ring";
            Ring_right.finger_specs.alpha = std::vector<double>(10);
            Ring_right.finger_specs.a = std::vector<double>(10);
            Ring_right.finger_specs.d = std::vector<double>(10);
            Ring_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Ring_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Ring_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Ring_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Ring_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(3) = Ring_right;

            // Little
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Little_right");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_right_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Little");}
            floatCount = DH_params_right_str.size()/sizeof(float);
            if(!DH_params_right_vec.empty()){DH_params_right_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_right_vec.push_back(static_cast<double>(((float*)DH_params_right_str.c_str())[k]));
            }
            Little_right = Robot_hand_right.fingers.at(4);
            Little_right.finger_name = "Little";
            Little_right.finger_specs.alpha = std::vector<double>(10);
            Little_right.finger_specs.a = std::vector<double>(10);
            Little_right.finger_specs.d = std::vector<double>(10);
            Little_right.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Little_right.finger_specs.alpha.at(i) = DH_params_right_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Little_right.finger_specs.a.at(i) = DH_params_right_vec.at(i+10)*1000;                                //[mm]
                Little_right.finger_specs.d.at(i) = DH_params_right_vec.at(i+20)*1000;                               //[mm]
                Little_right.finger_specs.theta.at(i) = DH_params_right_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_right.fingers.at(4) = Little_right;
            // ******************** //
        }else if(this->hand_code_right == 2){
            rposture.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            min_rlimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            max_rlimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper);
            this->hand_code_right = 2;

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("minAperture_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                minAp_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("A1_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                A1_right = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("D3_info_right");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                D3_right = srvf.response.signalValue*1000;
            }else{succ = false;}
        }

        // **** Left Hand **** //
        if(this->hand_code_left == 3){

            // **** Home Postures **** //
            lposture.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // left

            // **** Joint Limits **** //
            min_llimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // minimum left limits
            max_llimits.resize(JOINTS_ARM + JOINTS_HAND_QbSoftHand); // maximum left limits

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}

            // Thumb
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Thumb_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Thumb");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Thumb_left = Robot_hand_left.fingers.at(0);
            Thumb_left.finger_name = "Thumb";
            Thumb_left.finger_specs.alpha = std::vector<double>(7);
            Thumb_left.finger_specs.a = std::vector<double>(7);
            Thumb_left.finger_specs.d = std::vector<double>(7);
            Thumb_left.finger_specs.theta = std::vector<double>(7);
            for (int i = 0; i < 7 ; ++i) {
                Thumb_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;     //[rad]
                Thumb_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+7)*1000;                                //[mm]
                Thumb_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+14)*1000;                               //[mm]
                Thumb_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+21)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(0) = Thumb_left;

            // Index
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Index_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Index");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Index_left = Robot_hand_left.fingers.at(1);
            Index_left.finger_name = "Index";
            Index_left.finger_specs.alpha = std::vector<double>(10);
            Index_left.finger_specs.a = std::vector<double>(10);
            Index_left.finger_specs.d = std::vector<double>(10);
            Index_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Index_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Index_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Index_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Index_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(1) = Index_left;

            // Middle
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Middle_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Middle");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Middle_left = Robot_hand_left.fingers.at(2);
            Middle_left.finger_name = "Middle";
            Middle_left.finger_specs.alpha = std::vector<double>(10);
            Middle_left.finger_specs.a = std::vector<double>(10);
            Middle_left.finger_specs.d = std::vector<double>(10);
            Middle_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Middle_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Middle_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Middle_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Middle_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(2) = Middle_left;

            // Ring
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Ring_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Ring");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Ring_left = Robot_hand_left.fingers.at(3);
            Ring_left.finger_name = "Ring";
            Ring_left.finger_specs.alpha = std::vector<double>(10);
            Ring_left.finger_specs.a = std::vector<double>(10);
            Ring_left.finger_specs.d = std::vector<double>(10);
            Ring_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Ring_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Ring_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Ring_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Ring_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(3) = Ring_left;

            // Little
            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = string("DH_params_Little_left");
            add_client.call(srvs);
            if(srvs.response.result == 1){
                DH_params_left_str = srvs.response.signalValue;
            }else{succ = false; throw string("Error: Couldn't get the DH parameters of the Little");}
            floatCount = DH_params_left_str.size()/sizeof(float);
            if(!DH_params_left_vec.empty()){DH_params_left_vec.clear();}
            for(int k = 0; k < floatCount; ++k){
                DH_params_left_vec.push_back(static_cast<double>(((float*)DH_params_left_str.c_str())[k]));
            }
            Little_left = Robot_hand_left.fingers.at(4);
            Little_left.finger_name = "Little";
            Little_left.finger_specs.alpha = std::vector<double>(10);
            Little_left.finger_specs.a = std::vector<double>(10);
            Little_left.finger_specs.d = std::vector<double>(10);
            Little_left.finger_specs.theta = std::vector<double>(10);
            for (int i = 0; i < 10 ; ++i) {
                Little_left.finger_specs.alpha.at(i) = DH_params_left_vec.at(i)*static_cast<double>(M_PI)/180;   //[rad]
                Little_left.finger_specs.a.at(i) = DH_params_left_vec.at(i+10)*1000;                                //[mm]
                Little_left.finger_specs.d.at(i) = DH_params_left_vec.at(i+20)*1000;                               //[mm]
                Little_left.finger_specs.theta.at(i) = DH_params_left_vec.at(i+30)*static_cast<double>(M_PI)/180;  //[rad]
            }
            Robot_hand_left.fingers.at(4) = Little_left;
            //**********************//
        }else if (this->hand_code_left == 2){
            // **** Home Postures **** //
            lposture.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // left

            // **** Joint Limits **** //
            min_llimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // minimum left limits
            max_llimits.resize(JOINTS_ARM + JOINTS_HAND_Electric_Gripper); // maximum left limits

            add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
            srvf.request.signalName = string("maxAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                maxAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("minAperture_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                minAp_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("A1_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                A1_left = srvf.response.signalValue*1000;
            }else{succ = false;}
            srvf.request.signalName = string("D3_info_left");
            add_client.call(srvf);
            if(srvf.response.result == 1){
                D3_left = srvf.response.signalValue*1000;
            }else{succ = false;}
        }
        //______________________//

        //Torso
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("TorsoInfo");
        add_client.call(srvs);
        if(srvs.response.result == 1){
            torso_str = srvs.response.signalValue;
        }else{succ = false; throw string("Error: Couldn't get the information of the torso");}
        floatCount = torso_str.size()/sizeof(float);
        if(!torso_vec.empty()){torso_vec.clear();}
        for(int k = 0; k < floatCount; ++k)
            torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

        torso.Xpos = torso_vec.at(0)*1000; //[mm]
        torso.Ypos = torso_vec.at(1)*1000; //[mm]
        torso.Zpos = torso_vec.at(2)*1000; //[mm]
        torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
        torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180;; //[rad]
        torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180;; //[rad]
        torso.Xsize = torso_vec.at(6)*1000; //[mm]
        torso.Ysize = torso_vec.at(7)*1000; //[mm]
        torso.Zsize = torso_vec.at(8)*1000; //[mm]

        //right home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        for(size_t i = 0; i < rposture.size() ; i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if(srvf.response.result == 1){
               rposture.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }
        //minimum right limits
        for(size_t i = 0; i < min_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if(srvf.response.result == 1){
               min_rlimits.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }
        //maximum right limits
        for(size_t i = 0; i < max_rlimits.size(); i++){
            srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if(srvf.response.result == 1){
               max_rlimits.at(i) = srvf.response.signalValue;
            }else{succ = false;}
        }

        // left home posture
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        for (size_t i = 0; i <lposture.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 lposture.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // minimum left limits
        for (size_t i = 0; i <min_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 min_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }
        // maximum left limits
        for (size_t i = 0; i <max_llimits.size(); i++){
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1){
                 max_llimits.at(i)= srvf.response.signalValue;
            }else{succ = false;}
        }

        if(succ){
            humanoid_pos.Xpos = torso.Xpos;
            humanoid_pos.Ypos = torso.Ypos;
            humanoid_pos.Zpos = torso.Zpos;
            humanoid_or.roll = torso.Roll;
            humanoid_or.pitch = torso.Pitch;
            humanoid_or.yaw = torso.Yaw;
            humanoid_size.Xsize = torso.Xsize;
            humanoid_size.Ysize = torso.Ysize;
            humanoid_size.Zsize = torso.Zsize;
            if(this->hand_code_right == 2){
                Humanoid_gripper_specs.maxAperture = maxAp_right;
                Humanoid_gripper_specs.minAperture = minAp_right;
                Humanoid_gripper_specs.A1 = A1_right;
                Humanoid_gripper_specs.D3 = D3_right;
            }else if(this->hand_code_right == 3)
                Robot_hand_right.maxAp = maxAp_right;

            if(this->hand_code_left == 2){
                Humanoid_gripper_specs.maxAperture = maxAp_left;
                Humanoid_gripper_specs.minAperture = minAp_left;
                Humanoid_gripper_specs.A1 = A1_left;
                Humanoid_gripper_specs.D3 = D3_left;
            }else if(this->hand_code_left == 3)
                Robot_hand_left.maxAp = maxAp_left;

            Humanoid *hptr = new Humanoid(Hname,humanoid_pos, humanoid_or,humanoid_size,humanoid_arm_right_specs, humanoid_arm_left_specs, this->hand_code_right, this->hand_code_left, rposture, lposture,
                                          min_rlimits, max_rlimits, min_llimits,max_llimits);

            if(this->hand_code_right == 2){
                vector<int> rkk;
                rkk.push_back(-1.0);
                rkk.push_back(1.0);

                hptr->setElectricGripper(Humanoid_gripper_specs);
                hptr->setRK(rkk);
            }else if(this->hand_code_right == 3)
                hptr->setRightQbSoftHand(Robot_hand_right);


            if(this->hand_code_left == 2){
                vector<int> rkk;
                rkk.push_back(-1.0);
                rkk.push_back(1.0);

                hptr->setElectricGripper(Humanoid_gripper_specs);
                hptr->setRK(rkk);
            }else if(this->hand_code_left == 3)
                hptr->setLeftQbSoftHand(Robot_hand_left);

            hptr->setMatRight(mat_right);
            hptr->setMatLeft(mat_left);

            //get the postures
            std::vector<double> rightp;
            std::vector<double> leftp;
            hptr->getRightPosture(rightp);
            hptr->getLeftPosture(leftp);
            std::vector<string> rj = std::vector<string>(rightp.size());
            if(this->hand_code_right == 2){
                for(size_t i = 0; i < rightp.size(); i++){
                    if(i < rightp.size()-1){
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    }else{
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)).toStdString());
                    }
                    Q_EMIT newJoint(rj.at(i));
                }
            }else if(this->hand_code_right == 3){
                for(size_t i = 0; i < rightp.size(); i++){
                    rj.at(i) = string("right_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    Q_EMIT newJoint(rj.at(i));
                }
            }

            std::vector<string> lj = std::vector<string>(leftp.size());
            if(this->hand_code_left == 2){
                for(size_t i = 0; i < leftp.size(); i++){
                    if(i < leftp.size()-1){
                        lj.at(i) = string("left_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    }else{
                        lj.at(i) = string("left_joint " + QString::number(i+1).toStdString() + ": " +
                                        QString::number(leftp.at(i)).toStdString());
                    }
                    Q_EMIT newJoint(lj.at(i));
                }
            }else if(this->hand_code_left == 3){
                for (size_t i=0; i<leftp.size(); i++ ){
                    lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                           QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString());
                    Q_EMIT newJoint(lj.at(i));
                }
            }

            //display info of the humanoid
            infoLine = hptr->getInfoLine();
            Q_EMIT newElement(infoLine);
            scene->addHumanoid(humanoidPtr(hptr));

        }else{
            throw string("Error while retrieving elements from the scenario.");
        }
    }

    this->curr_scene = scene;

    // stop the simulation
    this->stopSim();
    got_scene = true;

    return succ;

}

// **** OBJECTS CALLBACKS **** //
void QNode::RolosCozinhaCallback(const geometry_msgs::PoseStamped &data)
{

    int obj_id = 0;
    string name = string("Rolos_cozinha");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::targetsRolosCozinhaCallback(const std_msgs::Float32MultiArray &targetsdata){
    this->received_targets_Rolos_Cozinha_data = targetsdata.data;
}

void QNode::targetsPastilhasMaqLoica14Callback(const std_msgs::Float32MultiArray &targetsdata){
    this->received_targets_Pastilhas_Maq_Loica_14_data = targetsdata.data;
}

void QNode::targetsDetergenteMaqLoica10Callback(const std_msgs::Float32MultiArray &targetsdata){
    this->received_targets_Detergente_Maq_Loica_10_data = targetsdata.data;
}

void QNode::targetsCupCallback(const std_msgs::Float32MultiArray &targetsdata){
    this->received_targets_Cup_data = targetsdata.data;
}

void QNode::PastilhasMaqLoicaCallback(const geometry_msgs::PoseStamped &data)
{

    /*int obj_id = 1;
    string name = string("Pastilhas_maq_loica14");

    this->updateObjectInfo(obj_id,name,data);*/

}

void QNode::DetergenteMaqLoicaCallback(const geometry_msgs::PoseStamped &data)
{

    /*int obj_id = 2;
    string name = string("detergente_maq_loica10");

    this->updateObjectInfo(obj_id,name,data);*/

}

void QNode::CupCallback(const geometry_msgs::PoseStamped &data){
    int obj_id = 0;
    string name = string("Cup");

    this->updateObjectInfo(obj_id, name, data);
}

void QNode::Shelf1Callback(const geometry_msgs::PoseStamped &data)
{

    //int obj_id = 3;
    int obj_id = 1;
    string name = string("Shelf1");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::Shelf2Callback(const geometry_msgs::PoseStamped &data)
{

    int obj_id = 2;
    //int obj_id = 4;
    string name = string("Shelf2");

    this->updateObjectInfo(obj_id,name,data);

}

void QNode::Shelf3Callback(const geometry_msgs::PoseStamped &data)
{

    //int obj_id = 5;
    /*int obj_id = 2;
    string name = string("Shelf3");

    this->updateObjectInfo(obj_id,name,data);*/
}

void QNode::Shelf4Callback(const geometry_msgs::PoseStamped &data)
{

    /*//int obj_id = 6;
    string name = string("Shelf4");

    this->updateObjectInfo(obj_id,name,data);*/

}

void QNode::TableCallback(const geometry_msgs::PoseStamped &data)
{

    //int obj_id = 7;
    int obj_id = 3;
    //string name = string("TableRight");
    string name = string("Table");

    this->updateObjectInfo(obj_id,name,data);

}

//Environment2

void QNode::Cup1_4_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 4;
    string name = string("Cup1_4");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup1_8_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 5;
    string name = string("Cup1_8");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup1_12_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 6;
    string name = string("Cup1_12");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup1_16_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 7;
    string name = string("Cup1_16");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup1_19_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 8;
    string name = string("Cup1_19");
    this->updateObjectInfo(obj_id,name,data);
}

void QNode::Cup2_4_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 9;
    //int obj_id = 4;
    string name = string("Cup2_4");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup2_8_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 10;
    //int obj_id = 5;
    string name = string("Cup2_8");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup2_12_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 11;
    //int obj_id = 6;
    string name = string("Cup2_12");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup2_16_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 12;
    //int obj_id = 7;
    string name = string("Cup2_16");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup2_19_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 13;
    //int obj_id = 8;
    string name = string("Cup2_19");
    this->updateObjectInfo(obj_id,name,data);
}
/*
void QNode::Cup3_4_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 9;
    string name = string("Cup3_4");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup3_8_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 10;
    string name = string("Cup3_8");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup3_12_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 11;
    string name = string("Cup3_12");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup3_16_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 12;
    string name = string("Cup3_16");
    this->updateObjectInfo(obj_id,name,data);
}
void QNode::Cup3_19_Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 13;
    string name = string("Cup3_19");
    this->updateObjectInfo(obj_id,name,data);
}*/
//------------------------------------------------------------------

//Environment3

void QNode::Cup1Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 14;
    string name = string("Cup1");

    this->updateObjectInfo(obj_id, name, data);
}
void QNode::Cup2Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 15;
    string name = string("Cup2");

    this->updateObjectInfo(obj_id, name, data);
}
void QNode::Cup3Callback(const geometry_msgs::PoseStamped &data){
    int obj_id = 16;
    string name = string("Cup3");

    this->updateObjectInfo(obj_id, name, data);
}
//------------------------------------------------------------------

//Environment4

void QNode::PersonCallback(const geometry_msgs::PoseStamped &data){
    int obj_id = 17;
    string name = string("Person");

    this->updateObjectInfo(obj_id, name, data);
}
// ************************** //

void QNode::updateObjectInfo(int obj_id, string name, const geometry_msgs::PoseStamped &data)
{
    objectPtr obj = this->curr_scene->getObject(name);

    if(name == "Cup"){
        // position
        pos poss;
        poss.Xpos = data.pose.position.x * 1000; //[mm]
        poss.Ypos = data.pose.position.y * 1000; //[mm]
        poss.Zpos = data.pose.position.z * 1000; //[mm]

        obj->setPos(poss,true);

        // orientation
        /*Quaterniond orr_q;
        // get the quaternion
        double epx = data.pose.orientation.x;
        double epy = data.pose.orientation.y;
        double epz = data.pose.orientation.z;
        double w = data.pose.orientation.w;
        orr_q.x() = epx;
        orr_q.y() = epy;
        orr_q.z() = epz;
        orr_q.w() = w;

        obj->setOr(orr_q,true);*/
    }

    string info = obj->getInfoLine();
    Q_EMIT updateElement(obj_id,info);
    if(this->curr_scene){
        this->curr_scene->setObject(obj_id,obj);
    }

}

void QNode::Target_pose_Callback(const geometry_msgs::PoseStamped& data)
{
    targetPtr h_tar = this->curr_scene->getHandTarget();

    // position
    pos poss;
    poss.Xpos = data.pose.position.x; //[mm]
    poss.Ypos = data.pose.position.y; //[mm]
    poss.Zpos = data.pose.position.z; //[mm]

    // orientation
    Quaterniond orr_q;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;
    orr_q.x() = epx;
    orr_q.y() = epy;
    orr_q.z() = epz;
    orr_q.w() = w;

    h_tar->setPos(poss);
    h_tar->setQuaternion(orr_q);
    this->curr_scene->setHandTarget(h_tar);
}

bool QNode::getRPY(Matrix4d Trans, std::vector<double> &rpy)
{

    rpy = std::vector<double>(3);


    if((abs(Trans(0,0))) < 1e-5 && (abs(Trans(1,0))) < 1e-5){
        // singularity
        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

        return false;

    }else{

        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        double sp = sin(rpy.at(0));
        double cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

        return true;
    }
}

void QNode::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty()){
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        // Rot = Rot_z * Rot_y * Rot_x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);
    }
}

void QNode::infoCallback(const vrep_common::VrepInfoConstPtr& info)
{

    simulationTime=info->simulationTime.data;
    simulationTimeStep=info->timeStep.data;
    simulationRunning=(info->simulatorState.data&1)!=0;
}

void QNode::rightProxCallback(const vrep_common::ProximitySensorData& data)
{
    if (this->curr_mov){
        int arm_code = this->curr_mov->getArm();
        //right arm
        int h_obj;
        int h_obj_body;
        int mov_type = this->curr_mov->getType();

        switch (mov_type) {
        case 0: // reach-to-grasp
            h_obj_body = this->curr_mov->getObject()->getHandleBody(); // visible handle of the object we want to grasp
            h_obj = this->curr_mov->getObject()->getHandle(); // non visible handle of the object we want to grasp
            if (arm_code == 1)
            {
                // right arm
                h_detobj = data.detectedObject.data; // handle of the object currently detected
            }else if(arm_code == 0){
                // dual arm
                r_h_detobj = data.detectedObject.data; // handle of the object currently detected
            }

            if (arm_code == 1)
            {
                // right arm
                obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
            }else if(arm_code == 0){
                // dual arm
                obj_in_r_hand = (h_obj == r_h_detobj) || (h_obj_body == r_h_detobj);
            }
            break;
        case 1: // reaching
            break;
        case 2: // transport
            break;
        case 3: // engage
            break;
        case 4: // disengage
            break;
        case 5: // go park
            break;
        case 6: // Retreating
            break;
        }
    }
}

void QNode::leftProxCallback(const vrep_common::ProximitySensorData& data)
{
    if (this->curr_mov){
        int arm_code = this->curr_mov->getArm();
        //left arm
        int h_obj;
        int h_obj_body;
        int mov_type = this->curr_mov->getType();
        switch (mov_type){
        case 0: // reach-to-grasp
            if (arm_code == 2)
            {
                // left arm
                h_obj_body = this->curr_mov->getObject()->getHandleBody(); // visible handle of the object we want to grasp
                h_obj = this->curr_mov->getObject()->getHandle(); // non visible handle of the object we want to grasp
                h_detobj = data.detectedObject.data; // handle of the object currently detected
            }else if(arm_code == 0){
                // dual arm
                h_obj_body = this->curr_mov->getObjectLeft()->getHandleBody(); // visible handle of the object we want to grasp
                h_obj = this->curr_mov->getObjectLeft()->getHandle(); // non visible handle of the object we want to grasp
                l_h_detobj = data.detectedObject.data; // handle of the object currently detected
            }

            if (arm_code == 2)
            {
                // left arm
                obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
            }else if(arm_code == 0){
                // dual arm
                obj_in_l_hand = (h_obj == l_h_detobj) || (h_obj_body == l_h_detobj);
            }
            break;
        case 1: // reaching
            break;
        case 2: // transport
            break;
        case 3: // engage
            break;
        case 4: // disengage
            break;
        case 5: // go park
            break;
        case 6 : // Retreating
            break;
        }
    }
}

void QNode::rightHandPosCallback(const geometry_msgs::PoseStamped& data)
{
    vector<double> hand_pos_mes(6);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    vector<double> rpy;
    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy)){
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
    }else{
        // TO DO
        // singularity: leave the previous orientation
    }

    hand_pos_mes.at(0) = poss.Xpos;
    hand_pos_mes.at(1) = poss.Ypos;
    hand_pos_mes.at(2) = poss.Zpos;
    hand_pos_mes.at(3) = orr.roll;
    hand_pos_mes.at(4) = orr.pitch;
    hand_pos_mes.at(5) = orr.yaw;

    this->curr_scene->getHumanoid()->setHandPosMes(1,hand_pos_mes);

}

void QNode::rightHandVelCallback(const geometry_msgs::TwistStamped& data)
{
    vector<double> hand_vel_mes(6);

    hand_vel_mes.at(0) = data.twist.linear.x;
    hand_vel_mes.at(1) = data.twist.linear.y;
    hand_vel_mes.at(2) = data.twist.linear.z;
    hand_vel_mes.at(3) = data.twist.angular.x;
    hand_vel_mes.at(4) = data.twist.angular.y;
    hand_vel_mes.at(5) = data.twist.angular.z;

    this->curr_scene->getHumanoid()->setHandVelMes(1,hand_vel_mes);

}

void QNode::leftHandPosCallback(const geometry_msgs::PoseStamped& data)
{
    vector<double> hand_pos_mes(6);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    vector<double> rpy;
    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy)){
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
    }else{
        // TO DO
        // singularity: leave the previous orientation
    }

    hand_pos_mes.at(0) = poss.Xpos;
    hand_pos_mes.at(1) = poss.Ypos;
    hand_pos_mes.at(2) = poss.Zpos;
    hand_pos_mes.at(3) = orr.roll;
    hand_pos_mes.at(4) = orr.pitch;
    hand_pos_mes.at(5) = orr.yaw;

    this->curr_scene->getHumanoid()->setHandPosMes(2,hand_pos_mes);
}

void QNode::leftHandVelCallback(const geometry_msgs::TwistStamped& data)
{
    vector<double> hand_vel_mes(6);

    hand_vel_mes.at(0) = data.twist.linear.x;
    hand_vel_mes.at(1) = data.twist.linear.y;
    hand_vel_mes.at(2) = data.twist.linear.z;
    hand_vel_mes.at(3) = data.twist.angular.x;
    hand_vel_mes.at(4) = data.twist.angular.y;
    hand_vel_mes.at(5) = data.twist.angular.z;

    this->curr_scene->getHumanoid()->setHandVelMes(2,hand_vel_mes);
}

bool QNode::execMovement(std::vector<MatrixXd>& traj_mov, std::vector<MatrixXd>& vel_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string>& traj_descr,movementPtr mov, scenarioPtr scene, bool vel_mode)
{
    this->curr_scene = scene;
    this->curr_mov = mov;
    int mov_type = mov->getType();
    int arm_code = mov->getArm();
    bool plan;
    bool approach;
    bool retreat;
    bool hand_closed;

    switch (mov_type){
    case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
        #if HAND == 0 || HAND == 1
        // **** Human and Barrett Hand **** //
          closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
        #else
            closed = false;
        #endif
        break;
    case 2: case 3: case 4: // transport, engage, disengage
        #if HAND == 0 || HAND == 1
        // **** Human and Barrett Hand **** //
          closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
        #else
            closed = true;
        #endif
        break;
    }//switch(mov_type)

    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    double pre_time= 0.0;

    int h_attach; // handle of the attachment point of the hand
    int r_h_attach; // handle of the attachment point of the hand
    int l_h_attach; // handle of the attachment point of the hand

    std::vector<int> handles;
    std::vector<int> r_handles;
    std::vector<int> l_handles;

    #if HAND == 0 || HAND == 1
    // **** Human, Barrett Hand and QbSoftHand **** //
        MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
        MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
        MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    #endif

    switch (arm_code){
    case 0: // dual arm
        r_handles = right_handles;
        l_handles = left_handles;
        #if HAND == 0 || HAND == 1
        // **** Human, Barrett Hand **** //
            r_hand_handles = right_hand_handles;
            l_hand_handles = left_hand_handles;
        #endif
        r_h_attach = right_attach;
        l_h_attach = left_attach;
    break;
    case 1: //right arm
        handles = right_handles;
        #if HAND == 0 || HAND == 1
        // **** Human, Barrett Hand **** //
            hand_handles = right_hand_handles;
        #endif
        h_attach = right_attach;
    break;
    case 2: // left arm
        handles = left_handles;
        #if HAND == 0 || HAND == 1
        // **** Human, Barrett Hand **** //
            hand_handles = left_hand_handles;
        #endif
        h_attach = left_attach;
    break;
    } //switch (arm_code)

        // set joints position or velocity (it depends on the settings)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    #if HAND==1
        // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
        ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
        vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
        srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
        srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
        srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
    #endif

    VectorXd f_posture; // the final posture
    bool f_reached;
    double tol_stop_stage;
    std::vector<double> timesteps_stage;
    MatrixXd traj;
    MatrixXd vel;
    double timeTot = 0.0;

    MatrixXd traj_plan_approach;
    MatrixXd vel_plan_approach;
    std::vector<double> timesteps_plan_approach;
    bool join_plan_approach = false;

    if(mov_type==0 || mov_type==2 || mov_type==3 || mov_type==4){
        // reach-to-grasp, transport, engage, disengage
        if(traj_mov.size() > 1){
            // there is more than one stage
            string mov_descr_1 = traj_descr.at(0);
            string mov_descr_2 = traj_descr.at(1);
            if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"approach")==0)){
                join_plan_approach = true;
                traj_plan_approach.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                vel_plan_approach.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());

            } // if-strcmp
        }// if-traj_mov
    }// if - mov_type

    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages

    for (size_t k=0; k< traj_mov.size();++k){
        string mov_descr = traj_descr.at(k);
        if(strcmp(mov_descr.c_str(),"plan")==0){
            plan=true; approach=false; retreat=false;
            if(join_plan_approach){
                MatrixXd tt = traj_mov.at(k);
                MatrixXd vv = vel_mov.at(k);
                std::vector<double> ttsteps = timesteps.at(k);
                traj_plan_approach.topLeftCorner(tt.rows(),tt.cols()) = tt;
                vel_plan_approach.topLeftCorner(vv.rows(),vv.cols()) = vv;
                timesteps_plan_approach.reserve(ttsteps.size());
                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
                continue;
            }// if - join_plan_approach
        } // if - strcmp
        else if(strcmp(mov_descr.c_str(),"approach")==0){
            plan=false; approach=true; retreat=false;
            if(join_plan_approach){
                MatrixXd tt = traj_mov.at(k);
                MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                MatrixXd vv = vel_mov.at(k);
                MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                std::vector<double> ttsteps = timesteps.at(k);
                traj_plan_approach.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                vel_plan_approach.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                timesteps_plan_approach.reserve(ttsteps.size());
                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
            }// if - join_plan_approach
        } // if - strcmp
        else if(strcmp(mov_descr.c_str(),"retreat")==0){
            plan=false; approach=false; retreat=true;
        }// if - strcmp

        switch (mov_type){
        case 0: // reach-to-grasp
            if(retreat){
                if(arm_code!=0){
                    //single-arm
                    #if HAND == 0 || HAND == 1
                    if(obj_in_hand){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        } // if - srvset_parent
                        #if HAND == 1 && OPEN_CLOSE_HAND ==1
                            this->closeBarrettHand(arm_code);
                        #elif HAND == 1
                            MatrixXd tt = traj_mov.at(k);
                            VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                            std::vector<double> hand_init_pos;
                            hand_init_pos.resize(init_h_posture.size());
                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                            this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
                        #endif
                    } // if - obj_in_hand
                    #else
                    if((this->hand_code_right == 2 && arm_code == 1) || (this->hand_code_left == 2 && arm_code == 2)){
                        //if(obj_in_hand){
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                            srvset_parent.request.parentHandle = h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1){
                                log(QNode::Error,string("Error in grasping the object "));
                            } // if - srvset_parent
                            closed = true;                       // } // if - obj_in_hand
                    }else if((this->hand_code_right == 3 && arm_code == 1) || (this->hand_code_left == 3 && arm_code == 2)){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in grasping the object "));
                        } // if - srvset_parent
                        closed = true;
                    }
                #endif
                } // if - arm_code
                else{
                    // dual-arm
                    #if HAND != 0 && HAND != 1
                      closed = true;
                    #endif
                    #if HAND == 0 || HAND == 1
                      if(obj_in_r_hand){ // right arm
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                          srvset_parent.request.parentHandle = r_h_attach;
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in grasping the object "));
                          } // if - srvset_parent
                          #if HAND == 1 && OPEN_CLOSE_HAND ==1
                              this->closeBarrettHand(1);
                          #elif HAND == 1
                              MatrixXd tt = traj_mov.at(k);
                              VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                              std::vector<double> hand_init_pos;
                              hand_init_pos.resize(init_h_posture.size());
                              VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                              this->closeBarrettHand_to_pos(1,hand_init_pos);
                          #endif
                      } // if - obj_in_r_hand*/
                      if(obj_in_l_hand){ // left arm
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          //srvset_parent.request.handle = h_detobj;
                          srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                          srvset_parent.request.parentHandle = l_h_attach;
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in grasping the object "));
                          } // if - srvset_parent
                          #if HAND == 1 && OPEN_CLOSE_HAND ==1
                              this->closeBarrettHand(2);
                          #elif HAND == 1
                              MatrixXd tt = traj_mov.at(k);
                              VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                              std::vector<double> hand_init_pos;
                              hand_init_pos.resize(init_h_posture.size());
                              VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                              this->closeBarrettHand_to_pos(2,hand_init_pos);
                          #endif
                      } // if - obj_in_l_hand
                    #else
                      if(this->hand_code_right == 2){
                          if(obj_in_r_hand){ // right arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = r_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              } // if - srvset_parent
                              closed = true;
                          } // if - obj_in_r_hand*/
                      }else if(this->hand_code_right == 3){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                          srvset_parent.request.parentHandle = r_h_attach;
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in grasping the object "));
                          } // if - srvset_parent
                          closed = true;
                       }

                      /*if(this->hand_code_left == 2){
                          if(obj_in_l_hand){ // left arm
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = l_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              } // if - srvset_parent
                              closed = true;
                          } // if - obj_in_l_hand
                      }else if(this->hand_code_left == 3){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          //srvset_parent.request.handle = h_detobj;
                          srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                          srvset_parent.request.parentHandle = l_h_attach;
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in grasping the object "));
                          } // if - srvset_parent
                          closed = true;
                      }*/
                    #endif
                } // if - else
            }// if - retreat
             break;
        case 1: // reaching
            if(arm_code!=0){
                // single-arm
                if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                    else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                    srvset_parent.request.parentHandle = -1; // parentless object
                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                    add_client.call(srvset_parent);
                    if (srvset_parent.response.result != 1){
                        log(QNode::Error,string("Error in releasing the object "));
                    }// if - srvset_parent
                }// if - strcmp
                closed = false;
            }// if - arm_code
            else{
                // dual-arm
                // right arm
                if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                    srvset_parent.request.parentHandle = -1; // parentless object
                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                    add_client.call(srvset_parent);
                    if (srvset_parent.response.result != 1){
                        log(QNode::Error,string("Error in releasing the object "));
                    }// if - srvset_parent
                }// if - strcmp
                closed = false;

                // left arm
                if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                    srvset_parent.request.parentHandle = -1; // parentless object
                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                    add_client.call(srvset_parent);
                    if (srvset_parent.response.result != 1){
                        log(QNode::Error,string("Error in releasing the object "));
                    }// if - srvset_parent
                }// if - strcmp
            }//if - else
            break;
        case 2: case 3:// transport, engage
            if(retreat){
                if(arm_code!=0){
                    // single-arm
                    if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = -1; // parentless object
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in releasing the object "));
                        }// if - srvset_parent
                    }// if - strcmp
                    #if HAND ==1 && OPEN_CLOSE_HAND ==1
                        MatrixXd tt = traj_mov.at(k);
                        VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(arm_code,hand_init_pos);
                    #elif HAND == 1
                        closed.at(0)=false;
                        closed.at(1)=false;
                        closed.at(2)=false;
                    #else
                        closed = false;
                    #endif
                }// if - arm_code
                else{
                    // dual-arm
                    // right arm
                    if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = -1; // parentless object
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in releasing the object "));
                        }// if - srvset_parent
                    }// if - strcmp
                    #if HAND ==1 && OPEN_CLOSE_HAND ==1
                        MatrixXd tt = traj_mov.at(k);
                        VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(1,hand_init_pos);
                    #elif HAND == 1
                        closed.at(0)=false;
                        closed.at(1)=false;
                        closed.at(2)=false;
                    #else
                        closed = false;
                    #endif

                    // left arm
                    if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                        srvset_parent.request.parentHandle = -1; // parentless object
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);
                        if (srvset_parent.response.result != 1){
                            log(QNode::Error,string("Error in releasing the object "));
                        }// if - srvset_parent
                    }// if - strcmp
                }//if - else
            }//if - retreat
            break;
        case 4:// disengage
            break;
        case 5: // go-park
            break;
        }//if - switch

        if(join_plan_approach && (strcmp(mov_descr.c_str(),"approach")==0)){
            traj = traj_plan_approach;
            vel = vel_plan_approach;
            timesteps_stage = timesteps_plan_approach;
        }else{
            traj = traj_mov.at(k);
            vel = vel_mov.at(k);
            timesteps_stage = timesteps.at(k);
        }

        tol_stop_stage = tols_stop.at(k);
        f_posture = traj.row(traj.rows()-1);
        f_reached=false;

        #if HAND==0
            if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1))
        #elif HAND==1
            if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
            client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1))
        #else
            if ((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
        #endif
            {
                #if HAND == 1
                    ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
                #endif

                // 5. Let's prepare a publisher of those values:
                ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);

                ros::spinOnce(); // handle ROS messages
                pre_time = simulationTime - timeTot; // update the total time of the movement

                tb = pre_time;
                for (int i = 0; i< vel.rows()-1; ++i){
                    VectorXd ya = vel.row(i);
                    VectorXd yb = vel.row(i+1);
                    VectorXd yat = traj.row(i);
                    VectorXd ybt = traj.row(i+1);
                    ta = tb;
                    double tt_step = timesteps_stage.at(i);
                    if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
                    tb = ta + tt_step;
                    bool interval = true;
                    double tx_prev;
                    double yxt_prev;

                    while (ros::ok() && simulationRunning && interval)
                    {// ros is running, simulation is running

                        vrep_common::JointSetStateData dataTraj;
                        #if HAND==1
                            vrep_common::JointSetStateData data_hand;
                        #endif

                        tx = simulationTime - timeTot;

                        if (tx > tb){
                            // go to the next interval
                            interval = false;
                        }// if - tx
                        else{
                            double m;
                            if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}

                            std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                            switch (arm_code) {
                            case 0: // dual arm
                                this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                                break;
                            case 1: // right arm
                                this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                                break;
                            case 2: //left arm
                                this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                                break;
                            } //  if - switch

                            double yx; double yxt; double thr = 0.0;
                            if(arm_code!=0){
                                thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                        pow((f_posture(1)-curr_post.at(1)),2)+
                                        pow((f_posture(2)-curr_post.at(2)),2)+
                                        pow((f_posture(3)-curr_post.at(3)),2)+
                                        pow((f_posture(4)-curr_post.at(4)),2)+
                                        pow((f_posture(5)-curr_post.at(5)),2)+
                                        pow((f_posture(6)-curr_post.at(6)),2));
                            }// if - arm_code
                            else{
                             #if HAND == 1 || HAND == 0
                                thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                        pow((f_posture(1)-r_post.at(1)),2)+
                                        pow((f_posture(2)-r_post.at(2)),2)+
                                        pow((f_posture(3)-r_post.at(3)),2)+
                                        pow((f_posture(4)-r_post.at(4)),2)+
                                        pow((f_posture(5)-r_post.at(5)),2)+
                                        pow((f_posture(6)-r_post.at(6)),2)+
                                        pow((f_posture(11)-l_post.at(0)),2)+
                                        pow((f_posture(12)-l_post.at(1)),2)+
                                        pow((f_posture(13)-l_post.at(2)),2)+
                                        pow((f_posture(14)-l_post.at(3)),2)+
                                        pow((f_posture(15)-l_post.at(4)),2)+
                                        pow((f_posture(16)-l_post.at(5)),2)+
                                        pow((f_posture(17)-l_post.at(6)),2));
                             #else
                                if(this->hand_code_right == 2){
                                    thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                            pow((f_posture(1)-r_post.at(1)),2)+
                                            pow((f_posture(2)-r_post.at(2)),2)+
                                            pow((f_posture(3)-r_post.at(3)),2)+
                                            pow((f_posture(4)-r_post.at(4)),2)+
                                            pow((f_posture(5)-r_post.at(5)),2)+
                                            pow((f_posture(6)-r_post.at(6)),2)+
                                            pow((f_posture(8)-l_post.at(0)),2)+
                                            pow((f_posture(9)-l_post.at(1)),2)+
                                            pow((f_posture(10)-l_post.at(2)),2)+
                                            pow((f_posture(11)-l_post.at(3)),2)+
                                            pow((f_posture(12)-l_post.at(4)),2)+
                                            pow((f_posture(13)-l_post.at(5)),2)+
                                            pow((f_posture(14)-l_post.at(6)),2));
                                }else if(this->hand_code_right == 3){
                                    thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                            pow((f_posture(1)-r_post.at(1)),2)+
                                            pow((f_posture(2)-r_post.at(2)),2)+
                                            pow((f_posture(3)-r_post.at(3)),2)+
                                            pow((f_posture(4)-r_post.at(4)),2)+
                                            pow((f_posture(5)-r_post.at(5)),2)+
                                            pow((f_posture(6)-r_post.at(6)),2)+
                                            pow((f_posture(9)-l_post.at(0)),2)+
                                            pow((f_posture(10)-l_post.at(1)),2)+
                                            pow((f_posture(11)-l_post.at(2)),2)+
                                            pow((f_posture(12)-l_post.at(3)),2)+
                                            pow((f_posture(13)-l_post.at(4)),2)+
                                            pow((f_posture(14)-l_post.at(5)),2)+
                                            pow((f_posture(15)-l_post.at(6)),2));
                                }
                             #endif
                            }// if - else


                            if(thr < tol_stop_stage){
                                f_reached=true;
                                break;
                            }// if - thr
                            else{f_reached=false;}

                            #if HAND == 0 || HAND == 1
                                hand_closed = (closed[0] && closed[1] && closed[2]);
                            #else
                                hand_closed = closed;
                            #endif

                            for (int k = 0; k < vel.cols(); ++k){// for loop joints
                                if(f_reached){
                                    yx=0;
                                    yxt=yxt_prev;
                                }// if - f_reached
                                else{
                                    yx = interpolate(ya(k),yb(k),m);
                                    yxt = interpolate(yat(k),ybt(k),m);
                                    yxt_prev=yxt;
                                }// else

                                if(arm_code!=0){
                                    // single-arm
                                    #if HAND == 0 || HAND == 1
                                        if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                                (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)) // joints of the hand if the hand is open{
                                            dataTraj.handles.data.push_back(handles.at(k));
                                        }
                                    #else
                                        if((this->hand_code_right == 2 && arm_code == 1) || (this->hand_code_left == 2 && arm_code == 2)){
                                            if((k!=vel.cols()-1) || ((k==vel.cols()-1)  && !hand_closed)){
                                                dataTraj.handles.data.push_back(handles.at(k));
                                            }
                                        }else if((this->hand_code_right == 3 && arm_code == 1) || (this->hand_code_left == 3 && arm_code == 2)){
                                            dataTraj.handles.data.push_back(handles.at(k));
                                        }
                                    #endif

                                }// if - arm_code
                                else{
                                    // dual-arm
                                    #if HAND == 0 || HAND == 1
                                        if((k < JOINTS_ARM) || // joints of the right arm OR
                                            ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(r_handles.at(k));
                                        }// if - k
                                        else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                                ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                                        {
                                            dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                                        }// else
                                    #else
                                        int total_joints_right;
                                        if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                        else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                        if((k < JOINTS_ARM) || // joints of the right arm OR
                                            (((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right) && !hand_closed) && this->hand_code_right == 2)) // joints of the right hand if the hand is open
                                            dataTraj.handles.data.push_back(r_handles.at(k));
                                        else if((this->hand_code_right == 3) && ((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)))
                                            dataTraj.handles.data.push_back(r_handles.at(k));
                                        else if (((k >= JOINTS_ARM + total_joints_right) && ( k < (JOINTS_ARM*2) + total_joints_right) )|| // joints of the left arm OR
                                                (((k >= (JOINTS_ARM*2) + total_joints_right) && !hand_closed) && this->hand_code_left == 2)) // joints of the left hand if the hand is open
                                            dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                        else if((this->hand_code_left == 3) && (k >= (JOINTS_ARM*2) + total_joints_right))
                                            dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                    #endif
                                }// else


                                int exec_mode; double exec_value;

                                if(vel_mode){
                                    //velocity
                                    exec_mode = 2;
                                    exec_value = yx;
                                }// if - vel_mode
                                else{
                                    // position
                                    exec_mode = 0;
                                    exec_value = yxt;
                                }// else


                                if(arm_code!=0){
                                    // single-arm
                                    //ARoS
                                    #if HAND == 0 || HAND == 1
                                        if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed) // joints of the hand
                                        {
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))) // joints of the arm
                                        {
                                            //dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(exec_value);
                                        }

                                    #else
                                        if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                            if((k==vel.cols()-1) && !hand_closed){
                                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                dataTraj.values.data.push_back(yxt);
                                            }else if(k !=vel.cols()-1){
                                                //dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                dataTraj.values.data.push_back(exec_value);
                                            }
                                        }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                dataTraj.values.data.push_back(exec_value);
                                        }
                                    #endif
                                }else{
                                    //dual-arm
                                    #if HAND == 0 || HAND == 1
                                        if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                                        {
                                            //dataTraj.setModes.data.push_back(1);
                                            dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(exec_value);
                                        }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                                (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                                        {
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }
                                    #else
                                        int total_joints_right;
                                        if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                        else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                        if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + total_joints_right) && ( k < total_joints_right + (JOINTS_ARM*2))) )// joints of the right or left arm
                                        {
                                            //dataTraj.setModes.data.push_back(1);
                                            dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(exec_value);
                                        }else if (((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && (this->hand_code_right == 3)){
                                            dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }else if((((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && !hand_closed) && (this->hand_code_right == 2)){
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }else if((k >= total_joints_right + (JOINTS_ARM*2)) && (this->hand_code_left == 3)){
                                            dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }else if(((k >= total_joints_right + (JOINTS_ARM*2)) && !hand_closed) && (this->hand_code_left == 2)){
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                            dataTraj.values.data.push_back(yxt);
                                        }

                                    #endif
                                }

                                #if HAND ==1
                                    if(arm_code!=0){
                                        //single-arm
                                        if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                            // the fingers are being addressed
                                            data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                            data_hand.setModes.data.push_back(1); // set the target position
                                            data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                        }
                                    }else{
                                        // dual-arm
                                        if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                            // the right fingers are being addressed
                                            data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                            data_hand.setModes.data.push_back(1); // set the target position
                                            data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                        }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                            // the left fingers are being addressed
                                            data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                            data_hand.setModes.data.push_back(1); // set the target position
                                            data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                        }
                                    }
                                #endif
                            } // for loop joints
                            pub.publish(dataTraj);
                            #if HAND ==1
                                pub_hand.publish(data_hand);
                            #endif

                            interval = true;
                            tx_prev = tx;
                        } // if tx is inside the interval
                        // handle ROS messages:
                        ros::spinOnce();
                    }// while
                    if(f_reached){
                        log(QNode::Info,string("Final Posture reached."));
                        break;
                    }
                }// for loop steps
                // ----- post-movement operations -------- //
                switch (mov_type) {
                case 0: // reach-to grasp
                    // grasp the object
                    if(approach ||(plan && (traj_mov.size()==1))){
                        if(arm_code!=0){
                            //single arm
                            #if HAND == 0 || HAND == 1
                                if(obj_in_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                            #else
                                if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                    //if(obj_in_hand){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    //}
                                }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                    srvset_parent.request.parentHandle = h_attach;
                                    srvset_parent.request.keepInPlace = true; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                            #endif
                        }
                        else{
                            // dual arm
                            #if HAND == 0 || HAND == 1
                                if(obj_in_r_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = r_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }

                                if(obj_in_l_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                    srvset_parent.request.parentHandle = l_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                        #else
                            if(this->hand_code_right == 2){
                                if(obj_in_r_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = r_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                            }else if(this->hand_code_right == 3){
                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                srvset_parent.request.parentHandle = r_h_attach;
                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                add_client.call(srvset_parent);
                                if (srvset_parent.response.result != 1){
                                    log(QNode::Error,string("Error in grasping the object "));
                                }
                            }

                            if(this->hand_code_left == 2){
                                if(obj_in_l_hand){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                    srvset_parent.request.parentHandle = l_h_attach;
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in grasping the object "));
                                    }
                                }
                            }else if(this->hand_code_left == 3){
                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                srvset_parent.request.parentHandle = l_h_attach;
                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                add_client.call(srvset_parent);
                                if (srvset_parent.response.result != 1){
                                    log(QNode::Error,string("Error in grasping the object "));
                                }
                            }
                        #endif
                        }
                    }
                    break;
                case 1: // reaching
                    break;
                case 2: // transport
                    break;
                case 3: // engage
                    break;
                case 4: // disengage
                    break;
                case 5: // go park
                    break;
                }
            }// if -subscribe
            // handle ROS messages:
            ros::spinOnce();
            timeTot = simulationTime; // update the total time of the movement
    } // for loop stages

    // pause the simulation
    this->pauseSim();

    // handle ROS messages:
    ros::spinOnce();

    TotalTime = simulationTime; // update the total time of the movement

    log(QNode::Info,string("Movement completed"));
    mov->setExecuted(true);


    return true;
}

bool QNode::execTask(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task,
                     vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task,taskPtr task, scenarioPtr scene,bool vel_mode)
{
    bool hand_closed;
    #if HAND == 0 || HAND == 1
      closed.at(0)=false;
      closed.at(1)=false;
      closed.at(2)=false;
    #else
      closed = false;
    #endif
    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    int arm_code;
    int mov_type;
    double timeTot = 0.0;
    this->curr_scene = scene;

    std::vector<int> handles;
    std::vector<int> r_handles;
    std::vector<int> l_handles;

    int h_attach; // handle of the attachment point of the hand
    int r_h_attach; // handle of the attachment point of the hand
    int l_h_attach; // handle of the attachment point of the hand

    #if HAND == 0 || HAND == 1
      MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
      MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
      MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    #endif

    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan; bool approach; bool retreat;
    double pre_time;

    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    #if HAND==1

      // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
      ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
      vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
      srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
      srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
      srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    #endif


    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task
    for(int kk=0; kk < task->getProblemNumber(); ++kk){ //for loop movements
      if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved()){
          int ii = kk - hh;
          vector<MatrixXd> traj_mov = traj_task.at(ii);
          vector<MatrixXd> vel_mov = vel_task.at(ii);
          vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
          vector<double> tols_stop_mov = tols_stop_task.at(ii);
          double tol_stop_stage;
          MatrixXd traj;
          MatrixXd vel;
          std::vector<double> timesteps_stage;
          movementPtr mov = task->getProblem(kk)->getMovement();
          vector<string> traj_descr_mov = traj_descr_task.at(ii);
          this->curr_mov = mov;
          arm_code = mov->getArm();
          mov_type = mov->getType();

          switch (mov_type){
          case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
              #if HAND == 0 || HAND == 1
                closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
              #else
                closed = false;
              #endif
              break;
          case 2: case 3: case 4: // transport, engage, disengage
              #if HAND == 0 || HAND == 1
                closed.at(0)=true; closed.at(1)=true; closed.at(2)=true;
              #else
                closed = true;
              #endif
              break;
          }
          switch (arm_code) {
          case 0: // dual arm
              r_handles = right_handles;
              r_h_attach = right_attach;
              l_handles = left_handles;
              l_h_attach = left_attach;
              #if HAND == 0 || HAND == 1
                l_hand_handles = left_hand_handles;
                r_hand_handles = right_hand_handles;
              #endif
              break;
          case 1: //right arm
              handles = right_handles;
              h_attach = right_attach;
              #if HAND == 0 || HAND == 1
                hand_handles = right_hand_handles;
              #endif
              break;
          case 2: // left arm
              handles = left_handles;
              h_attach = left_attach;
              #if HAND == 0 || HAND == 1
                hand_handles = left_hand_handles;
              #endif
              break;
          }

          for(size_t j=0; j < traj_mov.size();++j){ //for loop stages
              string mov_descr = traj_descr_mov.at(j);
              if(strcmp(mov_descr.c_str(),"plan")==0){
                  plan=true; approach=false; retreat=false;
              }else if(strcmp(mov_descr.c_str(),"approach")==0){
                  plan=false; approach=true; retreat=false;
              }else if(strcmp(mov_descr.c_str(),"retreat")==0){
                  plan=false; approach=false; retreat=true;
              }

              switch (mov_type){
              case 0: // reach-to-grasp
                  if(retreat){
                      if(arm_code!=0){
                          //single-arm
                          #if HAND == 0 || HAND == 1
                              if(obj_in_hand){
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }
                              #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                this->closeBarrettHand(arm_code);
                              #elif HAND == 0 || HAND == 1
                                MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                                std::vector<double> hand_init_pos;
                                hand_init_pos.resize(init_h_posture.size());
                                VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
                              #endif
                              }
                          #else
                              if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                  //if(obj_in_hand){
                                      add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                      vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                      if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                      else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                      srvset_parent.request.parentHandle = h_attach;
                                      srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                      add_client.call(srvset_parent);
                                      if (srvset_parent.response.result != 1){
                                          log(QNode::Error,string("Error in grasping the object "));
                                      }
                                      closed = true;
                                  //}
                              }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                  srvset_parent.request.parentHandle = h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }closed = true;
                              }
                          #endif
                      }else{
                          // dual-arm
                          #if HAND == 0 || HAND == 1
                              if(obj_in_r_hand){ // right arm
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  //srvset_parent.request.handle = h_detobj;
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = r_h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }
                              #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                this->closeBarrettHand(1);
                              #elif HAND == 0 || HAND == 1
                                MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                                std::vector<double> hand_init_pos;
                                hand_init_pos.resize(init_h_posture.size());
                                VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                this->closeBarrettHand_to_pos(1,hand_init_pos);
                              #endif
                              }

                              if(obj_in_l_hand){ // left arm
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  //srvset_parent.request.handle = h_detobj;
                                  srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                  srvset_parent.request.parentHandle = l_h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }
                              #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                this->closeBarrettHand(2);
                              #elif HAND == 0 || HAND == 1
                                MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                                std::vector<double> hand_init_pos;
                                hand_init_pos.resize(init_h_posture.size());
                                VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                this->closeBarrettHand_to_pos(2,hand_init_pos);
                              #endif
                          }
                        #else
                          if(this->hand_code_right == 2){
                              if(obj_in_r_hand){ // right arm
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  //srvset_parent.request.handle = h_detobj;
                                  srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                  srvset_parent.request.parentHandle = r_h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }
                                  closed = true;
                              }
                          }else if(this->hand_code_right == 3){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = r_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
                              closed = true;
                          }

                          if(this->hand_code_left == 2){
                              if(obj_in_l_hand){ // left arm
                                  add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                  vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                  //srvset_parent.request.handle = h_detobj;
                                  srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                  srvset_parent.request.parentHandle = l_h_attach;
                                  srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                  add_client.call(srvset_parent);
                                  if (srvset_parent.response.result != 1){
                                      log(QNode::Error,string("Error in grasping the object "));
                                  }
                                  closed = true;
                              }
                          }else if(this->hand_code_left == 3){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              //srvset_parent.request.handle = h_detobj;
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = l_h_attach;
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in grasping the object "));
                              }
                              closed = true;
                          }
                        #endif
                      }
                  }
                  break;
              case 1: // reaching
                  if(arm_code!=0){
                      // single-arm
                      if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                          srvset_parent.request.parentHandle = -1; // parentless object
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in releasing the object "));
                          }// if - srvset_parent
                      }// if - strcmp
                      closed = false;
                  }// if - arm_code
                  else{
                      // dual-arm
                      // right arm
                      if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                          srvset_parent.request.parentHandle = -1; // parentless object
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in releasing the object "));
                          }// if - srvset_parent
                      }// if - strcmp
                      closed = false;

                      // left arm
                      if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                          add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                          vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                          srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                          srvset_parent.request.parentHandle = -1; // parentless object
                          srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                          add_client.call(srvset_parent);
                          if (srvset_parent.response.result != 1){
                              log(QNode::Error,string("Error in releasing the object "));
                          }// if - srvset_parent
                      }// if - strcmp
                  }//if - else
                  break;
              case 2: case 3: // transport, engage
                  if(retreat){
                      if(arm_code!=0){
                          // single-arm
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = -1; // parentless object
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in releasing the object "));
                              }
                          }
                      #if HAND ==1 && OPEN_CLOSE_HAND ==1
                        MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(arm_code,hand_init_pos);
                      #elif HAND == 0 || HAND == 1
                        closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
                      #else
                          closed = false;
                      #endif
                      }else{
                          //dual-arm
                          // right arm
                          if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                              srvset_parent.request.parentHandle = -1; // parentless object
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in releasing the object "));
                              }
                          }
                          #if HAND ==1 && OPEN_CLOSE_HAND ==1
                            MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                            std::vector<double> hand_init_pos;
                            hand_init_pos.resize(init_h_posture.size());
                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                            this->openBarrettHand_to_pos(1,hand_init_pos);
                          #elif HAND == 0 || HAND == 1
                            closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
                          #else
                            closed = false;
                          #endif
                          // left arm
                          if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                              add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                              vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                              srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                              srvset_parent.request.parentHandle = -1; // parentless object
                              srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                              add_client.call(srvset_parent);
                              if (srvset_parent.response.result != 1){
                                  log(QNode::Error,string("Error in releasing the object "));
                              }
                          }
                          #if HAND ==1 && OPEN_CLOSE_HAND ==1
                            MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                            std::vector<double> hand_init_pos;
                            hand_init_pos.resize(init_h_posture.size());
                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                            this->openBarrettHand_to_pos(2,hand_init_pos);
                          #elif HAND == 0 || HAND == 1
                            closed.at(0)=false; closed.at(1)=false; closed.at(2)=false;
                          #else
                            closed = false;
                          #endif
                     }
                  }
                  break;
              case 4:// disengage
                  break;
              case 5: // go-park
                  break;
              }
              traj = traj_mov.at(j);
              vel = vel_mov.at(j);
              timesteps_stage = timesteps_mov.at(j);
              f_posture = traj.row(traj.rows()-1);
              f_reached=false;
              tol_stop_stage = tols_stop_mov.at(j);

              ros::spinOnce(); // handle ROS messages
              pre_time = simulationTime - timeTot; // update the total time of the movement

              #if HAND==0
                if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1))

              #elif HAND==1
                if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                     client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1))
              #else
                if(client_enableSubscriber.call(srv_enableSubscriber) && (srv_enableSubscriber.response.subscriberID!=-1))
              #endif
                {
                    #if HAND == 1
                      ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
                    #endif

                      // 5. Let's prepare a publisher of those values:
                    ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                    tb = pre_time;
                    for (int i = 0; i< vel.rows()-1; ++i){

                        VectorXd ya = vel.row(i);
                        VectorXd yb = vel.row(i+1);
                        VectorXd yat = traj.row(i);
                        VectorXd ybt = traj.row(i+1);

                        ta = tb;
                        double tt_step = timesteps_stage.at(i);
                        if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
                        tb = ta + tt_step;

                        bool interval = true;
                        double tx_prev;
                        double yxt_prev;
                        ros::spinOnce(); // get the simulationRunning value
                        while (ros::ok() && simulationRunning && interval)
                        {// ros is running, simulation is running
                            vrep_common::JointSetStateData dataTraj;
                            #if HAND==1
                              vrep_common::JointSetStateData data_hand;
                            #endif
                            tx = simulationTime - timeTot;

                            if (tx >= tb){
                                // go to the next interval
                                interval = false;
                            }else{
                                double m;
                                if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}
                                std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                                switch (arm_code) {
                                case 0: // dual arm
                                    this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                    this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                                    break;
                                case 1: // right arm
                                    this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                                    break;
                                case 2: //left arm
                                    this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                                    break;
                                }
                                double yx; double yxt; double thr = 0.0;
                                if(arm_code!=0){
                                    thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                               pow((f_posture(1)-curr_post.at(1)),2)+
                                               pow((f_posture(2)-curr_post.at(2)),2)+
                                               pow((f_posture(3)-curr_post.at(3)),2)+
                                               pow((f_posture(4)-curr_post.at(4)),2)+
                                               pow((f_posture(5)-curr_post.at(5)),2)+
                                               pow((f_posture(6)-curr_post.at(6)),2));
                                }else{
                                    #if HAND == 1 || HAND == 0
                                       thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                               pow((f_posture(1)-r_post.at(1)),2)+
                                               pow((f_posture(2)-r_post.at(2)),2)+
                                               pow((f_posture(3)-r_post.at(3)),2)+
                                               pow((f_posture(4)-r_post.at(4)),2)+
                                               pow((f_posture(5)-r_post.at(5)),2)+
                                               pow((f_posture(6)-r_post.at(6)),2)+
                                               pow((f_posture(11)-l_post.at(0)),2)+
                                               pow((f_posture(12)-l_post.at(1)),2)+
                                               pow((f_posture(13)-l_post.at(2)),2)+
                                               pow((f_posture(14)-l_post.at(3)),2)+
                                               pow((f_posture(15)-l_post.at(4)),2)+
                                               pow((f_posture(16)-l_post.at(5)),2)+
                                               pow((f_posture(17)-l_post.at(6)),2));
                                    #else
                                       if(this->hand_code_right == 2){
                                           thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                                   pow((f_posture(1)-r_post.at(1)),2)+
                                                   pow((f_posture(2)-r_post.at(2)),2)+
                                                   pow((f_posture(3)-r_post.at(3)),2)+
                                                   pow((f_posture(4)-r_post.at(4)),2)+
                                                   pow((f_posture(5)-r_post.at(5)),2)+
                                                   pow((f_posture(6)-r_post.at(6)),2)+
                                                   pow((f_posture(8)-l_post.at(0)),2)+
                                                   pow((f_posture(9)-l_post.at(1)),2)+
                                                   pow((f_posture(10)-l_post.at(2)),2)+
                                                   pow((f_posture(11)-l_post.at(3)),2)+
                                                   pow((f_posture(12)-l_post.at(4)),2)+
                                                   pow((f_posture(13)-l_post.at(5)),2)+
                                                   pow((f_posture(14)-l_post.at(6)),2));
                                       }else if(this->hand_code_right == 3){
                                           thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                                   pow((f_posture(1)-r_post.at(1)),2)+
                                                   pow((f_posture(2)-r_post.at(2)),2)+
                                                   pow((f_posture(3)-r_post.at(3)),2)+
                                                   pow((f_posture(4)-r_post.at(4)),2)+
                                                   pow((f_posture(5)-r_post.at(5)),2)+
                                                   pow((f_posture(6)-r_post.at(6)),2)+
                                                   pow((f_posture(9)-l_post.at(0)),2)+
                                                   pow((f_posture(10)-l_post.at(1)),2)+
                                                   pow((f_posture(11)-l_post.at(2)),2)+
                                                   pow((f_posture(12)-l_post.at(3)),2)+
                                                   pow((f_posture(13)-l_post.at(4)),2)+
                                                   pow((f_posture(14)-l_post.at(5)),2)+
                                                   pow((f_posture(15)-l_post.at(6)),2));
                                       }
                                    #endif
                                }
                                if(thr < tol_stop_stage){
                                    f_reached=true;
                                    log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                    //std::cout << "final posture reached, movement: " << mov->getStrType() << std::endl;
                                    break;
                                }else{f_reached=false;}

                                #if HAND == 0 || HAND == 1
                                  hand_closed = (closed[0] && closed[1] && closed[2]);
                                #else
                                  hand_closed = closed;
                                #endif

                                  for (int k = 0; k< vel.cols(); ++k){ // for loop joints
                                    if(f_reached){
                                        yx=0;
                                        yxt=yxt_prev;
                                    }else{
                                        yx = interpolate(ya(k),yb(k),m);
                                        yxt = interpolate(yat(k),ybt(k),m);
                                        yxt_prev=yxt;
                                    }

                                    #if HAND == 0 || HAND == 1
                                      bool isArmJoint =((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4));
                                      bool isHandJoint = ((k==vel.cols()-1) && (k==vel.cols()-2) && (k==vel.cols()-3) && (k==vel.cols()-4));
                                    #else
                                      bool isArmJoint;
                                      bool isHandJoint;
                                      bool useEG = false;
                                      switch (arm_code) {
                                      case 0:
                                          break;
                                      case 1:
                                          if(this->hand_code_right == 2){
                                            isArmJoint = (k!=vel.cols()-1);
                                            isHandJoint = (k==vel.cols()-1);
                                            useEG = true;
                                          }else if(this->hand_code_right == 3){
                                            isArmJoint = (k!=vel.cols()-1 && k != vel.cols()-2);
                                            isHandJoint = (k==vel.cols()-1 || k == vel.cols()-2);
                                            useEG = false;
                                          }
                                          break;
                                      case 2:
                                          if(this->hand_code_left == 2){
                                            isArmJoint = (k!=vel.cols()-1);
                                            isHandJoint = (k==vel.cols()-1);
                                            useEG = true;
                                          }else if(this->hand_code_left == 3){
                                            isArmJoint = (k!=vel.cols()-1 && k != vel.cols()-2);
                                            isHandJoint = (k==vel.cols()-1 || k == vel.cols()-2);
                                            useEG = false;
                                          }
                                          break;
                                      }
                                    #endif
                                    if(arm_code!=0){
                                        // single-arm
                                        if(isArmJoint || (isHandJoint && !useEG) || (isHandJoint && !hand_closed && useEG)){
                                            dataTraj.handles.data.push_back(handles.at(k));
                                        }
                                    }else{
                                        // dual-arm
                                        #if HAND == 0 || HAND == 1
                                            if((k < JOINTS_ARM) || // joints of the right arm OR
                                                ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                                            {
                                                dataTraj.handles.data.push_back(r_handles.at(k));
                                            }else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                                      ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                                            {
                                                dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                                            }
                                        #else
                                            int total_joints_right;
                                            if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                            else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                            if((k < JOINTS_ARM) || // joints of the right arm OR
                                                (((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right) && !hand_closed) && this->hand_code_right == 2)) // joints of the right hand if the hand is open
                                                dataTraj.handles.data.push_back(r_handles.at(k));
                                            else if((this->hand_code_right == 3) && ((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)))
                                                dataTraj.handles.data.push_back(r_handles.at(k));
                                            else if (((k >= JOINTS_ARM + total_joints_right) && ( k < (JOINTS_ARM*2) + total_joints_right) )|| // joints of the left arm OR
                                                    (((k >= (JOINTS_ARM*2) + total_joints_right) && !hand_closed) && this->hand_code_left == 2)) // joints of the left hand if the hand is open
                                                dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                            else if((this->hand_code_left == 3) && (k >= (JOINTS_ARM*2) + total_joints_right))
                                                dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                        #endif
                                    }
                                    int exec_mode; double exec_value;

                        if(vel_mode){
                            //velocity
                            exec_mode = 2; exec_value = yx;
                        }else{
                            // position
                            exec_mode = 0; exec_value = yxt;
                        }

                        if(arm_code!=0){
                            // single-arm
                            //ARoS
                            if(isHandJoint && !hand_closed && useEG) // joints of the hand
                            {
                                dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }else if(isArmJoint) // joints of the arm
                            {
                                dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(exec_value);
                            }else if(isHandJoint && !useEG) // joints of the hand
                            {
                                dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                dataTraj.values.data.push_back(yxt);
                            }
                        }else{
                            //dual-arm
                            #if HAND == 0 || HAND == 1
                                if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                                {
                                    dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(exec_value);
                                }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                          (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                                {
                                    dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(yxt);
                                }
                            #else
                                int total_joints_right;
                                if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + total_joints_right) && ( k < total_joints_right + (JOINTS_ARM*2))) )// joints of the right or left arm
                                {
                                    dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(exec_value);
                                }else if(((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && !hand_closed && (this->hand_code_right == 2)){
                                    dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(yxt);
                                }else if(((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && (this->hand_code_right == 3)){
                                    dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(yxt);
                                }else if((k >= (JOINTS_ARM*2) + total_joints_right) && !hand_closed && (this->hand_code_left == 2)){
                                    dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(yxt);
                                }else if((k >= (JOINTS_ARM*2) + total_joints_right) && (this->hand_code_left == 3)){
                                    dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                    dataTraj.values.data.push_back(yxt);
                                }
                            #endif
                        }

                      #if HAND==1

                        if(arm_code!=0){
                            //single-arm
                            if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the fingers are being addressed
                                data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }else{
                            // dual-arm
                            if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the right fingers are being addressed
                                data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                // the left fingers are being addressed
                                data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                data_hand.setModes.data.push_back(1); // set the target position
                                data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                            }
                        }
                      #endif

                    } // FOR LOOP JOINTS
                    //BOOST_LOG_SEV(lg, info) << "\n";

                    pub.publish(dataTraj);

#if HAND==1
                    pub_hand.publish(data_hand);
#endif

                        interval = true;
                        tx_prev = tx;
                    }

                    // handle ROS messages:
                    ros::spinOnce();

                } // while

                if(f_reached){
                    log(QNode::Info,string("Final Posture reached."));
                    break;}

            } // for loop step

                    // ---- post-movement operations ---- //
                    // set the detected object child of the attach point
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                    switch (mov_type) {
                    case 0: // reach-to grasp
                        // grasp the object
                        if(approach ||(plan && (traj_mov.size()==1))){
                            if(arm_code!=0){
                                // single-arm
                                #if HAND == 0 || HAND == 1
                                    if(obj_in_hand){
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }
                                #else
                                    if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                        //if(obj_in_hand){
                                            if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                            else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                            srvset_parent.request.parentHandle = h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                        //}
                                    }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                        if(arm_code == 1) srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        else if(arm_code == 2) srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }
                                #endif
                            }else{
                                // dual-arm
                                #if HAND == 0 || HAND == 1
                                    if(obj_in_r_hand){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = r_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }
                                    if(obj_in_l_hand){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = l_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }
                                #else
                                    if(this->hand_code_right == 2){
                                        if(obj_in_r_hand){
                                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                            srvset_parent.request.parentHandle = r_h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                        }
                                    }else if(this->hand_code_right == 3){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = r_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }

                                    if(this->hand_code_left == 2){
                                        if(obj_in_l_hand){
                                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                            srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                            srvset_parent.request.parentHandle = l_h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                        }
                                    }else if(this->hand_code_left == 3){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = l_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                    }
                                #endif
                            }
                        }
                        break;
                    case 1: // reaching
                        break;
                    case 2: // transport
                        break;
                    case 3: // engage
                        break;
                    case 4: // disengage
                        break;
                    case 5: // go-park
                        break;
                    }

                    // handle ROS messages:
                    ros::spinOnce();
                    timeTot = simulationTime; // update the total time of the movement

                } // if

          }//for loop stages
          // movement complete
          log(QNode::Info,string("Movement completed"));
          task->getProblem(kk)->getMovement()->setExecuted(true);
        }else{
            hh++;
        }// if prob is part of the task

      }// for loop movements
      log(QNode::Info,string("Task completed"));

      // pause the simulation
      this->pauseSim();

      // handle ROS messages:
      ros::spinOnce();

      TotalTime=simulationTime;

      return true;
}

bool QNode::execTask_complete(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task,
                              vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task, taskPtr task, scenarioPtr scene, bool vel_mode)
{
    bool hand_closed;
    #if HAND == 0 || HAND == 1
        closed.at(0)=false;
        closed.at(1)=false;
        closed.at(2)=false;
    #elif HAND == 2
        closed = false;
    #elif HAND == 3
        closed = false;
    #endif

    ros::NodeHandle node;

    double ta;
    double tb = 0.0;
    double tx;
    double timeTot = 0.0;

    int arm_code;
    int mov_type;

    this->curr_scene = scene;

    std::vector<int> handles;
    std::vector<int> r_handles;
    std::vector<int> l_handles;

    int h_attach; // handle of the attachment point of the hand
    int r_h_attach; // handle of the attachment point of the hand
    int l_h_attach; // handle of the attachment point of the hand

    #if HAND == 0 || HAND == 1
        MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
        MatrixXi r_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
        MatrixXi l_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    #endif

    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan;
    bool approach;
    bool retreat;
    double pre_time;

    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    #if HAND==1
        // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
        ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
        vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
        srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
        srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
        srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
    #endif

    // previous retreat trajectories
    MatrixXd traj_prev_retreat;
    MatrixXd vel_prev_retreat;
    std::vector<double> ttsteps_prev_retreat;

    // start the simulation
    this->startSim();
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task
    for(int kk=0; kk < task->getProblemNumber(); ++kk){ //for loop movements
        if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved()){
            int ii = kk - hh;
            vector<MatrixXd> traj_mov = traj_task.at(ii);
            vector<MatrixXd> vel_mov = vel_task.at(ii);
            vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
            vector<double> tols_stop_mov = tols_stop_task.at(ii);
            double tol_stop_stage;
            MatrixXd traj;
            MatrixXd vel;
            std::vector<double> timesteps_stage;
            movementPtr mov = task->getProblem(kk)->getMovement();
            vector<string> traj_descr_mov = traj_descr_task.at(ii);
            this->curr_mov = mov;
            arm_code = mov->getArm();
            mov_type = mov->getType();

            switch (mov_type){
            case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
                #if HAND == 0 || HAND == 1
                    closed.at(0)=false;
                    closed.at(1)=false;
                    closed.at(2)=false;
                #else
                    closed = false;
                #endif
                break;
            case 2: case 3: case 4: // transport, engage, disengage
                #if HAND == 0 || HAND == 1
                    closed.at(0)=true;
                    closed.at(1)=true;
                    closed.at(2)=true;
                #else
                    closed = true;
                #endif
                break;
            }

            switch (arm_code) {
            case 0: // dual arm
                r_handles = right_handles;
                l_handles = left_handles;
                #if HAND == 0 || HAND == 1
                    r_hand_handles = right_hand_handles;
                    l_hand_handles = left_hand_handles;
                #endif
                r_h_attach = right_attach;
                l_h_attach = left_attach;
                break;
            case 1: //right arm
                handles = right_handles;
                #if HAND == 0 || HAND == 1
                    hand_handles = right_hand_handles;
                #endif
                h_attach = right_attach;
                break;
            case 2: // left arm
                handles = left_handles;
                #if HAND == 0 || HAND == 1
                    hand_handles = left_hand_handles;
                #endif
                h_attach = left_attach;
                break;
            }

            MatrixXd traj_ret_plan_app;
            MatrixXd vel_ret_plan_app;
            std::vector<double> timesteps_ret_plan_app;
            bool join_ret_plan_app = false;
            bool join_ret_plan = false;
            bool join_plan_app = false;

            if(mov_type==0 || mov_type==2 || mov_type==3 || mov_type==4){
                // reach-to-grasp, transport, engage, disengage
                if(traj_mov.size() > 1){
                    // there is more than one stage
                    string mov_descr_1 = traj_descr_mov.at(0);
                    string mov_descr_2 = traj_descr_mov.at(1);
                    if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"approach")==0)){
                        if(ii==0 || traj_prev_retreat.rows()==0){
                            // first movement of the task => there is no previous retreat
                            join_plan_app = true;
                            traj_ret_plan_app.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                            vel_ret_plan_app.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                        }
                        else{
                            join_ret_plan_app = true;
                            traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1 + traj_mov.at(1).rows()-1),traj_mov.at(0).cols());
                            vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1 + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                        }
                    }
                    else if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"retreat")==0)){
                        if(ii!=0 && traj_prev_retreat.rows()!=0){
                            join_ret_plan = true;
                            traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                            vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                        }
                    }
                }
                else{
                    // there is only the plan stage
                    if(ii!=0 && traj_prev_retreat.rows()!=0){
                        join_ret_plan = true;
                        traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                        vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                    }
                }
            }
            else{
                // reaching, go-park
                if(ii!=0 && traj_prev_retreat.rows()!=0){
                    join_ret_plan = true;
                    traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov.at(0).rows()-1),traj_mov.at(0).cols());
                    vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                }
            }

                for(size_t j=0; j <traj_mov.size();++j){ //for loop stages
                    string mov_descr = traj_descr_mov.at(j);
                    if(strcmp(mov_descr.c_str(),"plan")==0){
                        plan=true;
                        approach=false;
                        retreat=false;
                        if(join_ret_plan_app){
                            traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                            vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                            timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                            std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                            MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                            MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                            std::vector<double> ttsteps = timesteps_mov.at(j);
                            traj_ret_plan_app.block(traj_prev_retreat.rows(),0,tt_red.rows(),tt_red.cols()) = tt_red;
                            vel_ret_plan_app.block(vel_prev_retreat.rows(),0,vv_red.rows(),vv_red.cols()) = vv_red;
                            timesteps_ret_plan_app.reserve(ttsteps.size());
                            std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                            traj_prev_retreat.resize(0,0);
                            continue;
                        }
                        else if(join_ret_plan){
                            traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                            vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                            timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                            std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                            MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                            MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                            std::vector<double> ttsteps = timesteps_mov.at(j);
                            traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                            vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                            timesteps_ret_plan_app.reserve(ttsteps.size());
                            std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                            traj_prev_retreat.resize(0,0);
                        }
                        else if(join_plan_app){
                            MatrixXd tt = traj_mov.at(j);
                            MatrixXd vv = vel_mov.at(j);
                            std::vector<double> ttsteps = timesteps_mov.at(j);
                            traj_ret_plan_app.topLeftCorner(tt.rows(),tt.cols()) = tt;
                            vel_ret_plan_app.topLeftCorner(vv.rows(),vv.cols()) = vv;
                            timesteps_ret_plan_app.reserve(ttsteps.size());
                            std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                            continue;
                        }
                    }
                    else if(strcmp(mov_descr.c_str(),"approach")==0){
                            plan=false;
                            approach=true;
                            retreat=false;
                            if(join_ret_plan_app || join_plan_app){
                                MatrixXd tt = traj_mov.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                                MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                                std::vector<double> ttsteps = timesteps_mov.at(j);
                                traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                                vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                                timesteps_ret_plan_app.reserve(ttsteps.size());
                                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                            }else if(join_ret_plan){
                                // ERROR
                            }
                    }
                    else if(strcmp(mov_descr.c_str(),"retreat")==0){
                        plan=false; approach=false; retreat=true;
                        traj_prev_retreat = traj_mov.at(j);
                        vel_prev_retreat = vel_mov.at(j);
                        ttsteps_prev_retreat = timesteps_mov.at(j);

                    }

                    switch (mov_type){
                    case 0: // reach-to-grasp
                        if(retreat){
                            if(arm_code!=0){
                                // single-arm
                                #if HAND == 0 || HAND == 1
                                    if(obj_in_hand){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                            this->closeBarrettHand(arm_code);
                                        #elif HAND == 0 || HAND == 1
                                            MatrixXd tt = traj_mov.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                                            std::vector<double> hand_init_pos;
                                            hand_init_pos.resize(init_h_posture.size());
                                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                            this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
                                        #endif
                                    }
                                    continue;
                                #else
                                    if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                        if(obj_in_hand){
                                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                            srvset_parent.request.parentHandle = h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                            closed = true;
                                        }
                                    }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        closed = true;
                                    }
                                    continue;
                                #endif
                            }
                            else{
                                // dual-arm
                                #if HAND == 0 || HAND == 1
                                    if(obj_in_r_hand){ // right arm
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = r_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                            this->closeBarrettHand(1);
                                        #elif HAND == 0 || HAND == 1
                                            MatrixXd tt = traj_mov.at(j);
                                            VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                                            std::vector<double> hand_init_pos;
                                            hand_init_pos.resize(init_h_posture.size());
                                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                            this->closeBarrettHand_to_pos(1,hand_init_pos);
                                        #elif HAND == 2
                                            closed = true;
                                        #elif HAND == 3
                                            closed = true;
                                        #endif
                                    }

                                    if(obj_in_l_hand){ // left arm
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = l_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        #if HAND == 1 && OPEN_CLOSE_HAND ==1
                                            this->closeBarrettHand(2);
                                        #elif HAND == 0 || HAND == 1
                                            MatrixXd tt = traj_mov.at(j);
                                            VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                                            std::vector<double> hand_init_pos;
                                            hand_init_pos.resize(init_h_posture.size());
                                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                            this->closeBarrettHand_to_pos(2,hand_init_pos);
                                        #endif
                                    }
                                    continue;
                                #else
                                    if(this->hand_code_right == 2){
                                        if(obj_in_r_hand){ // right arm
                                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                            srvset_parent.request.parentHandle = r_h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                            closed = true;
                                        }
                                    }else if(this->hand_code_right == 3){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                        srvset_parent.request.parentHandle = r_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        closed = true;
                                    }

                                    if(this->hand_code_left == 2){
                                        if(obj_in_l_hand){ // left arm
                                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                            srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                            srvset_parent.request.parentHandle = l_h_attach;
                                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                            add_client.call(srvset_parent);
                                            if (srvset_parent.response.result != 1){
                                                log(QNode::Error,string("Error in grasping the object "));
                                            }
                                            closed = true;
                                        }
                                    }else if(this->hand_code_left == 3){
                                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                        srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                        srvset_parent.request.parentHandle = l_h_attach;
                                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                        add_client.call(srvset_parent);
                                        if (srvset_parent.response.result != 1){
                                            log(QNode::Error,string("Error in grasping the object "));
                                        }
                                        closed = true;
                                    }
                                    continue;
                                #endif
                            }
                        }
                        break;
                    case 1: // reaching
                        break;
                    case 2: case 3: // transport, engage
                        if(retreat){
                            if(arm_code!=0){
                            // single-arm
                                if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = -1; // parentless object
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in releasing the object "));
                                    }
                                }
                                #if HAND ==1 && OPEN_CLOSE_HAND ==1
                                    MatrixXd tt = traj_mov.at(j);
                                    VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                                    std::vector<double> hand_init_pos;
                                    hand_init_pos.resize(init_h_posture.size());
                                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                    this->openBarrettHand_to_pos(arm_code,hand_init_pos);
                                #elif HAND == 0 || HAND == 1
                                    closed.at(0)=false;
                                    closed.at(1)=false;
                                    closed.at(2)=false;
                                #else
                                    closed = false;
                                #endif
                                continue;
                            }
                            else{
                                //dual-arm
                                // right arm
                                if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                    srvset_parent.request.parentHandle = -1; // parentless object
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in releasing the object "));
                                    }
                                }
                                #if HAND ==1 && OPEN_CLOSE_HAND ==1
                                    MatrixXd tt = traj_mov.at(k); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                                    std::vector<double> hand_init_pos;
                                    hand_init_pos.resize(init_h_posture.size());
                                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                    this->openBarrettHand_to_pos(1,hand_init_pos);
                                #elif HAND == 0 || HAND == 1
                                    closed.at(0)=false;
                                    closed.at(1)=false;
                                    closed.at(2)=false;
                                #else
                                    closed = false;
                                #endif

                                // left arm
                                if(std::strcmp(mov->getObjectLeft()->getName().c_str(),"")!=0){
                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                    srvset_parent.request.parentHandle = -1; // parentless object
                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                    add_client.call(srvset_parent);
                                    if (srvset_parent.response.result != 1){
                                        log(QNode::Error,string("Error in releasing the object "));
                                    }
                                }
                                #if HAND ==1 && OPEN_CLOSE_HAND ==1
                                    MatrixXd tt = traj_mov.at(k);
                                    VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM+JOINTS_HAND+JOINTS_ARM);
                                    std::vector<double> hand_init_pos;
                                    hand_init_pos.resize(init_h_posture.size());
                                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                                    this->openBarrettHand_to_pos(2,hand_init_pos);
                                #elif HAND == 0 || HAND == 1
                                    closed.at(0)=false;
                                    closed.at(1)=false;
                                    closed.at(2)=false;
                                #else
                                    closed = false;
                                #endif
                            }
                            continue;
                        }
                        break;
                    case 4:// disengage
                        break;
                    case 5: // go-park
                        break;
                    }

                    if((join_ret_plan && (strcmp(mov_descr.c_str(),"plan")==0)) || ((join_ret_plan_app || join_plan_app) && (strcmp(mov_descr.c_str(),"approach")==0))){
                        traj = traj_ret_plan_app;
                        vel = vel_ret_plan_app;
                        timesteps_stage = timesteps_ret_plan_app;
                    }else{
                        traj = traj_mov.at(j);
                        vel = vel_mov.at(j);
                        timesteps_stage = timesteps_mov.at(j);
                    }

                    f_posture = traj.row(traj.rows()-1);
                    f_reached=false;
                    tol_stop_stage = tols_stop_mov.at(j);

                    ros::spinOnce(); // handle ROS messages
                    pre_time = simulationTime - timeTot; // update the total time of the movement


                    #if HAND==0
                        if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1))
                    #elif HAND==1
                        if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                            client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1))
                    #else
                        if (client_enableSubscriber.call(srv_enableSubscriber) && (srv_enableSubscriber.response.subscriberID!=-1))
                    #endif
                        {
                            #if HAND == 1
                                ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
                            #endif

                            // 5. Let's prepare a publisher of those values:
                            ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                            tb = pre_time;

                            for (int i = 0; i< vel.rows()-1; ++i){

                                VectorXd ya = vel.row(i);
                                VectorXd yb = vel.row(i+1);
                                VectorXd yat = traj.row(i);
                                VectorXd ybt = traj.row(i+1);

                                ta = tb;
                                double tt_step = timesteps_stage.at(i);
                                if(tt_step<0.001){tt_step = MIN_EXEC_TIMESTEP_VALUE;}
                                tb = ta + tt_step;

                                bool interval = true;
                                double tx_prev;
                                double yxt_prev;
                                ros::spinOnce(); // get the simulationRunning value

                                while (ros::ok() && simulationRunning && interval)
                                {// ros is running, simulation is running
                                    vrep_common::JointSetStateData dataTraj;
                                    #if HAND==1
                                        vrep_common::JointSetStateData data_hand;
                                    #endif

                                    tx = simulationTime - timeTot;
                                    if (tx >= tb){
                                        // go to the next interval
                                        interval = false;
                                    }//tx
                                    else{
                                        double m;
                                        if((tb-ta)==0){m=1;}else{m = (tx-ta)/(tb-ta);}

                                        std::vector<double> r_post; std::vector<double> l_post; std::vector<double> curr_post;
                                        switch (arm_code) {
                                        case 0: // dual arm
                                            this->curr_scene->getHumanoid()->getRightPosture(r_post);
                                            this->curr_scene->getHumanoid()->getLeftPosture(l_post);
                                            break;
                                        case 1: // right arm
                                            this->curr_scene->getHumanoid()->getRightPosture(curr_post);
                                            break;
                                        case 2: //left arm
                                            this->curr_scene->getHumanoid()->getLeftPosture(curr_post);
                                            break;
                                        }//switch arm_code

                                        double yx;
                                        double yxt;
                                        double thr = 0.0;
                                        if(arm_code!=0){
                                            thr = sqrt(pow((f_posture(0)-curr_post.at(0)),2)+
                                                    pow((f_posture(1)-curr_post.at(1)),2)+
                                                    pow((f_posture(2)-curr_post.at(2)),2)+
                                                    pow((f_posture(3)-curr_post.at(3)),2)+
                                                    pow((f_posture(4)-curr_post.at(4)),2)+
                                                    pow((f_posture(5)-curr_post.at(5)),2)+
                                                    pow((f_posture(6)-curr_post.at(6)),2));
                                        }//if arm_code
                                        else{
                                            #if HAND == 1 || HAND == 0
                                               thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                                       pow((f_posture(1)-r_post.at(1)),2)+
                                                       pow((f_posture(2)-r_post.at(2)),2)+
                                                       pow((f_posture(3)-r_post.at(3)),2)+
                                                       pow((f_posture(4)-r_post.at(4)),2)+
                                                       pow((f_posture(5)-r_post.at(5)),2)+
                                                       pow((f_posture(6)-r_post.at(6)),2)+
                                                       pow((f_posture(11)-l_post.at(0)),2)+
                                                       pow((f_posture(12)-l_post.at(1)),2)+
                                                       pow((f_posture(13)-l_post.at(2)),2)+
                                                       pow((f_posture(14)-l_post.at(3)),2)+
                                                       pow((f_posture(15)-l_post.at(4)),2)+
                                                       pow((f_posture(16)-l_post.at(5)),2)+
                                                       pow((f_posture(17)-l_post.at(6)),2));
                                            #else
                                               if(this->hand_code_right == 2){
                                                   thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                                           pow((f_posture(1)-r_post.at(1)),2)+
                                                           pow((f_posture(2)-r_post.at(2)),2)+
                                                           pow((f_posture(3)-r_post.at(3)),2)+
                                                           pow((f_posture(4)-r_post.at(4)),2)+
                                                           pow((f_posture(5)-r_post.at(5)),2)+
                                                           pow((f_posture(6)-r_post.at(6)),2)+
                                                           pow((f_posture(8)-l_post.at(0)),2)+
                                                           pow((f_posture(9)-l_post.at(1)),2)+
                                                           pow((f_posture(10)-l_post.at(2)),2)+
                                                           pow((f_posture(11)-l_post.at(3)),2)+
                                                           pow((f_posture(12)-l_post.at(4)),2)+
                                                           pow((f_posture(13)-l_post.at(5)),2)+
                                                           pow((f_posture(14)-l_post.at(6)),2));
                                               }else if(this->hand_code_right == 3){
                                                   thr = sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                                           pow((f_posture(1)-r_post.at(1)),2)+
                                                           pow((f_posture(2)-r_post.at(2)),2)+
                                                           pow((f_posture(3)-r_post.at(3)),2)+
                                                           pow((f_posture(4)-r_post.at(4)),2)+
                                                           pow((f_posture(5)-r_post.at(5)),2)+
                                                           pow((f_posture(6)-r_post.at(6)),2)+
                                                           pow((f_posture(9)-l_post.at(0)),2)+
                                                           pow((f_posture(10)-l_post.at(1)),2)+
                                                           pow((f_posture(11)-l_post.at(2)),2)+
                                                           pow((f_posture(12)-l_post.at(3)),2)+
                                                           pow((f_posture(13)-l_post.at(4)),2)+
                                                           pow((f_posture(14)-l_post.at(5)),2)+
                                                           pow((f_posture(15)-l_post.at(6)),2));
                                               }
                                            #endif
                                        }//else arm_code

                                        if(thr < tol_stop_stage){
                                            f_reached=true;
                                            log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                            break;
                                        }//if thr
                                        else{f_reached=false;}

                                        #if HAND == 0 || HAND == 1
                                            hand_closed = (closed[0] && closed[1] && closed[2]);
                                        #else
                                            hand_closed = closed;
                                        #endif

                                        for (int k = 0; k< vel.cols(); ++k){
                                            if(f_reached){
                                                yx=0;
                                                yxt=yxt_prev;
                                            }// if reached
                                            else{
                                                yx = interpolate(ya(k),yb(k),m);
                                                yxt = interpolate(yat(k),ybt(k),m);
                                                yxt_prev=yxt;
                                            }//else reached

                                            if(arm_code!=0){
                                                // single-arm
                                                #if HAND == 0 || HAND == 1
                                                    if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                                        (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)) // joints of the hand if the hand is open
                                                    {
                                                            dataTraj.handles.data.push_back(handles.at(k));
                                                    }
                                                #else
                                                   if ((k!=vel.cols()-1) || ((k==vel.cols()-1) && !hand_closed) && ((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)))
                                                    {
                                                        dataTraj.handles.data.push_back(handles.at(k));
                                                   }else if(((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)) && ((k == vel.cols()-1) || (k == vel.cols()-2))){
                                                       dataTraj.handles.data.push_back(handles.at(k));
                                                   }
                                                #endif
                                            }//if arm_code
                                            else{
                                                // dual-arm
                                                #if HAND == 0 || HAND == 1
                                                    if((k < JOINTS_ARM) || // joints of the right arm OR
                                                        ((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND) && !hand_closed)) // joints of the right hand if the hand is open
                                                    {
                                                        dataTraj.handles.data.push_back(r_handles.at(k));
                                                    }else if (((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) )|| // joints of the left arm OR
                                                            ((k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && !hand_closed)) // joints of the left hand if the hand is open
                                                    {
                                                        dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + JOINTS_HAND)));
                                                    }
                                                #else
                                                    int total_joints_right;
                                                    if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                                    else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                                    if((k < JOINTS_ARM) || // joints of the right arm OR
                                                        ((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right) && !hand_closed && (this->hand_code_right == 2))) // joints of the right hand if the hand is open
                                                    {
                                                        dataTraj.handles.data.push_back(r_handles.at(k));
                                                    }else if(((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right) && (this->hand_code_right == 3))){
                                                        dataTraj.handles.data.push_back(r_handles.at(k));
                                                    }else if (((k >= JOINTS_ARM + total_joints_right) && ( k < total_joints_right + (JOINTS_ARM*2)) )|| // joints of the left arm OR
                                                            ((k >= total_joints_right + (JOINTS_ARM*2)) && !hand_closed && (this->hand_code_left == 2))) // joints of the left hand if the hand is open
                                                    {
                                                        dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                                    }else if((k > (JOINTS_ARM*2) + total_joints_right) && (this->hand_code_left == 3)){
                                                        dataTraj.handles.data.push_back(l_handles.at(k-(JOINTS_ARM + total_joints_right)));
                                                    }
                                                #endif
                                            }// else arm_code
                                            int exec_mode;
                                            double exec_value;

                                            if(vel_mode){
                                                //velocity
                                                exec_mode = 2;
                                                exec_value = yx;
                                            }// if vel_mode
                                            else{
                                                // position
                                                exec_mode = 0;
                                                exec_value = yxt;
                                            }// else vel_mode

                                            if(arm_code!=0){
                                                // single-arm
                                                //ARoS
                                                #if HAND == 0 || HAND == 1
                                                    if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed) // joints of the hand
                                                    {
                                                        dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(yxt);
                                                    }
                                                #else
                                                    if ((k==vel.cols()-1) && !hand_closed && ((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2))){
                                                        dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(yxt);
                                                    }else if(((k==vel.cols()-1) || (k == vel.cols()-2)) && ((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3))){
                                                        dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(yxt);
                                                    }
                                                #endif

                                                #if HAND == 0 || HAND == 1
                                                    else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))) // joints of the arm
                                                    {
                                                        dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(exec_value);
                                                    }
                                                #else
                                                    else
                                                    {
                                                        dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(exec_value);
                                                    }
                                                #endif
                                            }// if arm_code
                                            else{
                                                //dual-arm
                                                #if HAND == 0 || HAND == 1
                                                     if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + JOINTS_HAND) && ( k < JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) )// joints of the right or left arm
                                                    {
                                                        dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(exec_value);
                                                    }else if ((((k >= JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) || // joints of the right hand OR
                                                            (k >= JOINTS_ARM + JOINTS_HAND + JOINTS_ARM)) && !hand_closed) // joints of the left hand if the hands are open
                                                    {
                                                        dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                        dataTraj.values.data.push_back(yxt);
                                                    }
                                                #else
                                                    int total_joints_right;
                                                    if(this->hand_code_right == 2) total_joints_right = JOINTS_HAND_Electric_Gripper;
                                                    else if(this->hand_code_right == 3) total_joints_right = JOINTS_HAND_QbSoftHand;

                                                    if((k < JOINTS_ARM) || ((k >= JOINTS_ARM + total_joints_right) && ( k < total_joints_right + (JOINTS_ARM*2))) )// joints of the right or left arm
                                                   {
                                                       dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                       dataTraj.values.data.push_back(exec_value);
                                                   }else if(((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && !hand_closed && (this->hand_code_right == 2)){
                                                       dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                       dataTraj.values.data.push_back(yxt);
                                                   }else if(((k >= JOINTS_ARM) && (k < JOINTS_ARM + total_joints_right)) && (this->hand_code_right == 3)){
                                                       dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                       dataTraj.values.data.push_back(yxt);
                                                   }else if((k >= total_joints_right + (JOINTS_ARM*2)) && !hand_closed && (this->hand_code_left == 2)){
                                                       dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                       dataTraj.values.data.push_back(yxt);
                                                   }else if((k >= total_joints_right + (JOINTS_ARM*2)) && (this->hand_code_left == 3)){
                                                       dataTraj.setModes.data.push_back(0); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                                       dataTraj.values.data.push_back(yxt);
                                                   }
                                                #endif
                                            }// else arm_code

                                            #if HAND==1

                                                if(arm_code!=0){
                                                    //single-arm
                                                    if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                                        // the fingers are being addressed
                                                        data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                                        data_hand.setModes.data.push_back(1); // set the target position
                                                        data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                                    }
                                                }else{
                                                    // dual-arm
                                                    if(((k > JOINTS_ARM) && (k < JOINTS_ARM + JOINTS_HAND)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                                        // the right fingers are being addressed
                                                        data_hand.handles.data.push_back(r_hand_handles(k+3-(JOINTS_ARM + JOINTS_HAND),2));
                                                        data_hand.setModes.data.push_back(1); // set the target position
                                                        data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                                    }else if((k > JOINTS_ARM + JOINTS_HAND + JOINTS_ARM) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2)))){
                                                        // the left fingers are being addressed
                                                        data_hand.handles.data.push_back(l_hand_handles(k-8-(JOINTS_ARM + JOINTS_HAND),2));
                                                        data_hand.setModes.data.push_back(1); // set the target position
                                                        data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                                    }
                                                }
                                            #endif
                                        }
                                        pub.publish(dataTraj);
                                        std::cout << "Data Pos-----------------------------------------------------" << std::endl;
                                        std::cout << dataTraj << std::endl;
                                        std::cout << "-----------------------------------------------------" << std::endl;
                                        #if HAND==1
                                            pub_hand.publish(data_hand);
                                        #endif

                                        interval = true;
                                        tx_prev = tx;
                                    }
                                    // handle ROS messages:
                                    ros::spinOnce();
                                }

                                    if(f_reached){
                                        log(QNode::Info,string("Final Posture reached."));
                                        break;
                                    }
                            }

                            // ---- post-movement operations ---- //
                            // set the detected object child of the attach point
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            switch (mov_type) {
                            case 0: // reach-to grasp
                                // grasp the object
                                if(approach ||(plan && (traj_mov.size()==1))){
                                    if(arm_code!=0){
                                    //single-arm
                                        #if HAND == 0 || HAND == 1
                                            if(obj_in_hand){
                                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                srvset_parent.request.parentHandle = h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }
                                        #else
                                            if((arm_code == 1 && this->hand_code_right == 2) || (arm_code == 2 && this->hand_code_left == 2)){
                                                if(obj_in_hand){
                                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                    srvset_parent.request.parentHandle = h_attach;
                                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                    add_client.call(srvset_parent);
                                                    if (srvset_parent.response.result != 1){
                                                        log(QNode::Error,string("Error in grasping the object "));
                                                    }
                                                }
                                            }else if((arm_code == 1 && this->hand_code_right == 3) || (arm_code == 2 && this->hand_code_left == 3)){
                                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                srvset_parent.request.parentHandle = h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }
                                        #endif
                                    }
                                    else{
                                        //dual-arm
                                        #if HAND == 0 || HAND == 1
                                            if(obj_in_r_hand){
                                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                srvset_parent.request.parentHandle = r_h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }
                                            if(obj_in_l_hand){
                                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                                srvset_parent.request.parentHandle = l_h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }
                                        #else
                                            if(this->hand_code_right == 2){
                                                if(obj_in_r_hand){
                                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                    srvset_parent.request.parentHandle = r_h_attach;
                                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                    add_client.call(srvset_parent);
                                                    if (srvset_parent.response.result != 1){
                                                        log(QNode::Error,string("Error in grasping the object "));
                                                    }
                                                }
                                            }else if(this->hand_code_right == 3){
                                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                                srvset_parent.request.parentHandle = r_h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }

                                            if(this->hand_code_left == 2){
                                                if(obj_in_l_hand){
                                                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                    srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                                    srvset_parent.request.parentHandle = l_h_attach;
                                                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                    add_client.call(srvset_parent);
                                                    if (srvset_parent.response.result != 1){
                                                        log(QNode::Error,string("Error in grasping the object "));
                                                    }
                                                }
                                            }else if(this->hand_code_left == 3){
                                                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                                                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                                                srvset_parent.request.handle = this->curr_mov->getObjectLeft()->getHandle();
                                                srvset_parent.request.parentHandle = l_h_attach;
                                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                                add_client.call(srvset_parent);
                                                if (srvset_parent.response.result != 1){
                                                    log(QNode::Error,string("Error in grasping the object "));
                                                }
                                            }
                                        #endif
                                    }
                                }
                                break;
                            case 1: // reaching
                                break;
                            case 2: // transport
                                break;
                            case 3: // engage
                                break;
                            case 4: // disengage
                                break;
                            case 5: // go-park
                                break;
                            }

                            // handle ROS messages:
                            ros::spinOnce();
                            timeTot = simulationTime; // update the total time of the movement
                        }// subscriber
                }
                // movement complete
                log(QNode::Info,string("Movement completed"));
                task->getProblem(kk)->getMovement()->setExecuted(true);
        }
        else{
            hh++;
        }// if prob is part of the task
    }
    log(QNode::Info,string("Task completed"));

    // pause the simulation
    this->pauseSim();

    // handle ROS messages:
    ros::spinOnce();

    TotalTime=simulationTime;

    return true;
}

#if HAND == 0 || HAND ==1
  bool QNode::execKinControl(int arm, vector<double> &r_posture, vector<double> &r_velocities)

  {

      std::vector<int> handles;
      switch (arm) {
      case 0: // dual arm
          // TO DO
          break;
      case 1: //right arm
          handles = right_handles;
          break;
      case 2: // left arm
          handles = left_handles;
          break;
      }


      if(!simulationRunning)
      {
          ros::NodeHandle node;
          // set joints position or velocity (it depends on the settings)
          ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
          vrep_common::simRosEnableSubscriber srv_enableSubscriber;
          srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
          srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
          srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
          client_enableSubscriber.call(srv_enableSubscriber);

          // start the simulation
          add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
          vrep_common::simRosStartSimulation srvstart;
          add_client.call(srvstart);

          ros::spinOnce(); // first handle ROS messages
      }

      if(ros::ok() && simulationRunning)
      {// ros is running, simulation is running


          // handle ROS messages:
          ros::spinOnce();

          vrep_common::JointSetStateData data;
          //int exec_mode = 0;
          int exec_mode = 2;
          double exec_value;
          //double time_step = simulationTime - timetot;
          for (int i = 0; i < r_velocities.size(); ++i)
          {
              //exec_value = r_posture.at(i) + (r_velocities.at(i)) * simulationTimeStep;
              exec_value = r_velocities.at(i);

              if(arm!=0){
                  // single-arm
                  data.handles.data.push_back(handles.at(i));
                  data.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                  data.values.data.push_back(exec_value);
              }else{
                  // dual-arm (TO DO)
              }
          }// for loop joints
          pub_joints.publish(data);
      }
  }

  bool QNode::execKinControl(int arm, vector<double> &r_arm_posture, vector<double> &r_arm_velocities, vector<double> &r_hand_posture, vector<double> &r_hand_velocities,bool joints_arm_vel_ctrl, bool hand_ctrl)
  {

      std::vector<int> handles;
      //MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
      switch (arm) {
      case 0: // dual arm
          // TO DO
          break;
      case 1: //right arm
          handles = right_handles;
          //hand_handles = right_hand_handles;
          break;
      case 2: // left arm
          handles = left_handles;
          //hand_handles = left_hand_handles;
          break;
      }


      if(ros::ok() && simulationRunning)
      {// ros is running, simulation is running


          // handle ROS messages:
          ros::spinOnce();

          vrep_common::JointSetStateData data;
          int exec_arm_mode; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
          if(joints_arm_vel_ctrl){
              exec_arm_mode=2;
          }else{
              exec_arm_mode=0;
          }
          int exec_hand_mode = 1; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
          double exec_value;

          for (size_t i = 0; i < r_arm_velocities.size(); ++i)
          {
              if(joints_arm_vel_ctrl){
                  exec_value =  r_arm_velocities.at(i); // vel
              }else{
                 exec_value = r_arm_posture.at(i) +  r_arm_velocities.at(i)*simulationTimeStep; // pos
              }

              if(arm!=0){
                  // single-arm
                  data.setModes.data.push_back(exec_arm_mode);
                  data.handles.data.push_back(handles.at(i));
                  data.values.data.push_back(exec_value);
              }else{
                  // dual-arm (TO DO)
              }
          }// for loop arm joints

          if(hand_ctrl){
              for (size_t i = 0; i < r_hand_velocities.size(); ++i)
              {
                  exec_value = r_hand_posture.at(i) + (r_hand_velocities.at(i)) * simulationTimeStep;

                  if(arm!=0){
                      // single-arm
                      data.setModes.data.push_back(exec_hand_mode);
                      data.handles.data.push_back(handles.at(i+r_arm_velocities.size()));
                      data.values.data.push_back(exec_value);
                  }else{
                      // dual-arm (TO DO)
                  }
              }// for loop hand joints
          }
          pub_joints.publish(data);
      }
  }

  bool QNode::execKinRealControl(int arm, vector<double> &r_arm_velocities, vector<double> &r_hand_velocities, bool hand_ctrl)
  {
      std_msgs::Float32MultiArray arr_vel;
      arr_vel.data.clear();

      if(arm!=0){
          // single arm
          for(size_t i=0;i<r_arm_velocities.size();++i){
              arr_vel.data.push_back(r_arm_velocities.at(i));
          }
          if(hand_ctrl){
              for(size_t j=0; j<r_hand_velocities.size();++j){
                  arr_vel.data.push_back(r_hand_velocities.at(j));
              }
          }
      }else{
          // dual-arm

      }

      if(ros::ok()){
        pub_real_joints.publish(arr_vel);
      }

  }

  bool QNode::execKinControlAcc(int arm, vector<double> &r_arm_posture, vector<double> &r_arm_velocities, vector<double> &r_arm_accelerations, vector<double> &r_hand_posture, vector<double> &r_hand_velocities)
{

    std::vector<int> handles;
    switch (arm) {
    case 0: // dual arm
        // TO DO
        break;
    case 1: //right arm
        handles = right_handles;
        break;
    case 2: // left arm
        handles = left_handles;
        break;
    }

    if(ros::ok() && simulationRunning)
    {// ros is running, simulation is running


        // handle ROS messages:
        ros::spinOnce();

        vrep_common::JointSetStateData data;
        int exec_arm_mode = 2; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        int exec_hand_mode = 1; // 0 to set the position, 1 to set the target position, 2 to set the target velocity
        double exec_value;

        for (size_t i = 0; i < r_arm_accelerations.size(); ++i)
        {
            exec_value =  r_arm_accelerations.at(i) * simulationTimeStep; // vel

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_arm_mode);
                data.handles.data.push_back(handles.at(i));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop arm joints

        for (size_t i = 0; i < r_hand_velocities.size(); ++i)
        {
            exec_value = r_hand_posture.at(i) + (r_hand_velocities.at(i)) * simulationTimeStep;

            if(arm!=0){
                // single-arm
                data.setModes.data.push_back(exec_hand_mode);
                data.handles.data.push_back(handles.at(i+r_arm_accelerations.size()));
                data.values.data.push_back(exec_value);
            }else{
                // dual-arm (TO DO)
            }
        }// for loop hand joints
        pub_joints.publish(data);
    }
}
#endif

double QNode::interpolate(double ya, double yb, double m)
{

    // linear interpolation
    return ya+(yb-ya)*m;

}

void QNode::startSim()
{
    ros::NodeHandle node;

    // start the simulation
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);
    this->simulationRunning=true;
    this->simulationPaused=false;
    this->simulationTimePaused=0.0;

}

void QNode::stopSim()
{

    ros::NodeHandle node;

    // stop the simulation
    add_client = node.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);
    this->simulationTime=0.0;
    this->simulationTimePaused=0.0;
    this->simulationRunning=false;
    this->simulationPaused=false;
}

void QNode::pauseSim()
{
    ros::NodeHandle node;

    // pause the simulation
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);
    this->simulationPaused=true;
    this->simulationRunning=true;
    this->simulationTimePaused = this->simulationTime;
}

double QNode::getSimTime()
{
    return this->simulationTime;
}

double QNode::getSimTimePaused()
{
    return this->simulationTimePaused;
}

double QNode::getSimTimeStep()
{
    return this->simulationTimeStep;
}

string QNode::getNodeName()
{
    return this->nodeName;
}

bool QNode::isSimulationRunning()
{
    return this->simulationRunning;
}

bool QNode::isSimulationPaused()
{
    return this->simulationPaused;
}

void QNode::enableSetJoints()
{
    ros::NodeHandle node;
    // set joints position or velocity (it depends on the settings)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+this->nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type
    client_enableSubscriber.call(srv_enableSubscriber);
}

bool QNode::checkRViz()
{

    FILE *fp;
    const int length=1000;
    char result[length]; // line to read
    std::string s2("unknown node");
    bool online=false;

    fp = popen("rosnode ping -c 1 /move_group", "r");

    int cnt=0;
    while (fgets(result, length, fp) != NULL){
     //   printf("%s", result)
        if (cnt==1){
            // second line
            std::string s1(result);

            if (s1.find(s2) != std::string::npos){
                // V-REP is off-line
                online=false;
            }else{
                // V-REP is on-line
                online=true;
            }

        }
        cnt++;

    }
    return online;

}

bool QNode::checkVrep()
{
    FILE *fp;
    const int length=1000;
    char result[length]; // line to read
    std::string s2("unknown node");
    bool online=false;

    fp = popen("rosnode ping -c 1 /vrep", "r");

    int cnt=0;
    while (fgets(result, length, fp) != NULL){
     //   printf("%s", result)
        if (cnt==1){
            // second line
            std::string s1(result);

            if (s1.find(s2) != std::string::npos){
                // V-REP is off-line
                online=false;
            }else{
                // V-REP is on-line
                online=true;
            }

        }
        cnt++;

    }
    return online;

}

void QNode::run()
{
    ros::spinOnce(); // handles ROS messages
}

void QNode::JointsCallback(const sensor_msgs::JointState &state)
{

    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());
    std::vector<double> joints_vel(state.velocity.begin(),state.velocity.end());
    std::vector<double> joints_force(state.effort.begin(),state.effort.end());

    std::vector<double> right_posture;
    std::vector<double> right_vel;
    std::vector<double> right_forces;
    std::vector<double> left_posture;
    std::vector<double> left_vel;
    std::vector<double> left_forces;

    #if HAND == 0
        const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                                "right_joint_thumb_TMC_aa","right_joint_fing1_MCP","right_joint_fing3_MCP","right_joint_thumb_TMC_fe"};
        const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                                "left_joint_thumb_TMC_aa","left_joint_fing1_MCP","left_joint_fing3_MCP","left_joint_thumb_TMC_fe"};
        const char *r_2hand[] = {"right_joint_fing1_PIP","right_joint_fing3_PIP","right_joint_thumb_MCP"};
        const char *l_2hand[] = {"left_joint_fing1_PIP","left_joint_fing3_PIP","left_joint_thumb_MCP"};

    #elif HAND == 1
        const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                                "right_BarrettHand_jointA_0","right_BarrettHand_jointB_0","right_BarrettHand_jointB_2","right_BarrettHand_jointB_1"};
        const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                                "left_BarrettHand_jointA_0","left_BarrettHand_jointB_0","left_BarrettHand_jointB_2","left_BarrettHand_jointB_1"};
        const char *r_2hand[]={"right_BarrettHand_jointC_0","right_BarrettHand_jointC_2","right_BarrettHand_jointC_1"};
        const char *l_2hand[]={"left_BarrettHand_jointC_0","left_BarrettHand_jointC_2","left_BarrettHand_jointC_1"};
    #else
        std::vector<std::string> r_names;
        std::vector<std::string> l_names;
        int total_joints_right = 0;
        int total_joints_left = 0;

        if(this->hand_code_right == 2){
             r_names = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                                "right_gripper_jointClose"};
             total_joints_right = JOINTS_ARM + JOINTS_HAND_Electric_Gripper;
        }else if(this->hand_code_right == 3){
             r_names = {"right_joint0", "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6",
                                             "qbhand2m_right_synergy_joint", "qbhand2m_right_manipulation_joint"};
             total_joints_right = JOINTS_ARM + JOINTS_HAND_QbSoftHand;
        }

        if(this->hand_code_left == 2){
             l_names = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                                            "left_gripper_jointClose"};
             total_joints_left = JOINTS_ARM + JOINTS_HAND_Electric_Gripper;
        }else if(this->hand_code_left == 3){
             l_names = {"left_joint0", "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6",
                                             "qbhand2m_left_synergy_joint", "qbhand2m_left_manipulation_joint"};
             total_joints_left = JOINTS_ARM + JOINTS_HAND_QbSoftHand;
        }
    #endif

    #if HAND == 0 || HAND == 1
        for (int i = 0; i < JOINTS_ARM+JOINTS_HAND; ++i){

            size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();
            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_names[i]) - joints_names.begin();
            if (r_index >= joints_names.size() || l_index >= joints_names.size()){
                std::cout << "element not found in state.name\n";
            }else{

                right_posture.push_back(joints_pos.at(r_index));
                right_vel.push_back(joints_vel.at(r_index));
                right_forces.push_back(joints_force.at(r_index));

                left_posture.push_back(joints_pos.at(l_index));
                left_vel.push_back(joints_vel.at(l_index));
                left_forces.push_back(joints_force.at(l_index));
            }

        }
    #else
        for (int i = 0; i < total_joints_right; ++i){

            size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();
            if (r_index >= joints_names.size()){
                std::cout << "element not found in state.name\n";
            }else{

                right_posture.push_back(joints_pos.at(r_index));
                right_vel.push_back(joints_vel.at(r_index));
                right_forces.push_back(joints_force.at(r_index));
            }

        }

        for (int i = 0; i < total_joints_left; ++i){

            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_names[i]) - joints_names.begin();
            if (l_index >= joints_names.size()){
                std::cout << "element not found in state.name\n";
            }else{
                left_posture.push_back(joints_pos.at(l_index));
                left_vel.push_back(joints_vel.at(l_index));
                left_forces.push_back(joints_force.at(l_index));
            }

        }
    #endif

    #if HAND == 0 || HAND == 1
        for(int i = 0; i < HAND_FINGERS; ++i){

            size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_2hand[i]) - joints_names.begin();
            //size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_2hand[i]) - joints_names.begin();
            if (r_index >= joints_names.size() /*|| l_index >= joints_names.size()*/){
                std::cout << "element not found in state.name\n";
            }else{

                right_2hand_pos.at(i)=joints_pos.at(r_index);
                right_2hand_vel.at(i)=joints_vel.at(r_index);
                right_2hand_force.at(i)=joints_force.at(r_index);


                /*left_2hand_pos.at(i)=joints_pos.at(l_index);
                left_2hand_vel.at(i)=joints_vel.at(l_index);
                left_2hand_force.at(i)=joints_force.at(l_index);*/

            }
        }
    #endif

    if (this->curr_scene && this->sim_robot){

        this->curr_scene->getHumanoid()->setRightPosture(right_posture);
        this->curr_scene->getHumanoid()->setLeftPosture(left_posture);
        this->curr_scene->getHumanoid()->setRightVelocities(right_vel);
        this->curr_scene->getHumanoid()->setLeftVelocities(left_vel);
        this->curr_scene->getHumanoid()->setRightForces(right_forces);
        this->curr_scene->getHumanoid()->setLeftForces(left_forces);

    }

}

void QNode::JointsRealCallback(const sensor_msgs::JointState& state)
{
    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());
    std::vector<double> joints_vel(state.velocity.begin(),state.velocity.end());
    std::vector<double> joints_force(state.effort.begin(),state.effort.end());

    std::vector<double> right_posture;
    std::vector<double> right_vel;
    std::vector<double> right_forces;

    #if HAND == 0

    #elif HAND == 1
        const char *r_names[] = {"joint 0", "joint 1", "joint 2", "joint 3","joint 4", "joint 5", "joint 6","joint 7","joint 8","joint 9","joint 10"};
        const char *r_2hand[]={"BarrettHand_jointC_0","BarrettHand_jointC_2","BarrettHand_jointC_1"};
    #elif HAND == 2
        const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                                    "right_gripper_jointClose"};
    #elif HAND == 3
    const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6"};
    #endif

    for (int i = 0; i < JOINTS_ARM/*+JOINTS_HAND*/; ++i){

        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();
        if (r_index >= joints_names.size()){
            std::cout << "element not found in state.name\n";
        }else{
            right_posture.push_back(joints_pos.at(r_index));
            right_vel.push_back(joints_vel.at(r_index));
            right_forces.push_back(joints_force.at(r_index));
        }
    }

    #if HAND == 0 || HAND == 1
        for(int i = 0; i < HAND_FINGERS; ++i){

                right_2hand_pos.at(i)=0.0;
                right_2hand_vel.at(i)=0.0;
                right_2hand_force.at(i)=0.0;

        }
    #endif

    if (this->curr_scene && !this->sim_robot){

        this->curr_scene->getHumanoid()->setRightPosture(right_posture);
        this->curr_scene->getHumanoid()->setRightVelocities(right_vel);
        this->curr_scene->getHumanoid()->setRightForces(right_forces);
    }
}

void QNode::log( const LogLevel &level, const std::string &msg)
{
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  switch ( level ) {
    case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << currentDateTime() << "]: " << msg;
        break;
    }
    case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
        break;
    }
    case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
        break;
    }
    case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << currentDateTime() << "]: " << msg;
        break;
    }
    case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << currentDateTime() << "]: " << msg;
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated();
}

void QNode::checkProximityObject(movementPtr mov,string stage)
{
    this->curr_mov = mov;
    ros::NodeHandle node;
    int h_attach;
    int arm = this->curr_mov->getArm();
    int mov_type = this->curr_mov->getType();

    switch (arm) {
    case 0: // dual arm (TO DO)
        break;
    case 1: //right arm
        h_attach = right_attach;
        break;
    case 2: // left arm
        h_attach = left_attach;
        break;
    }

    switch (mov_type){
    case 0: // reach-to-grasp
        if(stage.compare("retreat")==0){
            if(obj_in_hand){
                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                srvset_parent.request.parentHandle = h_attach;
                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                add_client.call(srvset_parent);
                if (srvset_parent.response.result != 1){
                    log(QNode::Error,string("Error in grasping the object "));
                }
            }
        break;
        }
    }
}

const std::string QNode::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

void QNode::init()
{
    logging::add_file_log
    (
        keywords::file_name = "QNode_%N.log",                                        /*< file name pattern >*/
        keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
        keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0,0,0), /*< ...or at midnight >*/
        keywords::format = "[%TimeStamp%]: %Message%",                                 /*< log record format >*/
        keywords::target = "Boost_logs"
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
}

bool QNode::getArmsHandles(int humanoid)
{

    bool succ = true;

     ros::NodeHandle node;
     ros::ServiceClient add_client;

    // get the joint arm + hand handles
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    vrep_common::simRosGetObjectHandle srvgetHandle;

    for (int k = 0; k < JOINTS_ARM; ++k){
        srvgetHandle.request.objectName = string("right_joint")+QString::number(k).toStdString();
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            right_handles.push_back(srvgetHandle.response.handle);
        }else{succ=false;}

        srvgetHandle.request.objectName = string("left_joint")+QString::number(k).toStdString();
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            left_handles.push_back(srvgetHandle.response.handle);
        }else{succ=false;}
    }

    #if HAND == 0 || HAND == 1
        for (int k = 0; k < JOINTS_HAND; ++k){
            if (k == 0){
                switch(humanoid){
                case 0: // ARoS
                    srvgetHandle.request.objectName = string("right_BarrettHand_jointA_0");
                    add_client.call(srvgetHandle);
                    if (srvgetHandle.response.handle !=-1){
                        right_handles.push_back(srvgetHandle.response.handle);
                    }else{succ=false;}

                    srvgetHandle.request.objectName = string("left_BarrettHand_jointA_0");
                    add_client.call(srvgetHandle);
                    if (srvgetHandle.response.handle !=-1){
                        left_handles.push_back(srvgetHandle.response.handle);
                    }else{succ=false;}

                    break;
                case 1: // Jarde
                    srvgetHandle.request.objectName = string("right_joint_thumb_TMC_aa");
                    add_client.call(srvgetHandle);
                    if (srvgetHandle.response.handle !=-1){
                        right_handles.push_back(srvgetHandle.response.handle);
                    }else{succ=false;}

                    srvgetHandle.request.objectName = string("left_joint_thumb_TMC_aa");
                    add_client.call(srvgetHandle);
                    if (srvgetHandle.response.handle !=-1){
                        left_handles.push_back(srvgetHandle.response.handle);
                    }else{succ=false;}

                    break;
                case 2: // RAMBO
                    break;
                }
            }else if(k == 1){
                switch(humanoid){
                case 0: // ARoS
                    #if HAND == 1
                      srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;
                case 1: // Jarde
                    #if HAND == 0
                      srvgetHandle.request.objectName = string("right_joint_fing1_MCP");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_joint_fing1_MCP");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;
                case 2: // RAMBO
                  break;
                }
            }else if(k == 9){
                switch(humanoid){
                case 0: // ARoS
                    #if HAND == 1
                      srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;
                case 1: // Jarde
                    #if HAND == 0
                      srvgetHandle.request.objectName = string("right_joint_fing3_MCP");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_joint_fing3_MCP");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;
                }
            }else if(k == 10){

                switch(humanoid){

                case 0: // ARoS
                    #if HAND == 1
                      srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;

                case 1: // Jarde
                    #if HAND == 0
                      srvgetHandle.request.objectName = string("right_joint_thumb_TMC_fe");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          right_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}

                      srvgetHandle.request.objectName = string("left_joint_thumb_TMC_fe");
                      add_client.call(srvgetHandle);
                      if (srvgetHandle.response.handle !=-1){
                          left_handles.push_back(srvgetHandle.response.handle);
                      }else{succ=false;}
                    #endif
                    break;
                }
            }
        }
    #else
       int hand_joints_right, hand_joints_left;
       if(this->hand_code_right == 2) hand_joints_right = JOINTS_HAND_Electric_Gripper;
       else if(this->hand_code_right == 3) hand_joints_right = JOINTS_HAND_QbSoftHand;

       if(this->hand_code_left == 2) hand_joints_left = JOINTS_HAND_Electric_Gripper;
       else if(this->hand_code_left == 3) hand_joints_left = JOINTS_HAND_QbSoftHand;

       for (int k = 0; k < hand_joints_right; ++k){
           if (k == 0){
               switch(humanoid){
               case 0: // ARoS
                   break;
               case 1: // Jarde
                   break;
               case 2: // RAMBO
                   if(this->hand_code_right == 2){
                       srvgetHandle.request.objectName = string("right_gripper_jointClose");
                       add_client.call(srvgetHandle);
                       if (srvgetHandle.response.handle != -1){
                           right_handles.push_back(srvgetHandle.response.handle);
                       }
                       else{succ=false;}
                   }else if(this->hand_code_right == 3){
                       srvgetHandle.request.objectName = string("qbhand2m_right_synergy_joint");
                       add_client.call(srvgetHandle);
                       if(srvgetHandle.response.handle != -1){
                         right_handles.push_back(srvgetHandle.response.handle);
                       }else{succ = false;}
                   }
                   break;
               }
           }else if(k == 1){
               switch(humanoid){
                   case 0: // ARoS
                       break;
                   case 1: // Jarde
                       break;
                   case 2: // RAMBO
                     if(this->hand_code_right == 3){
                         srvgetHandle.request.objectName = string("qbhand2m_right_manipulation_joint");
                         add_client.call(srvgetHandle);
                         if(srvgetHandle.response.handle != -1){
                           right_handles.push_back(srvgetHandle.response.handle);
                         }else{succ = false;}
                     }
                     break;
            }
           }
       }

       for (int k = 0; k < hand_joints_left; ++k){
           if (k == 0){
               switch(humanoid){
               case 0: // ARoS
                   break;
               case 1: // Jarde
                   break;
               case 2: // RAMBO
                   if(this->hand_code_left == 2){
                       srvgetHandle.request.objectName = string("left_gripper_jointClose");
                       add_client.call(srvgetHandle);
                       if (srvgetHandle.response.handle != -1){
                           left_handles.push_back(srvgetHandle.response.handle);
                       }
                       else{succ=false;}
                   }else if(this->hand_code_left == 3){
                       srvgetHandle.request.objectName = string("qbhand2m_left_synergy_joint");
                       add_client.call(srvgetHandle);
                       if(srvgetHandle.response.handle != -1){
                         left_handles.push_back(srvgetHandle.response.handle);
                       }else{succ = false;}
                   }
                   break;
               }
           }else if(k == 1){
               switch(humanoid){
                   case 0: // ARoS
                       break;
                   case 1: // Jarde
                       break;
                   case 2: // RAMBO
                    if(this->hand_code_left == 3){
                         srvgetHandle.request.objectName = string("qbhand2m_left_manipulation_joint");
                         add_client.call(srvgetHandle);
                         if(srvgetHandle.response.handle != -1){
                           left_handles.push_back(srvgetHandle.response.handle);
                         }else{succ = false;}
                     }
                     break;
                }
           }
       }
    #endif

    // get the complete hand handles
    switch(humanoid){

    case 0: // ARoS
    #if HAND == 1
        for (int k = 0; k < HAND_FINGERS; ++k){

            if (k!=1){

                  srvgetHandle.request.objectName = string("right_BarrettHand_jointA_")+QString::number(k).toStdString();
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,0)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointA_")+QString::number(k).toStdString();
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,0)=(srvgetHandle.response.handle);
                  }else{succ=false;}


            }

            if (k == 0){
                  srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("right_BarrettHand_jointC_0");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointC_0");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}

            }

            if (k == 1){
                  srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("right_BarrettHand_jointC_2");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointC_2");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}
            }

            if (k == 2){
                  srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("right_BarrettHand_jointC_1");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      right_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,1)=(srvgetHandle.response.handle);
                  }else{succ=false;}

                  srvgetHandle.request.objectName = string("left_BarrettHand_jointC_1");
                  add_client.call(srvgetHandle);
                  if (srvgetHandle.response.handle !=-1){
                      left_hand_handles(k,2)=(srvgetHandle.response.handle);
                  }else{succ=false;}

            }
        }
     #endif
            break;

    case 1: // Jarde
    #if HAND == 0
        for (int k = 0; k < HAND_FINGERS; ++k){

            if (k==2){// Thumb

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_thumb_IP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,3)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_aa");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_TMC_fe");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_thumb_IP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,3)=(srvgetHandle.response.handle);
                }else{succ=false;}




            }else if(k==0){ // Index

                srvgetHandle.request.objectName = string("right_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing1_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing1_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing1_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

            }else if(k==1){ // Ring

                srvgetHandle.request.objectName = string("right_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing3_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("right_joint_fing3_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    right_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_MCP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,0)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_PIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,1)=(srvgetHandle.response.handle);
                }else{succ=false;}

                srvgetHandle.request.objectName = string("left_joint_fing3_DIP");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1){
                    left_hand_handles(k,2)=(srvgetHandle.response.handle);
                }else{succ=false;}
            }
        }
    #endif
        break;
    case 2:
        break;
    }

    switch(humanoid){

    case 0: // ARos

        // get the object handle of the sensors
        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("right_BarrettHand_attachProxSensor");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            right_sensor=srvgetHandle.response.handle;
        }else{succ=false;}

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_BarrettHand_attachProxSensor");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            left_sensor=srvgetHandle.response.handle;
        }else{succ=false;}


        // get the object handle of the attach points
        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("right_BarrettHand_attachPoint");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            right_attach=srvgetHandle.response.handle;
        }else{succ=false;}

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_BarrettHand_attachPoint");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            left_attach=srvgetHandle.response.handle;
        }else{succ=false;}

        break;


    case 1: // Jarde

        // get the object handle of the sensors
        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("right_hand_attachProxSensor");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            right_sensor=srvgetHandle.response.handle;
        }else{succ=false;}

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_hand_attachProxSensor");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            left_sensor=srvgetHandle.response.handle;
        }else{succ=false;}


        // get the object handle of the attach points
        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("right_hand_attachPoint");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            right_attach=srvgetHandle.response.handle;
        }else{succ=false;}

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_hand_attachPoint");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1){
            left_sensor=srvgetHandle.response.handle;
        }else{succ=false;}

    break;
    case 2 : // RAMBO
        if(this->hand_code_right == 2){
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("right_gripper_attachProxSensor");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                right_sensor=srvgetHandle.response.handle;
            }else{succ=false;}

            // get the object handle of the attach points
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("right_gripper_attachPoint");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                right_attach=srvgetHandle.response.handle;
            }else{succ=false;}
        }else if(this->hand_code_right == 3){
            // get the object right handle of the attach points
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("qbhand2m_right_end_effector_fixed_joint");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                right_attach=srvgetHandle.response.handle;
            }else{succ=false;}
        }

        if(this->hand_code_left == 2){
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("left_gripper_attachProxSensor");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                left_sensor=srvgetHandle.response.handle;
            }else{succ=false;}

            // get the object handle of the attach points
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("left_gripper_attachPoint");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                left_attach=srvgetHandle.response.handle;
            }else{succ=false;}
        }else if(this->hand_code_left == 3){
            // get the object left handle of the attach points
            add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
            srvgetHandle.request.objectName = string("qbhand2m_left_end_effector_fixed_joint");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1){
                left_attach=srvgetHandle.response.handle;
            }else{succ=false;}
        }
        break;
    }
    return succ;
}

bool QNode::getSimRobot()
{
    return this->sim_robot;
}

void QNode::setSimRobot(bool sr)
{
    this->sim_robot = sr;
}

#if HAND == 1
void QNode::reset_open_close_BH()
{
    this->hand_closed = false;
}

bool QNode::open_close_BH(bool close)
{
    if(!this->hand_closed){
        open_close_bh::OpenClose_BH srv;
        srv.request.close = close;
        if(this->clientOpenCloseBH.call(srv))
        {
            this->hand_closed = srv.response.success;
            return srv.response.success;
        }else{return false;}
    }else{return false;}
}

bool QNode::closeBarrettHand(int hand)
{


    int cnt = 0;
    std::vector<int> firstPartTorqueOvershootCount(3, 0);
    firstPartLocked.at(0)=false;
    firstPartLocked.at(1)=false;
    firstPartLocked.at(2)=false;

    needFullOpening.at(0)=0;
    needFullOpening.at(1)=0;
    needFullOpening.at(2)=0;

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;


    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    //set the force
    ros::ServiceClient client_setForce = node.serviceClient<vrep_common::simRosSetJointForce>("/vrep/simRosSetJointForce");
    vrep_common::simRosSetJointForce srv_setForce;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


while (ros::ok() && simulationRunning && (!closed[0] || !closed[1] || !closed[2]) && cnt<1000){

    cnt++;

    switch (hand) {

    case 1: // right hand

        hand_handles = right_hand_handles;
        this->curr_scene->getHumanoid()->getRightHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getRightHandPosture(hand_posture);

        break;

    case 2: // left hand

        hand_handles = left_hand_handles;
        this->curr_scene->getHumanoid()->getLeftHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getLeftHandPosture(hand_posture);

        break;
    }

/*

    BOOST_LOG_SEV(lg, info) << "cnt: " << cnt << ", "<< "Hand forces: "
                            << hand_forces.at(0)<< " "
                            << hand_forces.at(1)<< " "
                            << hand_forces.at(2)<< " "
                            << hand_forces.at(3)<< " "   ;

                            */




        for (size_t i = 0; i < HAND_FINGERS; i++)
        {

            if (firstPartLocked.at(i)){

                closed[i] = true;


                // set the velocity of the second phalanx (1/3 of the velocity of the first phalanx)
                srv_setObjInt.request.handle = hand_handles(i,2);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                srv_setTarVel.request.handle = hand_handles(i,2);
                srv_setTarVel.request.targetVelocity = closingVel/3.0f;
                client_setTarVel.call(srv_setTarVel);
                //srv_setTarVel.response.results;
                //simSetJointTargetVelocity(handHandles[i][2], closingVel / 3.0f);
                //if joints locked, do nothing.



            }else if (!firstPartLocked.at(i)){

                double t = 0.0f;

                //int res = simJointGetForce(handfirstpartTorqueHandles[i], &t);
                t = hand_forces.at(i+1);

                if (abs(t) > firstPartMaxTorque)
                    firstPartTorqueOvershootCount[i] ++;
                else
                    firstPartTorqueOvershootCount[i] = 0;

                if (firstPartTorqueOvershootCount[i] >= firstPartTorqueOvershootCountRequired)
                {
                    needFullOpening[i] = 1;
                    firstPartLocked[i] = true;
                    //First part is now locked and holding the position

                    // lock the first part
                    // set the position control
                    srv_setObjInt.request.handle = hand_handles(i,1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 1;
                    client_setIntParam.call(srv_setObjInt);
                    // set the force
                    srv_setForce.request.handle = hand_handles(i,1);
                    srv_setForce.request.forceOrTorque = closingOpeningTorque*100.0f;
                    client_setForce.call(srv_setForce);
                    // set the target position
                    srv_setTarPos.request.handle = hand_handles(i,1);
                    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
                    client_setTarPos.call(srv_setTarPos);


                    // go on with the second part
                    srv_setObjInt.request.handle = hand_handles(i,2);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i,2);
                    srv_setTarVel.request.targetVelocity = closingVel / 3.0f;
                    client_setTarVel.call(srv_setTarVel);

                }
                else
                {
                    //make first joint to close with a predefined velocity
                    srv_setObjInt.request.handle = hand_handles(i,1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i,1);
                    srv_setTarVel.request.targetVelocity = closingVel;
                    client_setTarVel.call(srv_setTarVel);

                    //second joint position is 1/3 of the first
                    srv_setTarPos.request.handle = hand_handles(i,2);
                    srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1) / 3.0f;
                    client_setTarPos.call(srv_setTarPos);

                }
            }
        }

        // handle ROS messages:
        ros::spinOnce();

    } // while loop

for (size_t i = 0; i < HAND_FINGERS; i++){
    // set the position control
    srv_setObjInt.request.handle = hand_handles(i,1);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
    // set the target position
    srv_setTarPos.request.handle = hand_handles(i,1);
    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
    client_setTarPos.call(srv_setTarPos);

    srv_setObjInt.request.handle = hand_handles(i,2);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
}

    log(QNode::Info,string("Hand closed."));
    return (closed[0] && closed[1] && closed[2]);


}

bool QNode::openBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand2_pos;

    switch (hand) {
    case 1: // right hand
        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        break;
    case 2: // left hand
        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        break;
    }

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;

    for (size_t i = 0; i < HAND_FINGERS; ++i){
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,2);
        srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = false;
        needFullOpening[i] = 0;
        closed[i]=false;

    }
    log(QNode::Info,string("Hand open."));
    return true;
}

bool QNode::closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand2_pos;

    switch (hand) {
    case 1: // right hand
        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        break;
    case 2: // left hand
        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        break;
    }

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;

    for (size_t i = 0; i < HAND_FINGERS; ++i){
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i,2);
        srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = true;
        needFullOpening[i] = 1;
        closed[i]=true;

    }
    log(QNode::Info,string("Hand closed."));
    return true;
}

bool QNode::openBarrettHand(int hand)
{

    int cnt = 0;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;
    std::vector<double> hand2_pos;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    //set the force
    ros::ServiceClient client_setForce = node.serviceClient<vrep_common::simRosSetJointForce>("/vrep/simRosSetJointForce");
    vrep_common::simRosSetJointForce srv_setForce;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


while (ros::ok() && simulationRunning && (closed[0] || closed[1] || closed[2]) && cnt<1000){

    cnt++;

    switch (hand) {

    case 1: // right hand

        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        this->curr_scene->getHumanoid()->getRightHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getRightHandPosture(hand_posture);

        break;

    case 2: // left hand

        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        this->curr_scene->getHumanoid()->getLeftHandForces(hand_forces);
        this->curr_scene->getHumanoid()->getLeftHandPosture(hand_posture);

        break;
    }



    //BOOST_LOG_SEV(lg, info) << "cnt: " << cnt << ", "<< "Hand posture: "
      //                      << hand_posture.at(0)<< " "
       //                     << hand_posture.at(1)<< " "
        //                    << hand_posture.at(2)<< " "
        //                    << hand_posture.at(3)<< " "   ;



    for (size_t i = 0; i < HAND_FINGERS; i++){

        srv_setObjInt.request.handle = hand_handles(i,2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 0;
        client_setIntParam.call(srv_setObjInt);

        srv_setTarVel.request.handle = hand_handles(i,2);
        srv_setTarVel.request.targetVelocity = openingVel/3.0f;
        client_setTarVel.call(srv_setTarVel);

        if (firstPartLocked[i]){

            if(hand2_pos.at(i) < 45.5*static_cast<double>(M_PI) / 180.0){
                // unlock the first part

                // set the velocity control
                srv_setObjInt.request.handle = hand_handles(i,1);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                //make first joints to open with a predefined velocity
                srv_setTarVel.request.handle = hand_handles(i,1);
                srv_setTarVel.request.targetVelocity = openingVel;
                client_setTarVel.call(srv_setTarVel);



                firstPartLocked[i] = false;

                //BOOST_LOG_SEV(lg, info) << " First part unlocked: " << i;
            }

        }else{

            if (needFullOpening[i] != 0){

                // full opening is needed

                if (hand2_pos.at(i) < 45.5f*static_cast<double>(M_PI) / 180.0f && hand_posture.at(i+1) < 0.5f*static_cast<double>(M_PI) / 180.0f){

                    needFullOpening[i] = 0;
                    // second joint in position control
                    // set the position control
                    srv_setObjInt.request.handle = hand_handles(i,2);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 1;
                    client_setIntParam.call(srv_setObjInt);

                    // set the target position
                    srv_setTarPos.request.handle = hand_handles(i,2);
                    srv_setTarPos.request.targetPosition = 45.0f*static_cast<double>(M_PI) / 180.0f + hand_posture.at(i+1)/3.0f ;
                    client_setTarPos.call(srv_setTarPos);

                    //BOOST_LOG_SEV(lg, info) << " Full opening needed: " << i;

                }


            }else{

                // full opening is NOT needed

                //make first joint to open with a predefined velocity
                srv_setObjInt.request.handle = hand_handles(i,1);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                srv_setTarVel.request.handle = hand_handles(i,1);
                srv_setTarVel.request.targetVelocity = openingVel;
                client_setTarVel.call(srv_setTarVel);



                //BOOST_LOG_SEV(lg, info) << "hand posture " << i << hand_posture.at(i+1) <<" Full opening NOT needed ";

                if ( hand_posture.at(i+1) <= 36.0f*static_cast<double>(M_PI) / 180.0f){

                    closed[i]=false;

                    //BOOST_LOG_SEV(lg, info) << " closed false: " << i;
                }



            }



        }



    }// for loop


    // handle ROS messages:
    ros::spinOnce();

}// while loop




for (size_t i = 0; i < HAND_FINGERS; i++){
    // set the position control
    srv_setObjInt.request.handle = hand_handles(i,1);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);
    // set the target position
    srv_setTarPos.request.handle = hand_handles(i,1);
    srv_setTarPos.request.targetPosition = hand_posture.at(i+1);
    client_setTarPos.call(srv_setTarPos);

    srv_setObjInt.request.handle = hand_handles(i,2);
    srv_setObjInt.request.parameter = 2001;
    srv_setObjInt.request.parameterValue = 1;
    client_setIntParam.call(srv_setObjInt);

}

log(QNode::Info,string("Hand open."));
return true;

}
#endif

}// namespace motion_manager
