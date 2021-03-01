#include "ros/ros.h"

#include <math.h>
#include <vector>

#include "std_msgs/String.h"
#include <sensor_msgs/Range.h>

#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>

#include <gym_env/RegisterSpaces.h>
#include <gym_env/SensorMessage.h>
#include <gym_env/ActuatorMessage.h>
#include <gym_env/CloseEnv.h>
#include <gym_env/ResetEnv.h>
#include <gym_env/StepEnv.h>
#include <gym_env/GetReward.h>

#define _USE_MATH_DEFINES

#define MOTOR_NUMBER 9

static ros::ServiceClient motors[MOTOR_NUMBER];
static float motorMin[MOTOR_NUMBER];
static float motorMax[MOTOR_NUMBER];
static float motorRange[MOTOR_NUMBER];
float motorSetPoints[MOTOR_NUMBER] = {};

static ros::Subscriber sensors[MOTOR_NUMBER];
static ros::Publisher sensorPublisher;
gym_env::SensorMessage sensorMessage;

static ros::ServiceClient resetEnv;
static ros::ServiceClient stepEnv;
static ros::ServiceClient modeSetSrv;
webots_ros::set_int modeSetMsgSrv;
webots_ros::set_int stepEnvMsgSrv;

//static const char *motor_names[MOTOR_NUMBER] = {"1", "2", "3", "4", "5", "6", "7", "7_left"};
static const char *motor_names[MOTOR_NUMBER] = {"panda_joint1", "panda_joint2",
 "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
  "panda_finger_joint1", "panda_finger_joint2"};
static const char *sensor_name = "_sensor";

static char modelList[10][100];
static int count = 0;

bool finishedInit = false;

float sensor_values[MOTOR_NUMBER];

webots_ros::set_float motorSrv;

void sensorCallback(const webots_ros::Float64Stamped::ConstPtr &value, const int motor) {
    sensor_values[motor] = value->data;
    sensorMessage.observations = std::vector<float>(std::begin(sensor_values), std::end(sensor_values));
    sensorPublisher.publish(sensorMessage);
    ROS_INFO("Sensor #%d: %f.", motor, sensor_values[motor]);
}

void initSensors(ros::NodeHandle &n, std::string &modelName) {

    webots_ros::set_int enableSensSrv;
    enableSensSrv.request.value = 80;

    for (int i = 0; i < MOTOR_NUMBER; ++i) {
        
        ros::ServiceClient sc = n.serviceClient<webots_ros::set_int>(modelName + "/" + motor_names[i] + sensor_name + "/enable");
        if (sc.call(enableSensSrv) && enableSensSrv.response.success) {
            ROS_INFO("Device %d enabled.", i);
            sensors[i] = n.subscribe<webots_ros::Float64Stamped>(modelName + "/" + motor_names[i] + sensor_name + "/value", 1, boost::bind(sensorCallback, _1, i));
            while (sensors[i].getNumPublishers() == 0) {
            }
        }
    }
}

void modelNameCallback(const std_msgs::String::ConstPtr &name) {
  count++;
  strcpy(modelList[count], name->data.c_str());
  ROS_INFO("Model #%d: %s.", count, name->data.c_str());
}

bool sensorSpacesCallback(gym_env::RegisterSpaces::Request& request, gym_env::RegisterSpaces::Response& response) {

    while (!finishedInit) {
        return false;
    }

    response.low = std::vector<float>(std::begin(motorMin), std::end(motorMin));
    std::for_each(response.low.begin(), response.low.end(), [](float& f) { f-=2e-5;}); //Avoids rounding error triggers
    response.high = std::vector<float>(std::begin(motorMax), std::end(motorMax));
    std::for_each(response.high.begin(), response.high.end(), [](float& f) { f+=2e-5;}); //Avoids rounding error triggers
    response.topic = "/joint_sensors/value";
    return true;
}

bool actuatorSpacesCallback(gym_env::RegisterSpaces::Request& request, gym_env::RegisterSpaces::Response& response) {

    while (!finishedInit) {
        return false;
    }

    response.low = std::vector<float>(MOTOR_NUMBER, -1.0);
    response.high = std::vector<float>(MOTOR_NUMBER, 1.0);

    //response.low = std::vector<float>(std::begin(motorMin), std::end(motorMin));
    //response.high = std::vector<float>(std::begin(motorMax), std::end(motorMax));
    response.topic = "/joint_actuators/value";
    return true;
}

void actuatorCommandCallback(const gym_env::ActuatorMessage::ConstPtr &message) {
    
    for (int i = 0; i < MOTOR_NUMBER; ++i) {
        motorSetPoints[i] += motorRange[i]*message->actions[i]/20;
        motorSetPoints[i] = std::min(std::max(motorSetPoints[i], motorMin[i]), motorMax[i]);
        motorSrv.request.value = motorSetPoints[i];
        motors[i].call(motorSrv);
    }
}

void initActuators(ros::NodeHandle &n, std::string &modelName) {
    webots_ros::get_float gfSrv;
    gfSrv.request.ask = true;

    for (int i = 0; i < MOTOR_NUMBER; ++i) {
        motors[i] = n.serviceClient<webots_ros::set_float>(modelName + "/" + motor_names[i] + "/set_position");

        ros::ServiceClient mmServ = n.serviceClient<webots_ros::get_float>(modelName + "/" + motor_names[i] + "/get_max_position");
        mmServ.call(gfSrv);
        motorMax[i] = gfSrv.response.value - 1e-5;
        mmServ = n.serviceClient<webots_ros::get_float>(modelName + "/" + motor_names[i] + "/get_min_position");
        mmServ.call(gfSrv);
        motorMin[i] = gfSrv.response.value + 1e-5;
        motorRange[i] = motorMax[i] - motorMin[i];
    }
}

bool stepCallback(gym_env::StepEnv::Request& request, gym_env::StepEnv::Response& response) {
    ROS_INFO("Step");

    /*
    //Set fast mode
    modeSetMsgSrv.request.value = 2;
    modeSetSrv.call(modeSetMsgSrv);
    */

    //Wait for step to finish
    while(!(stepEnv.call(stepEnvMsgSrv) && stepEnvMsgSrv.response.success)) {
        ROS_WARN("Failed step, retrying.");
    }

    /*
    //Set realtime mode
    modeSetMsgSrv.request.value = 1;
    modeSetSrv.call(modeSetMsgSrv);
    */
    return true;
}

bool closeCallback(gym_env::CloseEnv::Request& request, gym_env::CloseEnv::Response& response) {
    ROS_INFO("Close");
    return true;
}

bool resetCallback(gym_env::ResetEnv::Request& request, gym_env::ResetEnv::Response& response, ros::NodeHandle &n, std::string &modelName) {
    ROS_INFO("Reset");
    webots_ros::get_bool resetSrv;
    resetSrv.request.ask = true;

    bool reset = resetEnv.call(resetSrv) && resetSrv.response.value;

    if (!reset) {
        return false;
    }
    
    webots_ros::set_int msg;
    msg.request.value = 8;

    //We need to step for the reset to take effect
    stepEnv.call(msg);

    while (!(stepEnv.call(msg) && msg.response.success)) {
        ros::Duration(0.5).sleep(); //Retry every half second
    }
    initSensors(n, modelName);

    ROS_INFO("Reset done");
    return true;
}

bool rewardCallback(gym_env::GetReward::Request& request, gym_env::GetReward::Response& response) {
    float currentReward = 0.0;
    for (int i = 0; i < MOTOR_NUMBER; i++) {
        //ROS_INFO("%f", sensor_values[i]);
        currentReward -= std::abs(sensor_values[i] - (motorMin[i] + motorRange[i]/2));
    }
    response.reward = currentReward;

    ROS_INFO("Reward %f", response.reward);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "webots_gym");

    ros::NodeHandle n;

    // declaration of variable names used to define services and topics name dynamically
    std::string modelName;

    // get the name of the robot
    ros::Subscriber nameSub = n.subscribe("model_name", 100, modelNameCallback);
    while (count == 0 || count < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();
    if (count == 1)
        modelName = modelList[1];
    else {
        int wantedModel = 0;
        std::cout << "Choose the # of the model you want to use:\n";
        std::cin >> wantedModel;
        if (1 <= wantedModel && wantedModel <= count)
        modelName = modelList[wantedModel];
        else {
        ROS_ERROR("Invalid choice.");
        return 1;
        }
    }
    nameSub.shutdown();

    resetEnv = n.serviceClient<webots_ros::get_bool>(modelName + "/supervisor/simulation_reset");
    stepEnv = n.serviceClient<webots_ros::set_int>(modelName + "/robot/time_step");
    stepEnvMsgSrv.request.value = 80; //ms

    initActuators(n, modelName);
    initSensors(n, modelName);

    //modeSetSrv = n.serviceClient<webots_ros::set_int>(modelName + "/supervisor/simulation_set_mode");

    ros::ServiceServer sensorService = n.advertiseService("/joint_sensors", sensorSpacesCallback);
    ros::ServiceServer actuatorService = n.advertiseService("/joint_actuators", actuatorSpacesCallback);

    ros::ServiceServer stepService = n.advertiseService("/webots/step", stepCallback);
    ros::ServiceServer closeService = n.advertiseService("/webots/close", closeCallback);
    ros::ServiceServer resetService = n.advertiseService<gym_env::ResetEnv::Request, gym_env::ResetEnv::Response>("/webots/reset", boost::bind(resetCallback, _1, _2, boost::ref(n), boost::ref(modelName)));
    ros::ServiceServer rewardService = n.advertiseService("/reward", rewardCallback);

    ros::Subscriber actionSubscriber = n.subscribe("/joint_actuators/value", 1, actuatorCommandCallback);
    sensorPublisher = n.advertise<gym_env::SensorMessage>("/joint_sensors/value", 1);


    finishedInit = true;


    ros::spin();
}