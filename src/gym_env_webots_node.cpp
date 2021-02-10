#include "ros/ros.h"

#include <math.h>
#include <vector>

#include "std_msgs/String.h"
#include <sensor_msgs/Range.h>

#include <webots_ros/get_bool.h>
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

#define MOTOR_NUMBER 8

static ros::ServiceClient motors[MOTOR_NUMBER];
static ros::Subscriber sensors[MOTOR_NUMBER];
static ros::Publisher sensorPublisher;
gym_env::SensorMessage sensorMessage;

static ros::ServiceClient resetEnv;

static const char *motor_names[MOTOR_NUMBER] = {"1", "2", "3", "4", "5", "6", "7", "7_left"};
static const char *sensor_name = "_sensor";

static char modelList[10][100];
static int count = 0;

float sensor_values[MOTOR_NUMBER];

float previousReward = 0.0;

webots_ros::set_float motorSrv;

void sensorCallback(const webots_ros::Float64Stamped::ConstPtr &value, const int motor) {
    sensor_values[motor] = value->data;
    sensorMessage.observations = std::vector<float>(sensor_values, sensor_values + MOTOR_NUMBER);
    sensorPublisher.publish(sensorMessage);
    //ROS_INFO("Sensor #%d: %f.", motor, sensor_values[motor]);
}

void modelNameCallback(const std_msgs::String::ConstPtr &name) {
  count++;
  strcpy(modelList[count], name->data.c_str());
  ROS_INFO("Model #%d: %s.", count, name->data.c_str());
}

bool sensorSpacesCallback(gym_env::RegisterSpaces::Request& request, gym_env::RegisterSpaces::Response& response) {

    response.low = std::vector<float>(MOTOR_NUMBER, -2.967);
    response.high = std::vector<float>(MOTOR_NUMBER, 2.967);
    response.topic = "/joint_sensors/value";
    return true;
}

bool actuatorSpacesCallback(gym_env::RegisterSpaces::Request& request, gym_env::RegisterSpaces::Response& response) {

    response.low = std::vector<float>(MOTOR_NUMBER, -2.967);
    response.high = std::vector<float>(MOTOR_NUMBER, 2.967);
    response.topic = "/joint_actuators/value";
    return true;
}

void actuatorCommandCallback(const gym_env::ActuatorMessage::ConstPtr &message) {

    for (int i = 0; i < MOTOR_NUMBER; ++i) {
        motorSrv.request.value = message->actions[i];
        motors[i].call(motorSrv);
    }
}

bool stepCallback(gym_env::StepEnv::Request& request, gym_env::StepEnv::Response& response) {
    ROS_INFO("Step");
    ros::Duration(0.1).sleep();
    return true;
}

bool closeCallback(gym_env::CloseEnv::Request& request, gym_env::CloseEnv::Response& response) {
    ROS_INFO("Close");
    return true;
}

bool resetCallback(gym_env::ResetEnv::Request& request, gym_env::ResetEnv::Response& response) {
    ROS_INFO("Reset");
    webots_ros::get_bool resetSrv;
    resetSrv.request.ask = true;
    return resetEnv.call(resetSrv) && resetSrv.response.value;
}

bool rewardCallback(gym_env::GetReward::Request& request, gym_env::GetReward::Response& response) {
    float currentReward = 0.0;
    for (float val : sensor_values) {
        if (std::abs(val - 0.5) < 0.2)
            currentReward += 1;
    }
    response.reward = currentReward - previousReward;

    ROS_INFO("Reward %f", response.reward);

    previousReward = currentReward;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "webots_gym");

    ros::NodeHandle n;

    ros::ServiceServer sensorService = n.advertiseService("/joint_sensors", sensorSpacesCallback);
    ros::ServiceServer actuatorService = n.advertiseService("/joint_actuators", actuatorSpacesCallback);

    ros::ServiceServer stepService = n.advertiseService("/webots/step", stepCallback);
    ros::ServiceServer closeService = n.advertiseService("/webots/close", closeCallback);
    ros::ServiceServer resetService = n.advertiseService("/webots/reset", resetCallback);
    ros::ServiceServer rewardService = n.advertiseService("/reward", rewardCallback);

    ros::Subscriber actionSubscriber = n.subscribe("/joint_actuators/value", 1, actuatorCommandCallback);
    sensorPublisher = n.advertise<gym_env::SensorMessage>("/joint_sensors/value", 1);


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

    motorSrv.request.value = 1.0;

    webots_ros::set_int enableSensSrv;
    enableSensSrv.request.value = 100;

    for (int i = 0; i < MOTOR_NUMBER; ++i) {
        motors[i] = n.serviceClient<webots_ros::set_float>(modelName + "/" + motor_names[i] + "/set_position");
        //motors[i].call(motorSrv);
        
        ros::ServiceClient sc = n.serviceClient<webots_ros::set_int>(modelName + "/" + motor_names[i] + sensor_name + "/enable");
        if (sc.call(enableSensSrv) && enableSensSrv.response.success) {
            ROS_INFO("Device %d enabled.", i);
            sensors[i] = n.subscribe<webots_ros::Float64Stamped>(modelName + "/" + motor_names[i] + sensor_name + "/value", 1, boost::bind(sensorCallback, _1, i));
            while (sensors[i].getNumPublishers() == 0) {
            }
        }
    }


    ros::spin();
}