#ifndef __X_H_INCLUDED__   // if x.h hasn't been included yet...
#define __X_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/String.h>

#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

//#include "baxter_core_msgs/JointCommandMode.h"


//robot_state::RobotStatePtr kinematic_state;
robot_model::RobotModelPtr kinematic_model;
const robot_state::JointModelGroup* left_joint_model_group;
const robot_state::JointModelGroup* right_joint_model_group;
const std::vector<std::string> right_joint_names;
const std::vector<std::string> left_joint_names;
robot_state::RobotState *k_state;

baxter_core_msgs::JointCommand right_goal, left_goal;
ros::Publisher pub_joint_position, pub_joint_mode;
ros::Publisher right_joint_pub, left_joint_pub;
ros::Subscriber joint_state_sub,right_ik_sub, left_ik_sub;


//void setJointCommandMode();

baxter_core_msgs::JointCommand setJointCommand(const std::vector<std::string> &joint_names, std::vector<double> joint_values, int pos);

#endif 
