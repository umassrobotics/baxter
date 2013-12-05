#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/String.h>

#include "sensor_msgs/JointState.h"
#include "baxter_msgs/JointPositions.h"
#include "baxter_msgs/JointCommandMode.h"
#include "baxter_msgs/SolvePositionIK.h"
#include "baxter_msgs/SolvePositionIKRequest.h"

#include "geometry_msgs/PoseStamped.h"

using namespace std;

bool sending = false;
ros::Publisher right_joint_pub, left_joint_pub, pub_joint_mode;
ros::Subscriber joint_state_sub, ik_sub;

void setJointPositionMode()
{
        baxter_msgs::JointCommandMode msg;
        msg.mode = 1;//baxter_msgs::JointCommandMode.msg.POSITION;
        pub_joint_mode.publish(msg);
}

baxter_msgs::JointPositions right, left;

void setRightJointPositions(const std::vector<std::string> &joint_names, std::vector<double> joint_values)
{
  sending = true;
  baxter_msgs::JointPositions jp;
  for(int i = 0; i < joint_values.size(); i++)
  {
    jp.angles.push_back(joint_values[i]);
    jp.names.push_back(joint_names[i]);
  }
  setJointPositionMode();
  right = jp;
}


void setLeftJointPositions(const std::vector<std::string> &joint_names, std::vector<double> joint_values)
{
  sending = true;
  baxter_msgs::JointPositions jp;
  for(int i = 0; i < joint_values.size(); i++)
  {
    jp.angles.push_back(joint_values[i]);
    jp.names.push_back(joint_names[i]);
  }
  setJointPositionMode();
  left = jp;
}

void left_solveik(geometry_msgs::PoseStamped pose)
{
  left = setJointPositions(joint_names, joint_values);
}

void right_solveik(geometry_msgs::PoseStamped pose)
{
  
  right = setJointPositions(joint_names, joint_values);
}

void getJointStates(sensor_msgs::JointState _states)
{
  states = _states;
}



int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::NodeHandle n;
  
  right_joint_pub = n.advertise<baxter_msgs::JointPositions>("/robot/limb/right/command_joint_angles",5);
  left_joint_pub = n.advertise<baxter_msgs::JointPositions>("/robot/limb/left/command_joint_angles",5);
  pub_joint_mode = n.advertise<baxter_msgs::JointCommandMode>("/robot/limb/left/joint_command_mode",5);
  joint_state_sub = n.subscribe("/robot/joint_states", 5, getJointStates);  
  right_ik_sub = n.subscribe("/robot/limb/right/ik", 5, isolveik);
  left_ik_sub = n.subscribe("/robot/limb/left/ik", 5, solveik);
  setJointPositionMode();
  
  
  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
    right_joint_pub.publish(right);
    left_joint_pub.publish(left);
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  ros::shutdown();
  return 0;
}
