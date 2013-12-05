//#include <ros/ros.h>

//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <std_msgs/String.h>

//#include "sensor_msgs/JointState.h"
//#include "baxter_msgs/JointPositions.h"
//#include "baxter_msgs/JointCommandMode.h"
#include "ik.h"

using namespace std;

//void setJointPositionMode()
/*{
        baxter_msgs::JointCommandMode msg;
        msg.mode = 1;//baxter_msgs::JointCommandMode.msg.POSITION;
        pub_joint_mode.publish(msg);
}
*/

baxter_core_msgs::JointCommand setJointPositions(const std::vector<std::string> &joint_names, std::vector<double> joint_values)
{
  baxter_core_msgs::JointCommand jp;
  for(int i = 0; i < joint_values.size(); i++)
  {
    jp.command.push_back(joint_values[i]);
    jp.names.push_back(joint_names[i]);
  }
  return jp;
}

baxter_core_msgs::JointCommand solveik(const robot_state::JointModelGroup* joint_model_group,
      const geometry_msgs::PoseStamped &pose, const std::vector<std::string> &joint_names,std::vector<double> joint_values)
{
  //kinematic_state->setToRandomPositions(joint_model_group);  
  //const Eigen::Affine3d &end_effector_state = k_state->getGlobalLinkTransform("left_hand");
  bool found_ik = k_state->setFromIK(joint_model_group, pose.pose, 10, 0.1);
  if(found_ik)
  {
     k_state->copyJointGroupPositions(joint_model_group, joint_values);
     for(std::size_t i=0; i < joint_names.size(); ++i)
     {
       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
     }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  } 
  return setJointPositions(joint_names, joint_values);
}

void solvefik(const robot_state::JointModelGroup* joint_model_group)
{ 
  const Eigen::Affine3d &end_effector_state = k_state->getGlobalLinkTransform("left_hand");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
}

void left_solveik(geometry_msgs::PoseStamped pose)
{
  const std::vector<std::string> &left_joint_names = left_joint_model_group->getJointModelNames();
  std::vector<double> joint_values;
  
  k_state->copyJointGroupPositions(left_joint_model_group, joint_values);
  left_goal = solveik( left_joint_model_group, pose,  left_joint_names, joint_values);
}

void right_solveik(geometry_msgs::PoseStamped pose)
{
  const std::vector<std::string> &right_joint_names = right_joint_model_group->getJointModelNames();
  std::vector<double> joint_values;
  
  k_state->copyJointGroupPositions(right_joint_model_group, joint_values);
  right_goal = setJointPositions( right_joint_names, joint_values);
}


void getJointStates(sensor_msgs::JointState _states)
{
//  states = _states;
}



int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::NodeHandle n;
  
  right_joint_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",5);
  left_joint_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",5);
  //pub_joint_mode = n.advertise<baxter_msgs::JointCommandMode>("/robot/limb/left/joint_command_mode",5);
  joint_state_sub = n.subscribe("/robot/joint_states", 5, getJointStates);
  right_ik_sub = n.subscribe("/robot/limb/right/ik", 5, right_solveik);
  left_ik_sub = n.subscribe("/robot/limb/left/ik", 5, left_solveik);

  //setJointPositionMode();
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //robot_state::RobotStatePtr
  k_state = new robot_state::RobotState(kinematic_model);
  //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
   
  //robot_state::RobotStatePtr kinematic_state(k_state);
  //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  //kinematic_state->setToDefaultValues();
  //  'e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2'
  //-0.9119515773559571, 0.7508835948120117, -0.7198204838928223, -0.02224272139892578, 1.0331360594604493, 1.038504992211914, -0.5069806498168946
  map<std::string, double> vcard;
  vcard["left_e0"] = -0.91195;
  vcard["left_e1"] = 0.7508835;
  vcard["left_s0"] = -0.7198204838928223;
  vcard["left_s1"] = -0.02224272139892578;
  vcard["left_w0"] = 1.0331360594604493;
  vcard["left_w1"] = 1.0385049;
  vcard["left_w2"] = -0.5069806498168946;
  //k_state->setVariablePositions(vcard);
 // k_state->setToRandomPositions();

  right_joint_model_group = k_state->getJointModelGroup("right_arm");
  left_joint_model_group = k_state->getJointModelGroup("left_arm");
  solvefik(left_joint_model_group);

  const std::vector<std::string> &right_joint_names = right_joint_model_group->getJointModelNames();
  const std::vector<std::string> &left_joint_names = left_joint_model_group->getJointModelNames();

  std::vector<double> joint_values;
  k_state->copyJointGroupPositions(left_joint_model_group, joint_values);

  for(std::size_t i = 0; i < left_joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", left_joint_names[i].c_str(), joint_values[i]);
  }
  
  //solveik(kinematic_state, joint_model_group, joint_names, joint_values);

  ros::Rate loop_rate(10);
  left_goal.mode = left_goal.POSITION_MODE;
  right_goal.mode = right_goal.POSITION_MODE;
  while (ros::ok())
  {
    if(false)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = 0.6;
      pose.pose.position.y = 0.8;
      pose.pose.position.z = 0.3;
      pose.pose.orientation.x=0.367048116303;
      pose.pose.orientation.y=0.885911751787;
      pose.pose.orientation.z=-0.108908281936;
      pose.pose.orientation.w=0.261868353356;
      left_solveik(pose);
    }
    //solvefik(left_joint_model_group);
    //right_joint_pub.publish(right_goal);
    left_joint_pub.publish(left_goal);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
