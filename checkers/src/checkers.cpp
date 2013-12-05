#include "checkers.h"

using namespace std;

typedef map<string, geometry_msgs::PoseStamped> positionstype;
map<string, geometry_msgs::PoseStamped> positions;

std::string to_string(int number)
{
  string number_string = "";
  char ones_char;
  int ones = 0;
  while(true){
    ones = number % 10;
    switch(ones){
      case 0: ones_char = '0'; break;
      case 1: ones_char = '1'; break;
      case 2: ones_char = '2'; break;
      case 3: ones_char = '3'; break;
      case 4: ones_char = '4'; break;
      case 5: ones_char = '5'; break;
      case 6: ones_char = '6'; break;
      case 7: ones_char = '7'; break;
      case 8: ones_char = '8'; break;
      case 9: ones_char = '9'; break;
      default : ones_char = '0';
      }
    number -= ones;
    number_string = ones_char + number_string;
    if(number == 0){
      break;
    }
    number = number/10;
  }
  return number_string;
}



//int64 start_loc    # Square ID from Below
//int64[] end_loc      # Square IDs from Below, in order
//bool king
//---
//bool success
//string message
bool move(checkers_common::moveChecker::Request &req, checkers_common::moveChecker::Response &res)
{ 
  int start;
  if( req.start_loc > 0)
  {
    start = req.start_loc;
  }
  else 
  {
    res.success = false;
    res.message = "No start location!";
    return true;
  }
  
  std::vector<int> end;
  if(req.end_loc.size() > 0)
  {
    //might be able to optimize this...
    for(int i = 0; i < req.end_loc.size(); i++)
    {
      end.push_back(req.end_loc[i]);
    }
  }
  else
  {
    res.success = false;
    res.message = "No end location!";
    return true;
  }
   res.success = true;
    
}

//int64 location    # Square ID from Below
//bool king
//--- 
//bool success
//string message
bool removew(checkers_common::moveChecker::Request &req, checkers_common::moveChecker::Response &res)
{
  return true;
}

//int64 location    # Square ID from Below
//---
//bool success
//string message
bool king(checkers_common::kingChecker::Request &req, checkers_common::kingChecker::Response &res)
{
  return true;
}

//int64 location    # Square ID from Below
//---
//bool success
//string message
bool unking(checkers_common::unkingChecker::Request &req, checkers_common::unkingChecker::Response &res)
{
  return true;
}

//---
//bool success
//string message
bool rest(checkers_common::restChecker::Request &req, checkers_common::restChecker::Response &res)
{
  return true;
}

bool pickupChecker(int loc, bool king)
{
  std::string start, goal;
  start = to_string(loc);
  start.push_back('a');
  goal = to_string(loc);
  start.push_back('b');
 if(king)
  {
    left_pub.publish(positions[start]); //point of entry
    //wait for status completion
    left_pub.publish(positions[goal]); //grasp piece
    
    //Close claw
    left_pub.publish(positions[start]); //point of entry
  }
  else
  {
    right_pub.publish(positions[start]); //point of entry
    //wait for status completion
    right_pub.publish(positions[goal]);
    //close gripper
    right_pub.publish(positions[start]); //point of entry
  }

  return true;
}

bool placeChecker(std::vector<int> loc, bool king)
{
  int i = 0;
  std::string start, goal;
  for(i = 0; i < loc.size(); i++)
  {
    start = to_string(loc[i]);
    start.push_back('a');
    goal = to_string(loc[i]);
    start.push_back('b');
    if(king)
    {
      left_pub.publish(positions[start]); //point of entry
      //wait for status completion
      left_pub.publish(positions[goal]); //grasp piece
    }
    else
    {
      right_pub.publish(positions[start]); //point of entry
      //wait for status completion
      right_pub.publish(positions[goal]);
    }
  }

  //No more moves to make
  if(king)
  {
    //open gripper
    left_pub.publish(positions[start]); //go back to neutral ish pose
  }
  else
  {
    //open gripper
    right_pub.publish(positions[start]);  //go back to neutral ish pose
  }
  return true; 
}



//Positions are a grid  with r=[1-8] and c=[a-h]
//ex positions[1a]
//position.x = depth
//position.y = width
//position.z = height
///rpy
//0,0,0 -> straight up
//90,0,0 -> pointing left
//180,0,0
//0,90,0 -> Pointing at me
//0,0,90 ->wrist rotation
void setup()
{
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  printf("Q: %f\n",q.x);
  printf("SETTING UP IN HERE!\n");
  geometry_msgs::PoseStamped pose;
      pose.pose.position.x = 0.609414;
      pose.pose.position.y = 0.1358135;
      pose.pose.position.z = 0.54124;
      pose.pose.orientation = q;
//      pose.pose.orientation.x=0.067048116303;
 //     pose.pose.orientation.y=0.885911751787;
  //    pose.pose.orientation.z=-0.108908281936;
   //   pose.pose.orientation.w=0.261868353356;
  positions["1a"] = pose;

      pose.pose.position.x = 0.6;
      pose.pose.position.y = 0.2;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x=0.367048116303;
      pose.pose.orientation.y=0.885911751787;
      pose.pose.orientation.z=-0.108908281936;
      pose.pose.orientation.w=0.261868353356;
  positions["2a"] = pose;
}

//0.809414
//0.0358135
//-0.0214124

//Based off of FIK
//0.797462
//0.992465
//0.320976

int main(int argc, char **argv)
{
  printf("...you think this is a joke?\n");
  ros::init(argc, argv, "checkers_node");
  ros::NodeHandle n;
  printf("WHAT THE FUCKING SHIT!");

  right_pub = n.advertise<geometry_msgs::PoseStamped>("/robot/limb/right/ik", 5);
  left_pub = n.advertise<geometry_msgs::PoseStamped>("/robot/limb/left/ik", 5);
   
  move_service = n.advertiseService("/checkers/move", move);
  remove_service = n.advertiseService("/checkers/remove", removew);
  king_service = n.advertiseService("/checkers/kinge", king);
  unking_service = n.advertiseService("/checkers/unking", unking);
  rest_service = n.advertiseService("/checkers/rest", rest);
 
  setup();
  //left_pub.publish(positions["1a"]);
  printf("MY VAR: %f\n",positions["1a"].pose.position.x);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
   left_pub.publish(positions["1a"]);
   ros::spinOnce();
   loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}




/*
Picking up pawn 
metry_msgs::PoseStamped pose;
      pose.pose.position.x = 0.6;
      pose.pose.position.y = 0.2;
      pose.pose.position.z = -0.05;
      pose.pose.orientation.x=0.367048116303;
      pose.pose.orientation.y=0.885911751787;
      pose.pose.orientation.z=-0.108908281936;
      pose.pose.orientation.w=0.261868353356;
*/



/*geometry_msgs::PoseStamped pose;
      pose.pose.position.x = 0.6;
      pose.pose.position.y = 0.8;
      pose.pose.position.z = 0.3;
      pose.pose.orientation.x=0.367048116303;
      pose.pose.orientation.y=0.885911751787;
      pose.pose.orientation.z=-0.108908281936;
      pose.pose.orientation.w=0.261868353356;
      left_solveik(pose);
*/
