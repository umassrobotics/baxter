#ifndef __X_H_INCLUDED__   // if x.h hasn't been included yet...
#define __X_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include <ros/ros.h>


#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "checkers_common/moveChecker.h"
#include "checkers_common/removeChecker.h"
#include "checkers_common/kingChecker.h"
#include "checkers_common/unkingChecker.h"
#include "checkers_common/restChecker.h"
#include <stdlib.h>

ros::Publisher right_pub, left_pub;

ros::ServiceServer move_service, remove_service, king_service, unking_service, rest_service;

geometry_msgs::PoseStamped right, left;

std::string to_string(int number);

bool move(checkers_common::moveChecker::Request &req, checkers_common::moveChecker::Response &res);
bool removew(checkers_common::moveChecker::Request &req, checkers_common::moveChecker::Response &res);
bool king(checkers_common::kingChecker::Request &req, checkers_common::kingChecker::Response &res);
bool unking(checkers_common::unkingChecker::Request &req, checkers_common::unkingChecker::Response &res);
bool rest(checkers_common::restChecker::Request &req, checkers_common::restChecker::Response &res);

bool pickupChecker(int loc, bool king);
bool placeChecker(std::vector<int> loc, bool king);



#endif

