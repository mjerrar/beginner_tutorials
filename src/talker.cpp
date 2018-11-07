/************************************************************************************
/  Copyright (C) <2018>  <Jerrar Bukhari>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ************************************************************************************/

/**
 *  @file    talker.cpp
 *  @author  Jerrar Bukhari
 *  @date    11/6/2018
 *
 *  @brief Source file to implement a ROS publisher node and a service
 *         server node
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ChangeStr.h"


std::string msgModified;


/**
 *   @brief  service callback fucntion to change text string
 *
 *   @param  req string data to send to change string to
 *           resp bool success status of service call
 *   @return boolean true to indicate succesful service, false to
 *           indicate failure
 */
bool changeCallback(beginner_tutorials::ChangeStr::Request &req,
          beginner_tutorials::ChangeStr::Response &resp) {
  ROS_DEBUG_STREAM("Size of String received :" + req.addText.size());
  if (req.addText.size() > 0) {
    if (req.addText.size() < 3) {
      ROS_WARN_STREAM("Sent very short string, expecting at least 5 letters");
    }
    ROS_INFO_STREAM("Changing string");
    msgModified = req.addText;
    resp.success = true;
    return true;
  } else {
    ROS_ERROR_STREAM("empty string received");
    return false;
  }
}


/**
 *   @brief  main function implementing publisher and Service server adverstisement
 *
 *   @param  none
 *   @return integer 0 for success
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::ServiceServer changeStr = n.advertiseService("ChangeStr",
    &changeCallback);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",
    1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    std_msgs::String msg;

    std::stringstream ss;

    msg.data = msgModified;

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
