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

// Headers
#include <tf/transform_broadcaster.h>
#include <sstream>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "beginner_tutorials/ChangeStr.h"

// Global Variable to communicate between callback and main func
std::string msgModified = "vacant";

/**
 *   @brief  service callback fucntion to change text string
 *
 *   @param  req is string data to send to change string to
 *           resp is bool success status of service call
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


  /**
   * NodeHandle points to the current node for communication with 
   * the rest of the ROS system
   */
  ros::NodeHandle n;

  /**
   * Declaration of service server that will advertise its 
   * availability to the ROS master
   */
  ros::ServiceServer changeStr = n.advertiseService("ChangeStr",
    &changeCallback);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",
    1000);
  // loop rate ensures publishing of commands at uniform intervals
  ros::Rate loop_rate(10);

  // create broadcaster for transform
  tf::TransformBroadcaster br;
  // create the transform object
  tf::Transform transform;
  // set an arbitrary location vector
  transform.setOrigin(tf::Vector3(5.0, 5.0, 1.0));
  /**
   * quaternions are the method of choice for rotation representation
   * in computer vision and robotics applications. Its does not suffer
   * from gimbal lock.
   */
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 1.571);
  // set an arbitrary rotation vector for the transform
  transform.setRotation(q);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    // update the msg to be published from global variable
    msg.data = msgModified;

    ROS_INFO("%s", msg.data.c_str());
    // publsih msg to topic
    chatter_pub.publish(msg);
    // broadcast the transform according to the parent child and time
    // mentioned
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "world", "talk"));
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
