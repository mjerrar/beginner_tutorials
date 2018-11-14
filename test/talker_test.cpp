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
 *  @file    talker_test.cpp
 *  @author  Jerrar Bukhari
 *  @date    11/6/2018
 *
 *  @brief Source file to implement a ROS publisher node and a service
 *         server node
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/ChangeStr.h>
#include <std_msgs/String.h>
#include <string>


ros::NodeHandle nhs;

TEST(TESTSuite, ChangeStrSrv) {
  ros::ServiceClient client = nhs.serviceClient<beginner_tutorials
                               ::ChangeStr>("ChangeStr");
  beginner_tutorials::ChangeStr srv;
  srv.request.addText = "a";
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(client.call(srv));
  EXPECT_EQ(true, ros::service::call("ChangeStr", srv));
  EXPECT_EQ("/talker_test", ros::this_node::getName());
}


/**
 * @brief Run all rostests for talker node
 *
 * @param none
 * @return 0 on successful exit
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "modify_output");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
