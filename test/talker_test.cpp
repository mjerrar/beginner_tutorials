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
 *  @brief Test cases for testing the talker node 
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/ChangeStr.h>
#include <std_msgs/String.h>
#include <string>


/**
 * @brief Test case to check the existence and success of calling
 * the ChangeStr service 
 * @param none
 * @return none
 */
TEST(TESTSuite, ChangeStrSrv) {
  // Create a node handle for this test
  ros::NodeHandle nhs;
  // create a service client object for service CHangeStr
  ros::ServiceClient client = nhs.serviceClient<beginner_tutorials
                               ::ChangeStr>("ChangeStr");
  // create service call msg
  beginner_tutorials::ChangeStr srv;
  srv.request.addText = "a";
  // wait for service to be created available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
  // call the CHangStr service
  EXPECT_TRUE(client.call(srv));
  // service call should return success
  EXPECT_EQ(true, ros::service::call("ChangeStr", srv));
  // confirm the name of current node
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
