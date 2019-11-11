/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Abhinav Modi 

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/
/**@file talkerTest.cpp
 * @brief Source file containing tests for ROS nodes
 *
 * Detailed description follows here.
 * @author     : Abhinav Modi
 * @created on : Nov 11, 2019
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/modify_string.h"

/**
 * @brief      Tests whether the modify_string service exists
 * @param      TESTSuite                gtest framework
 * @param      testServiceExistence     Name of the test
 * @return     none
 */
TEST(TESTSuite, testServiceExistence) {
    // create node handle
    ros::NodeHandle n;
    // register a client to a service
    auto client = n.serviceClient<beginner_tutorials::modify_string>(
                                                         "modify_string");
    // check if the service exists
    bool exists(client.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);
}

/**
 * @brief      Tests the working of the modify_string service
 * @param      TESTSuite            gtest framework
 * @param      testServiceResponse  Name of the test
 * @return     none
 */
TEST(TESTSuite, testServiceResponse) {
    // create node handler
    ros::NodeHandle n;
    // register client to the service
    auto client = n.serviceClient<beginner_tutorials::modify_string>(
                                                        "modify_string");
    // modify the string
    beginner_tutorials::modify_string srv;
    srv.request.input = "modified_string";
    client.call(srv);
    // compare the service output string to given string
    EXPECT_STREQ("modified_string", srv.response.output.c_str());
}
