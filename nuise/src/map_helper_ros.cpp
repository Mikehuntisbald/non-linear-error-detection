#include <iostream>
#include "map_helper_ros.h"
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixed_frame_publish_node");
    ros::NodeHandle nh;
    int state_num, control_num;
    nh.getParam("state_num", state_num);
    nh.getParam("control_num", control_num);
//    tf2_ros::Buffer buffer(ros::Duration(10));
//    tf2_ros::TransformListener tf(buffer);
    shared_ptr<roboADS> ads = make_shared<roboADS>(state_num, control_num);
//    ads->control_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [ads](auto& twist){ads->controlCB(twist);});
    tf::TransformListener listener;

    //ros::MultiThreadedSpinner s;
    ros::Rate r(20);
    static tf::TransformBroadcaster br;
    while (ros::ok()){
        try{
            ROS_INFO("Check!");

            listener.lookupTransform("map", "camera_init",
                                     ros::Time(0), ads->transform_);
            ROS_INFO("%f", ads->transform_.getRotation().getX());
            ROS_INFO("%f", ads->transform_.getRotation().getY());
            ROS_INFO("%f", ads->transform_.getRotation().getZ());
            ROS_INFO("%f", ads->transform_.getRotation().getW());
            break;
        }
        catch (tf::TransformException &ex) {
//            ROS_ERROR("%s",ex.what());
            ros::Duration(0.4).sleep();
        }
        r.sleep();
    }
    while (ros::ok()){
        ads->transform_.frame_id_ = "map";
        ads->transform_.child_frame_id_ = "velodyne_init";
        ads->transform_.stamp_ = ros::Time::now();
        br.sendTransform(ads->transform_);
//        tf::StampedTransform tr1;
//        tf::StampedTransform tr2;

//        while(true){
//            try{
//                listener.lookupTransform("velodyne_init", "velodyne",
//                                         ros::Time(0), tr1);
//                break;
//            }
//            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(1.0).sleep();
//            }
//        }
//        while(true){
//            try{
//                listener.lookupTransform("camera_init", "velodyne",
//                                         ros::Time(0), tr2);
//                break;
//            }
//            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(1.0).sleep();
//            }
//        }

        //ROS_INFO("%f",ads->transform_.stamp_.toSec());
        r.sleep();
    }

    return(0);

}



///*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of Willow Garage, Inc. nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: TKruse
// *********************************************************************/
//#include "map_helper_ros.h"
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/convert.h>
//
//namespace base_local_planner {
//
//mapHelperRos::mapHelperRos(std::string map_topic) {
//  setmapTopic( map_topic );
//}
//
//void mapHelperRos::mapCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//    ROS_INFO_ONCE("map received!");
//
//  //we assume that the map is published in the frame of the base
//  boost::mutex::scoped_lock lock(map_mutex_);
//  base_map_.twist.twist.linear.x = msg->twist.twist.linear.x;
//  base_map_.twist.twist.linear.y = msg->twist.twist.linear.y;
//  base_map_.twist.twist.angular.z = msg->twist.twist.angular.z;
//  base_map_.child_frame_id = msg->child_frame_id;
////  ROS_DEBUG_NAMED("dwa_local_planner", "In the map callback with velocity values: (%.2f, %.2f, %.2f)",
////      base_map_.twist.twist.linear.x, base_map_.twist.twist.linear.y, base_map_.twist.twist.angular.z);
//}
//
////copy over the map information
//void mapHelperRos::getmap(nav_msgs::Odometry& base_map) {
//  boost::mutex::scoped_lock lock(map_mutex_);
//  base_map = base_map_;
//}
//
//
//void mapHelperRos::getRobotVel(geometry_msgs::PoseStamped& robot_vel) {
//  // Set current velocities from map
//  geometry_msgs::Twist global_vel;
//  {
//    boost::mutex::scoped_lock lock(map_mutex_);
//    global_vel.linear.x = base_map_.twist.twist.linear.x;
//    global_vel.linear.y = base_map_.twist.twist.linear.y;
//    global_vel.angular.z = base_map_.twist.twist.angular.z;
//
//    robot_vel.header.frame_id = base_map_.child_frame_id;
//  }
//  robot_vel.pose.position.x = global_vel.linear.x;
//  robot_vel.pose.position.y = global_vel.linear.y;
//  robot_vel.pose.position.z = 0;
//  tf2::Quaternion q;
//  q.setRPY(0, 0, global_vel.angular.z);
//  tf2::convert(q, robot_vel.pose.orientation);
//  robot_vel.header.stamp = ros::Time();
//}
//
//void mapHelperRos::setmapTopic(std::string map_topic)
//{
//  if( map_topic != map_topic_ )
//  {
//    map_topic_ = map_topic;
//
//    if( !map_topic_.empty() )
//    {
//      ros::NodeHandle gn;
//      map_sub_ = gn.subscribe<nav_msgs::Odometry>( map_topic_, 1, [this](auto& msg){ mapCallback(msg); });
//    }
//    else
//    {
//      map_sub_.shutdown();
//    }
//  }
//}
//
//} /* namespace base_local_planner */
