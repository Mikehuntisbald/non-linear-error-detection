//
// Created by aesv on 27/1/23.
//
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <cmath>

#ifndef CYBER_MAIN_H
#define CYBER_MAIN_H
using namespace std;
///2D-model is used
struct ret{
    Eigen::Vector3d newStateEst;
    Eigen::Matrix3d newP;
    double newMiu;
    double ret1;
    double ret2;
};

struct Sensor{
    Eigen::Vector3d stdDev;
//    Eigen::Vector3d mean;
    Eigen::Matrix3d R;
    tf::StampedTransform transform;
    ///SLAM as sensor, SLAM covariance is needed. Here assume all 0.01(sensor covariance)
    Sensor():stdDev(0.01,0.01,0.01){
        R << stdDev(1) * stdDev(1), 0, 0,
                0, stdDev(2) * stdDev(2), 0,
                0, 0, stdDev(3) * stdDev(3);
    }
    Sensor(double a, double b, double c):stdDev(a,b,c){
        R << stdDev(0) * stdDev(0), 0, 0,
                0, stdDev(1) * stdDev(1), 0,
                0, 0, stdDev(2) * stdDev(2);
    }
};

struct Process{
    Eigen::Vector3d stdDev; ///Velocity and Angular Velocity
    Eigen::Vector3d mean;
    Eigen::Matrix3d Q;
    Process():stdDev(0.002/2,0.002/0.287,0), mean(0,0,0){

    }
    void setd(double a, double b, double c){
        stdDev << a , b , c;
    }
    /// Known velocity noise
    /// \param theta
    explicit Process(double theta):stdDev(0.002/2,0.002/0.287,0), mean(0,0,0){
        Q << cos(theta) * cos(theta) * stdDev(0) * stdDev(0), sin(theta) * cos(theta) * stdDev(0) * stdDev(0), 0,
                sin(theta) * cos(theta) * stdDev(0) * stdDev(0), sin(theta) * sin(theta) * stdDev(0) * stdDev(0), 0,
                0                                                               , 0                               , stdDev(1) * stdDev(1);
    }
    void setQ(double theta){
        Q << cos(theta) * cos(theta) * stdDev(0) * stdDev(0), sin(theta) * cos(theta) * stdDev(0) * stdDev(0), 0,
                sin(theta) * cos(theta) * stdDev(0) * stdDev(0), sin(theta) * sin(theta) * stdDev(0) * stdDev(0), 0,
                0                                                               , 0                               , stdDev(1) * stdDev(1);
    }
};

class roboADS{
public:
    tf::StampedTransform transform_;
    ros::Subscriber measure_sub_;
    ros::Subscriber control_sub_;

    roboADS(): state_num_(3), control_num_(2){

    }

    roboADS(int state_num, int control_num): state_num_(state_num), control_num_(control_num){

    }

    void measureCB(const geometry_msgs::Twist::ConstPtr twist){

    }
    void controlCB(const geometry_msgs::Twist::ConstPtr twist){

    }
    ///
    /// \param control velocity comes from topic /cmd_vel
    /// \param measurement directly read from SLAM but remember to compute C at first
    /// \param mode 1 stands for 2D, 2 stands for 3D
    /// \return may return nothing, all saved in member variable

    double wraptopi(double angle)
    {
        while (angle >= 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }
private:
    ///Parameter Initialization
    u_int8_t state_num_;
    u_int8_t control_num_;
    Eigen::Matrix3d P1_;
    Eigen::Matrix3d P2_;

    double mu_;
    ///Stated in turtlebot3_description
    shared_ptr<Sensor> sensor_2d_ = make_shared<Sensor>(0.01, 0.01, 0.01);
    ///M1 lidar covariance
    shared_ptr<Sensor> sensor_3d_ = make_shared<Sensor>(0.008, 0.008, 0.008);
    shared_ptr<Process> process = make_shared<Process>();
    Eigen::Vector3d stateEst_;
    tf::TransformListener listener_;
    vector<Eigen::Vector3d> anomaly_s_{10,Eigen::Vector3d(0,0,0)};
    vector<Eigen::Vector3d>::iterator it_s_ = anomaly_s_.begin();
    vector<Eigen::Vector3d> anomaly_a_{10,Eigen::Vector3d(0,0,0)};
    vector<Eigen::Vector3d>::iterator it_a_ = anomaly_a_.begin();
};
#endif //CYBER_MAIN_H




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
//
//#ifndef map_HELPER_ROS2_H_
//#define map_HELPER_ROS2_H_
//
//#include <nav_msgs/Odometry.h>
//#include <ros/ros.h>
//#include <boost/thread.hpp>
//#include <geometry_msgs/PoseStamped.h>
//
//namespace base_local_planner {
//
//class mapHelperRos {
//public:
//
//  /** @brief Constructor.
//   * @param map_topic The topic on which to subscribe to map
//   *        messages.  If the empty string is given (the default), no
//   *        subscription is done. */
//  mapHelperRos(std::string map_topic = "map");
//  ~mapHelperRos() {}
//
//  /**
//   * @brief  Callback for receiving map data
//   * @param msg An map message
//   */
//  void mapCallback(const nav_msgs::Odometry::ConstPtr& msg);
//
//  void getmap(nav_msgs::Odometry& base_map);
//
//  void getRobotVel(geometry_msgs::PoseStamped& robot_vel);
//
//  /** @brief Set the map topic.  This overrides what was set in the constructor, if anything.
//   *
//   * This unsubscribes from the old topic (if any) and subscribes to the new one (if any).
//   *
//   * If map_topic is the empty string, this just unsubscribes from the previous topic. */
//  void setmapTopic(std::string map_topic);
//
//  /** @brief Return the current map topic. */
//  std::string getmapTopic() const { return map_topic_; }
//
//private:
//  //map topic
//  std::string map_topic_;
//
//  // we listen on map on the map topic
//  ros::Subscriber map_sub_;
//  nav_msgs::Odometry base_map_;
//  boost::mutex map_mutex_;
//  // global tf frame id
//  std::string frame_id_; ///< The frame_id associated this data
//};
//
//} /* namespace base_local_planner */
//#define CHUNKY 1
//#endif /* map_HELPER_ROS2_H_ */
