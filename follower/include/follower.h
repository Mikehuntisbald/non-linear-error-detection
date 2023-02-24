//
// Created by aesv on 27/1/23.
//
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Eigenvalues>
#include <algorithm>   //nth_element()
//#include <iomanip>      //erfc
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cfloat>
#include <numeric>
#include <eigen_conversions/eigen_msg.h>

#ifndef CYBER_MAIN_H
#define CYBER_MAIN_H
#define POSE_NUM 3
#define POSE_X 0
#define POSE_Y 1
#define POSE_THETA 2
#define CONTROL_NUM 2
#define CONTROL_V 0
#define CONTROL_W 1

using namespace std;

inline double wraptopi(double angle)
{
    while (angle >= M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

/// Struct Model contains everything during loops
struct Model{
    u_int8_t mode;
    double a;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd G{3,2};
    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

    Eigen::Vector3d residual;
    ///SLAM as sensor, SLAM covariance is needed. Here assume all 0.01(sensor covariance)

    void linearize(const double& ts, const geometry_msgs::Twist& twist){
        A << 1, 0 , -ts * twist.linear.x * sin(posteriori_stateEst_(2)),
             0, 1 , ts * twist.linear.x * cos(posteriori_stateEst_(2)),
             0, 0 , 1;

        G << ts * cos(posteriori_stateEst_(2)), 0,
             ts * sin(posteriori_stateEst_(2)), 0,
             0, ts;
    }

    Eigen::Vector3d pred(const double& ts, const Eigen::Vector2d& twist) {
        Eigen::Vector3d tmp(posteriori_stateEst_(0) + ts * twist(0) * cos(posteriori_stateEst_(2)), posteriori_stateEst_(1) + ts * twist(0) * sin(posteriori_stateEst_(2)),wraptopi(posteriori_stateEst_(2) + ts * twist(1)));
        return tmp;
    }

    Eigen::Vector3d posteriori_stateEst_;

    ///initialization
    vector<double> mu_{16,0.5};
    vector<double>::iterator it_mu_ = mu_.begin();

    ///Estimation Posteriori Covariance
    vector<Eigen::Matrix3d> Pe_{16};
    vector<Eigen::Matrix3d>::iterator it_pe_ = Pe_.begin();

    ///Actuator Covariance
//    vector<Eigen::Matrix3d> Ps_2d_{10,sensor_2d_->R};
//    vector<Eigen::Matrix3d> Ps_3d_{10,sensor_3d_->R};
    vector<Eigen::Matrix2d> Pa_{16};
    vector<Eigen::Matrix2d>::iterator it_pa_ = Pa_.begin();
    vector<Eigen::Matrix3d> Ps_{16};
    vector<Eigen::Matrix3d>::iterator it_ps_ = Ps_.begin();

    vector<Eigen::Vector3d> anomaly_s_{16,Eigen::Vector3d(0,0,0)};
    vector<Eigen::Vector3d>::iterator it_s_ = anomaly_s_.begin();
    vector<Eigen::Vector2d> anomaly_a_{16,Eigen::Vector2d(0,0)};
    vector<Eigen::Vector2d>::iterator it_a_ = anomaly_a_.begin();

    Model() = delete;
    explicit Model(u_int8_t mode) : mode(mode){}
};

/// Measurement is stored in Struct sensor
struct Sensor{
    Eigen::Vector3d stdDev;
    Eigen::Vector3d mean;
    Eigen::Matrix3d R;
    Eigen::Vector3d measurement;

    ///SLAM as sensor, SLAM covariance is needed. Here assume all 0.01(sensor covariance)
    Sensor():stdDev(0.01,0.01,0.01), mean(0,0,0){
        R << stdDev(0) * stdDev(0), 0, 0,
             0, stdDev(1) * stdDev(1), 0,
             0, 0, stdDev(2) * stdDev(2);
    }
    Sensor(double a, double b, double c):stdDev(a,b,c), mean(0,0,0){
        R << stdDev(0) * stdDev(0), 0, 0,
             0, stdDev(1) * stdDev(1), 0,
             0, 0, stdDev(2) * stdDev(2);
    }
    Sensor(const Sensor& s){
        R = s.R;
        measurement = s.measurement;
    }
};

struct Process{
    Eigen::Vector3d stdDev; ///Velocity and Angular Velocity
    Eigen::Vector3d mean;
    Eigen::Matrix3d Q;
    /// Variance Propagation
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

struct Path{
    Path(Eigen::Vector3d in1, double in2):vector(in1), fCurvature(in2){

    }
    Eigen::Vector3d vector;
    double fCurvature;

//    void initialize(Follower& follower, double fXParameter){
//    position = Eigen::Vector2d(fXParameter,follower.fa * fXParameter * fXParameter + follower.fb * fXParameter + follower.fc);
//    dX = 1;
//    dY = 2 * follower.fa *  + follower.fb;
//    fHeading = atan2( dY, dX );
//    fCurvature = ( 2 * follower.fa ) / pow( 1 + pow(( 2 * follower.fa * fXParameter + follower.fb), 2), 1.5);
//    }
};

class Follower{
public:
    shared_ptr<Model>model;
    double last_time = ros::Time::now().toSec();
    int mode{2};
    /// leader variable
    Eigen::Vector3d mLeaderPose{0,0,0};
    Eigen::Vector2d mLeaderControl{0,0};
    Eigen::Vector3d mLeaderVirtualTargetPose{0,0,0};
    double vLeaderReferencePathCurvature{0};
    double vLeaderVirtualTargetSpeed{0};
    double vLeaderXSpeed{0};
    double vLeaderXParameter{0};

    /// robot parameters
    Eigen::Vector3d vInitialLeaderPose{-0.4, 2, M_PI/2};
    double fVelocityMax = 0.26;
    double fAngularVelocityMax = 0.26;

    /// reference path parameters
    double fa = 0;
    double fb = 0;
    double fc = 0;

    /// path following control parameters
    double fk1 = 1;
    double fk2 = 1;
    double fgamma = 1;
    double ftheta_delta = M_PI/3;
    double fgamma_delta = 1;

    tf::TransformListener listener_;
    tf::StampedTransform transform_2d_;
    tf::StampedTransform transform_3d_;
    tf::StampedTransform transform_leader_;
    tf::StampedTransform transform_;

    ros::Subscriber control_sub_;
    ros::Publisher vel_pub_;
    tf::TransformBroadcaster br_;

    Follower(): state_num_(3), control_num_(2){

    }

    Follower(int state_num, int control_num): state_num_(state_num), control_num_(control_num){

    }
    void ControlInputSaturation();
    void LeaderReferencePath(const double& fXParameter);
    void leadController();
    void PathFollowingController();
    void getTransform();
    void controlCB(const geometry_msgs::Twist::ConstPtr& msg);
    ///
    /// \param control velocity comes from topic /cmd_vel
    /// \param measurement directly read from SLAM but remember to compute C at first
    /// \param mode 1 stands for 2D, 2 stands for 3D
    /// \return may return nothing, all saved in member variable
    void NUISE(double& ts, geometry_msgs::Twist& twist, const shared_ptr<Model>& model, const shared_ptr<Sensor>& sensor, const shared_ptr<Process>& process);

private:
    boost::mutex nuise_mutex_;

    double previous_t_ = ros::Time::now().toSec();

    ///Parameter Initialization
    u_int8_t state_num_;
    u_int8_t control_num_;

};

#endif //CYBER_MAIN_H

