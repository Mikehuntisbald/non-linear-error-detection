#include <iostream>
#include "follower.h"
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "follow_node");
    ros::NodeHandle nh;
    int state_num, control_num;
    nh.getParam("state_num", state_num);
    nh.getParam("control_num", control_num);
    shared_ptr<Follower> follower = make_shared<Follower>(state_num, control_num);
    follower->vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ROS_INFO("Initialization Done");
    /// robot variable
//    follower->model->posteriori_stateEst_ = follower->mLeaderPose;
//    follower->mLeaderPose = follower->vInitialLeaderPose;
    double s11, s12, s13;///Must decide whether to correct before nuise start. or a1 error will accumulate
    nh.getParam("/s11",s11);nh.getParam("/s12",s12);nh.getParam("/s13",s13);
    double s21, s22, s23;///Must decide whether to correct before nuise start. or a1 error will accumulate
    nh.getParam("/s21",s21);nh.getParam("/s22",s22);nh.getParam("/s23",s23);
    /// Error Detection
    while (true){
        nh.getParam("/mode",follower->mode);
        if(follower->mode == 1){
            try{
                follower->listener_.lookupTransform("velodyne_init", "velodyne",
                                                    ros::Time(0), follower->transform_leader_);
//                follower->transform_leader_.frame_id_ = "velodyne_init";
//                follower->transform_leader_.child_frame_id_ = "leader";
////                if(follower->mode == 2){
////
////                }else{
////                    follower->transform_leader_.setOrigin(tf::Vector3(follower->transform_leader_.getOrigin().x()-s11,follower->transform_leader_.getOrigin().y()-s12,follower->transform_leader_.getOrigin().z()));
////                    Eigen::Quaterniond eq;
////                    tf::Quaternion q = follower->transform_leader_.getRotation();
////                    tf::quaternionTFToEigen(q, eq);
////                    Eigen::Vector3d tmp_vec(eq.toRotationMatrix().eulerAngles(2, 1, 0));
////                    q.setRPY(tmp_vec(2), tmp_vec(1), tmp_vec(0) - s13);
////                    follower->transform_leader_.setRotation(q);
////                }
//                follower->br_.sendTransform(follower->transform_leader_);
//                follower->mLeaderPose = Eigen::Vector3d(follower->transform_leader_.getOrigin().x(),follower->transform_leader_.getOrigin().y(),tf::getYaw(follower->transform_leader_.getRotation()));
                break;
            }
            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(0.4).sleep();
            }
        }else{
            try{
                follower->listener_.lookupTransform("camera_init", "velodyne",
                                                    ros::Time(0), follower->transform_leader_);
//                follower->transform_leader_.frame_id_ = "camera_init";
//                follower->transform_leader_.child_frame_id_ = "leader";
////                follower->transform_leader_.stamp_ = ros::Time::now();
////                if(follower->mode == 1){
////
////                }else{
////                    follower->transform_leader_.setOrigin(tf::Vector3(follower->transform_leader_.getOrigin().x()-s21,follower->transform_leader_.getOrigin().y()-s22,follower->transform_leader_.getOrigin().z()));
////                    Eigen::Quaterniond eq;
////                    tf::Quaternion q = follower->transform_leader_.getRotation();
////                    tf::quaternionTFToEigen(q, eq);
////                    Eigen::Vector3d tmp_vec(eq.toRotationMatrix().eulerAngles(2, 1, 0));
////                    q.setRPY(tmp_vec(2), tmp_vec(1), tmp_vec(0) - s23);
////                    follower->transform_leader_.setRotation(q);
////                }
//                follower->br_.sendTransform(follower->transform_leader_);
//                follower->mLeaderPose = Eigen::Vector3d(follower->transform_leader_.getOrigin().x(),follower->transform_leader_.getOrigin().y(),tf::getYaw(follower->transform_leader_.getRotation()));
                break;
            }
            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(0.4).sleep();
            }
        }
    }
    ROS_INFO("OK");
    follower->last_time = ros::Time::now().toSec();
    ros::Rate r(20);
    while(ros::ok()){
        nh.getParam("/mode",follower->mode);
        follower->leadController();
        ros::spinOnce();
        r.sleep();
    }
    return(0);

}
void Follower::leadController(){
    /// leader reference path
    LeaderReferencePath(vLeaderXParameter);
    double ts = ros::Time::now().toSec()-last_time;
    /// leader controller
    mLeaderControl(CONTROL_V) = 0.2;
    PathFollowingController();
    vLeaderXSpeed = vLeaderVirtualTargetSpeed / ( sqrt( 1 + pow( 2 * fa * vLeaderXParameter + fb, 2) ) );
    vLeaderXParameter = vLeaderXParameter + ts * vLeaderXSpeed;
    last_time = ros::Time::now().toSec();
    ControlInputSaturation();
    geometry_msgs::Twist twist;
    twist.linear.x = mLeaderControl(CONTROL_V);
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = mLeaderControl(CONTROL_W);
    vel_pub_.publish(twist);
}

void Follower::ControlInputSaturation(){
    /// input saturation
    if (mLeaderControl(CONTROL_V) > fVelocityMax){
        mLeaderControl(CONTROL_V) = fVelocityMax;
    }else if (mLeaderControl(CONTROL_V) < -fVelocityMax){
        mLeaderControl(CONTROL_V) = -fVelocityMax;
    }
    if (mLeaderControl(CONTROL_W) > fAngularVelocityMax){
        mLeaderControl(CONTROL_W) = fAngularVelocityMax;
    }else if (mLeaderControl(CONTROL_W) < -fAngularVelocityMax){
        mLeaderControl(CONTROL_W) = -fAngularVelocityMax;
    }
}

void Follower::PathFollowingController(){
    /// compute robot local state with respect to the virtual target
    if(mode == 1){
        while(ros::ok()){
            try{
                listener_.lookupTransform("leader1", "velodyne",
                                          ros::Time(0), transform_);
                break;
            }
            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(0.4).sleep();
            }
        }
    }else{
        while(ros::ok()){
            try{
                listener_.lookupTransform("leader2", "velodyne",
                                          ros::Time(0), transform_);
                break;
            }
            catch (tf::TransformException &ex) {
//                ROS_ERROR("%s",ex.what());
//                ros::Duration(0.4).sleep();
            }
        }
    }


    /// virtual target speed
    vLeaderVirtualTargetSpeed = mLeaderControl(0) * cos(tf::getYaw(transform_.getRotation())) + fk1 * transform_.getOrigin().x();

    /// robot angular velocity
    double temp_delta = -ftheta_delta * tanh( fgamma_delta * transform_.getOrigin().y() );
    double temp_dy1 = -vLeaderReferencePathCurvature * vLeaderVirtualTargetSpeed * transform_.getOrigin().x() + mLeaderControl(0) * sin( tf::getYaw(transform_.getRotation()) );
    double temp_ddelta = -ftheta_delta * (1 - pow( tanh( fgamma_delta * transform_.getOrigin().y() ), 2)) * fgamma_delta * temp_dy1;

    double temp_var;
    if (fabs(tf::getYaw(transform_.getRotation()) - temp_delta) > 1e-6){
        temp_var = (sin(tf::getYaw(transform_.getRotation())) - sin(temp_delta)) / (tf::getYaw(transform_.getRotation()) - temp_delta);
    }
    else{
        temp_var = cos((tf::getYaw(transform_.getRotation()) + temp_delta) / 2);
    }
    mLeaderControl(CONTROL_W) = temp_ddelta - fgamma * transform_.getOrigin().y() * mLeaderControl(0) * temp_var - fk2 * (tf::getYaw(transform_.getRotation()) - temp_delta) + vLeaderReferencePathCurvature * vLeaderVirtualTargetSpeed;

}

void Follower::LeaderReferencePath(const double& fXParameter){
    Eigen::Isometry3d iso;
    tf::transformTFToEigen(transform_leader_,iso);
    Eigen::Vector2d position = Eigen::Vector2d(fXParameter,fa * fXParameter * fXParameter + fb * fXParameter + fc);
    double dX = 1;
    double dY = 2 * fa * fXParameter + fb;
    double fHeading = atan2( dY, dX );
    double fCurvature = ( 2 * fa ) / pow( 1 + pow(( 2 * fa * fXParameter + fb), 2), 1.5);
    mLeaderVirtualTargetPose = Eigen::Vector3d(position(0),position(1),fHeading);
    vLeaderReferencePathCurvature = fCurvature;

    Eigen::AngleAxisd as(fHeading,Eigen::Vector3d(0,0,1));
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.rotate(as);
    delta.pretranslate(Eigen::Vector3d(position(0),position(1),0));
    iso = iso * delta;
    tf::StampedTransform t;
    tf::transformEigenToTF(iso,t);
    t.stamp_ = ros::Time::now();
    t.frame_id_ = "velodyne_init";
    t.child_frame_id_ = "leader1";
    br_.sendTransform(t);
    t.frame_id_ = "camera_init";
    t.child_frame_id_ = "leader2";
    br_.sendTransform(t);
//    return {Eigen::Vector3d(position(1),position(2),fHeading),fCurvature};
}

void Follower::controlCB(const geometry_msgs::Twist::ConstPtr& msg){

}

void Follower::getTransform(){
    try{
        listener_.lookupTransform("camera_init", "velodyne",
                                  ros::Time(0), transform_3d_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.4).sleep();
    }
    try{
        listener_.lookupTransform("velodyne_init", "velodyne",
                                  ros::Time(0), transform_2d_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.4).sleep();
    }
}