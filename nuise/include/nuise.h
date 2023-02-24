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
//#define DV_1 = 1.15034938037601
//#define DV_2 = -0.674489750196082
//#define DV_3 = -1.15034938037601
//#define DV_4 = -1.15034938037601
//#define DV_5 = -1.15034938037601
//#define DV_6 = -1.15034938037601
//#define DV_7 = -1.15034938037601
//#define DV_8 = -1.15034938037601

using namespace std;

inline vector<double> pdet_3d(const Eigen::Matrix3d& mat){
    int n = 0;
    double prod = 1.0;
    Eigen::EigenSolver<Eigen::Matrix3d> es(mat);
    for (int i = 0; i < es.pseudoEigenvalueMatrix().rows(); ++i) {
        if (es.pseudoEigenvalueMatrix()(i,i) > 0.000001){
            prod *= es.pseudoEigenvalueMatrix()(i,i);
            n++;
        }
    }
    vector<double>ret(2);
    ret[0] = n;
    ret[1] = prod;
    return ret;
}
inline bool chi_test_a (const vector<Eigen::Vector2d>& anomaly, const vector<Eigen::Matrix2d>& anomaly_p)
{
    double alpha = 12;//14.067;
    /// Vector length 40, 8 division adapted, from 0.125 to 0.875
    /// 0.5 * erfc(-x * sqrt(0.5)) = constant
    vector<double>div(7,0);
    div[0] = -1.15034938037601;
    div[1] = -0.674489750196082;
    div[2] = -0.318639363964375;
    div[3] = 0;
    div[4] = 0.318639363964375;
    div[5] = 0.674489750196082;
    div[6] = 1.15034938037601;
    vector<vector<double>>dv(2,vector<double>(7,0));

    vector<vector<double>>cat(2,vector<double>(8,0));

    /// Average value of anomaly value and standard variance of each row
    vector<double>avg(2,0);
    vector<double>avg_p(2,0);

    for (int i = 0; i < 2; ++i) {
        double acc = 0;
        double acc_p = 0;
        for (int j = 0; j < anomaly.size(); ++j) {
            acc = acc + anomaly[j](i);
            acc_p = acc_p + anomaly_p[j](i,i);
        }
        avg[i] = acc / static_cast<int>(anomaly.size());
        avg_p[i] = sqrt(acc_p) / static_cast<int>(anomaly_p.size());
    }
    ROS_INFO("avg is x %f, y %f",avg[0],avg[1]);
    /// i stands for each element in anomaly vector, j stands for division point
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < dv[i].size(); ++j) {
            dv[i][j] = div[j] * avg_p[i] + avg[i];
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (const auto & j : anomaly) {
            if (j(i) > -DBL_MAX &&  j(i) < dv[i][1]){
                cat[i][0]++;
            }else if (j(i) > dv[i][1] &&  j(i) < dv[i][2]){
                cat[i][1]++;
            }else if (j(i) > dv[i][1] &&  j(i) < dv[i][3]){
                cat[i][2]++;
            }else if (j(i) > dv[i][2] &&  j(i) < dv[i][4]){
                cat[i][3]++;
            }else if (j(i) > dv[i][4] &&  j(i) < dv[i][5]){
                cat[i][4]++;
            }else if (j(i) > dv[i][5] &&  j(i) < dv[i][6]){
                cat[i][5]++;
            }else if (j(i) > dv[i][6] &&  j(i) < dv[i][7]){
                cat[i][6]++;
            }else if (j(i) > dv[i][7] &&  j(i) < DBL_MAX){
                cat[i][7]++;
            }
        }
    }

    ///since 40 data, each batch expects to contain 5 data.
    vector<vector<double>>chi(2,vector<double>(8,1));
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < chi[i].size(); ++j) {
            chi[i][j] = ((chi[i][j]  - cat[i][j]) * (chi[i][j]  - cat[i][j])) / chi[i][j];
        }
    }

//    if(accumulate(chi[0].begin(),chi[0].end(),0.0) > alpha){
//        ROS_INFO("Element one is under normal distribution");
//    }
//    if(accumulate(chi[1].begin(),chi[1].end(),0.0) > alpha){
//        ROS_INFO("Element one is under normal distribution");
//    }
    if(accumulate(chi[0].begin(),chi[0].end(),0.0) > alpha){
        if(accumulate(chi[1].begin(),chi[1].end(),0.0) > alpha){
            ROS_INFO("Actuator Test Pass");
            return true;
        }
    }
    ROS_INFO("Actuator Test DO NOT Pass");
    return false;
}

inline bool chi_test_s (const vector<Eigen::Vector3d>& anomaly, const vector<Eigen::Matrix3d>& anomaly_p)
{
    double alpha = 12;//14.067;
    /// Vector length 40, 8 division adapted, from 0.125 to 0.875
    /// 0.5 * erfc(-x * sqrt(0.5)) = constant
    vector<double>div(7,0);
    div[0] = -1.15034938037601;
    div[1] = -0.674489750196082;
    div[2] = -0.318639363964375;
    div[3] = 0;
    div[4] = 1.15034938037601;
    div[5] = 0.674489750196082;
    div[6] = 0.318639363964375;
    vector<vector<double>>dv(3,vector<double>(7,0));

    vector<vector<double>>cat(3,vector<double>(8,0));

    /// Average value of anomaly value and standard variance of each row
    vector<double>avg(3,0);
    vector<double>avg_p(3,0);

    for (int i = 0; i < 3; ++i) {
        double acc = 0;
        double acc_p = 0;
        for (int j = 0; j < anomaly.size(); ++j) {
            acc = acc + anomaly[j](i);
            acc_p = acc_p + anomaly_p[j](i,i);
        }
        avg[i] = acc / static_cast<int>(anomaly.size());
        avg_p[i] = sqrt(acc_p) / static_cast<int>(anomaly_p.size());
    }
    ROS_INFO("avg is x %f, y %f, z %f",avg[0],avg[1],avg[2]);
    /// i stands for each element in anomaly vector, j stands for division point
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < dv[i].size(); ++j) {
            dv[i][j] = div[j] * avg_p[i] + avg[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (const auto & j : anomaly) {
            if (j(i) > -DBL_MAX &&  j(i) < dv[i][1]){
                cat[i][0]++;
            }else if (j(i) > dv[i][1] &&  j(i) < dv[i][2]){
                cat[i][1]++;
            }else if (j(i) > dv[i][1] &&  j(i) < dv[i][3]){
                cat[i][2]++;
            }else if (j(i) > dv[i][2] &&  j(i) < dv[i][4]){
                cat[i][3]++;
            }else if (j(i) > dv[i][4] &&  j(i) < dv[i][5]){
                cat[i][4]++;
            }else if (j(i) > dv[i][5] &&  j(i) < dv[i][6]){
                cat[i][5]++;
            }else if (j(i) > dv[i][6] &&  j(i) < dv[i][7]){
                cat[i][6]++;
            }else if (j(i) > dv[i][7] &&  j(i) < DBL_MAX){
                cat[i][7]++;
            }
        }
    }

    ///since 40 data, each batch expects to contain 5 data.
    vector<vector<double>>chi(3,vector<double>(8,2));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < chi[i].size(); ++j) {
            chi[i][j] = ((chi[i][j]  - cat[i][j]) * (chi[i][j]  - cat[i][j])) / chi[i][j];
        }
    }
    if(accumulate(chi[0].begin(),chi[0].end(),0.0) > alpha){
        if(accumulate(chi[1].begin(),chi[1].end(),0.0) > alpha){
            if(accumulate(chi[2].begin(),chi[2].end(),0.0) > alpha){
                ROS_INFO("Sensor Test Pass");
                return true;
            }
        }
    }
    ROS_INFO("Sensor Test DO NOT Pass");
    return false;
}

inline Eigen::MatrixXd pinv(const Eigen::MatrixXd&  A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    double  pinvtoler = 1.e-8; //tolerance
    long int row = A.rows();
    long int col = A.cols();
    int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

    return X;

}
//template<typename _Matrix_Type_>
//_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
//std::numeric_limits<double>::epsilon())
//{
//    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
//    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
//    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
//}

inline double wraptopi(double angle)
{
    while (angle >= M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

///2D-model is used
//struct ret{
//    Eigen::Vector3d newStateEst;
//    Eigen::Matrix3d newP;
//    double newMiu;
//    double ret1;
//    double ret2;
//};
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

    Eigen::Vector3d pred(const double& ts, const geometry_msgs::Twist& twist) {
        Eigen::Vector3d tmp(posteriori_stateEst_(0) + ts * twist.linear.x * cos(posteriori_stateEst_(2)), posteriori_stateEst_(1) + ts * twist.linear.x * sin(posteriori_stateEst_(2)),wraptopi(posteriori_stateEst_(2) + ts * twist.angular.z));
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

class roboADS{
public:
    bool first{true};
    bool chance{false};
    double mode_trans = ros::Time::now().toSec();
    ///1 - mode 1, 2 - mode 2
    int mode_switch_{2};
    ///Stated in turtlebot3_description
    shared_ptr<Sensor> sensor_2d_ = make_shared<Sensor>(0.01, 0.01, 0.01);
    ///M1 lidar covariance
    shared_ptr<Sensor> sensor_3d_ = make_shared<Sensor>(0.008, 0.008, 0.008);
    shared_ptr<Process> process_2d_ = make_shared<Process>();
    shared_ptr<Process> process_3d_ = make_shared<Process>();
    shared_ptr<Model> model_2d_ = make_shared<Model>(1);
    shared_ptr<Model> model_3d_ = make_shared<Model>(2);


    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    tf::StampedTransform transform_2d_;
    tf::StampedTransform transform_3d_;

    ros::Subscriber measure_sub_;
    ros::Subscriber control_sub_;

//    double chi_alpha = 0.05;
//    double hz{50};

    roboADS(): state_num_(3), control_num_(2){

    }

    roboADS(int state_num, int control_num): state_num_(state_num), control_num_(control_num){

    }

    void getTransform();
    //void measureCB(const geometry_msgs::Twist::ConstPtr& msg);
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

