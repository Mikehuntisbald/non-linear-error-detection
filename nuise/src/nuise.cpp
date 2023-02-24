#include <iostream>
#include "nuise.h"
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "nuise_node");
    ros::NodeHandle nh;
    int state_num, control_num;
    nh.getParam("state_num", state_num);
    nh.getParam("control_num", control_num);
//    tf2_ros::Buffer buffer(ros::Duration(10));
//    tf2_ros::TransformListener tf(buffer);
    shared_ptr<roboADS> ads = make_shared<roboADS>(state_num, control_num);
    ads->control_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [ads](auto& twist){ads->controlCB(twist);});

//    ros::Rate r(100);
    while (true){
        try{
            ROS_INFO("Check1!!");

            ads->listener_.lookupTransform("camera_init", "velodyne",
                                           ros::Time(0), ads->transform_3d_);

        }
        catch (tf::TransformException &ex) {
            ros::Duration(0.5).sleep();
        }

        try{
            ROS_INFO("Check2!!");

            ads->listener_.lookupTransform("velodyne_init", "velodyne",
                                           ros::Time(0), ads->transform_2d_);
            break;

        }
        catch (tf::TransformException &ex) {
            ros::Duration(0.5).sleep();
        }
//        r.sleep();
    }
    ROS_INFO("OK");
    /// Initial measurement as initial value
    ads->model_2d_->posteriori_stateEst_ = Eigen::Vector3d(ads->transform_2d_.getOrigin().getX(),ads->transform_2d_.getOrigin().getY(),getYaw(ads->transform_2d_.getRotation()));
    ads->model_3d_->posteriori_stateEst_ = Eigen::Vector3d(ads->transform_3d_.getOrigin().getX(),ads->transform_3d_.getOrigin().getY(),getYaw(ads->transform_3d_.getRotation()));

    for (int i = 0; i < ads->model_2d_->Pe_.size(); ++i) {
        ads->model_2d_->Pe_[i] = ads->sensor_2d_->R;
        ads->model_3d_->Pe_[i] = ads->sensor_3d_->R;
        ///yisiyisi
        ads->model_2d_->Pa_[i] << ads->process_2d_->Q(0,0)+ads->process_2d_->Q(1,1), 0,
                0, ads->process_2d_->Q(2,2);
        ads->model_3d_->Pa_[i] << ads->process_3d_->Q(0,0)+ads->process_3d_->Q(1,1), 0,
                0, ads->process_3d_->Q(2,2);
        ads->model_2d_->Ps_[i] = ads->sensor_2d_->R;
        ads->model_3d_->Ps_[i] = ads->sensor_3d_->R;
    }
    ads->mode_trans = ros::Time::now().toSec();
//    ROS_INFO("22222");
    ros::Rate r(30);
    while(ros::ok()){
        nh.setParam("/mode",ads->mode_switch_);
        ros::spinOnce();
        r.sleep();
    }
    return(0);

}
void roboADS::NUISE(double& ts, geometry_msgs::Twist& twist, const shared_ptr<Model>& model, const shared_ptr<Sensor>& sensor, const shared_ptr<Process>& process){
    /// Estimation Covariance
    Eigen::Matrix3d P_tilde = model->A * model->Pe_.back() * model->A.transpose() + process->Q;
//    cout<<"model->A "<<model->A << endl;
//    cout<<"process->Q "<<process->Q << endl;
//    cout<<"Pe "<< model->Pe_.back()<< endl;
    /// Biased Measurement Covariance
    Eigen::Matrix3d R_star = model->C * P_tilde * model->C.transpose() + sensor->R;
//    cout<<"R_starinv "<<R_star.inverse() << endl;
//    cout<<"G "<<model->G << endl;
    /// Gaussian Markov weighted Sum
    Eigen::MatrixXd GMM(2,3);
    GMM = (model->G.transpose() * model->C.transpose() * model->C * model->G).inverse() * model->G.transpose() * model->C.transpose();
//    GMM = (model->G.transpose() * model->C.transpose() * R_star.inverse() * model->C * model->G).inverse() * model->G.transpose() * model->C.transpose() * R_star.inverse();
//    cout<<"GMMinv "<<(model->G.transpose() * model->C.transpose() * R_star.inverse() * model->C * model->G).inverse() << endl;
//    cout<<"GMM "<<model->G.transpose() * model->C.transpose() * R_star.inverse() * model->C * model->G << endl;
    /// Anomaly Vector Estimation
    Eigen::Vector3d tmp_est = model->pred(ts, twist);
    model->it_a_ = model->anomaly_a_.erase(model->it_a_);
    model->anomaly_a_.emplace_back(GMM * (sensor->measurement - model->C * tmp_est));
    model->it_pa_ = model->Pa_.erase(model->it_pa_);
    model->Pa_.emplace_back(GMM * R_star * GMM.transpose());

    /// Re-prediction using anomaly actuator vector
    geometry_msgs::Twist rectified_twist;
    rectified_twist.linear.x = twist.linear.x + model->anomaly_a_.back()(0);
    rectified_twist.linear.y = twist.linear.y;
    rectified_twist.linear.z = twist.linear.z;
    rectified_twist.angular.x = twist.angular.x;
    rectified_twist.angular.y = twist.angular.y;
    rectified_twist.angular.z = twist.angular.z + model->anomaly_a_.back()(1);

    /// State Prediction
    Eigen::Vector3d priori_est = model->pred(ts, rectified_twist);
//    cout<<"priori_est "<< priori_est<< endl;
    Eigen::Matrix3d A_bar = (Eigen::Matrix3d::Identity() - model->G * GMM * model->C) * model->A;
    Eigen::Matrix3d Q_bar = (Eigen::Matrix3d::Identity() - model->G * GMM * model->C) * process->Q * (Eigen::Matrix3d::Identity() - model->G * GMM * model->C).transpose() + model->G * GMM * sensor->R * GMM.transpose() * model->G.transpose();
//    cout<<"A_bar and Q_bar is "<< A_bar << Q_bar << endl;

    Eigen::Matrix3d P_priori_est = A_bar * model->Pe_.back() * A_bar.transpose() + Q_bar;// * A_bar.transpose() + Q_bar;
    /// State Estimation
    Eigen::Matrix3d R = model->C * P_priori_est * model->C + sensor->R +model->C * model->G * GMM * sensor->R + sensor->R * GMM.transpose() * model->G.transpose() * model->C.transpose();
    Eigen::Matrix3d L = (model->C * P_priori_est + sensor->R * GMM.transpose() * model->G.transpose()) * R.inverse();
    model->posteriori_stateEst_ = priori_est + L * (sensor->measurement - model->C * priori_est);
    cout<<"posteriori_stateEst_ "<< model->posteriori_stateEst_<< endl;
    model->it_pe_ = model->Pe_.erase(model->it_pe_);
    model->Pe_.emplace_back((Eigen::Matrix3d::Identity() - L * model->C) * P_priori_est * (Eigen::Matrix3d::Identity() - L * model->C).transpose() + L * sensor->R * L.transpose() - (Eigen::Matrix3d::Identity() - L * model->C) * model->G * GMM * sensor->R * L.transpose() - L * sensor->R * GMM.transpose() * model->G.transpose() * (Eigen::Matrix3d::Identity() - L * model->C));
//    cout<<P_priori_est<<endl;
//    int n = 0;
//    double prod = 1.0;
//    Eigen::EigenSolver<Eigen::Matrix3d> es(P_priori_est);
//    for (int i = 0; i < es.pseudoEigenvalueMatrix().rows(); ++i) {
//        if (es.pseudoEigenvalueMatrix()(i,i) > 0.000000000000000000000001){
//            prod *= es.pseudoEigenvalueMatrix()(i,i);
//            n++;
//        }
//    }
//    cout<<"xianyan det is"<<prod<<";rank is "<<n<<endl;
    /// Anomaly Vector Estimation
    if(model->mode == 1){
        model->it_s_ = model->anomaly_s_.erase(model->it_s_);
        model->anomaly_s_.emplace_back(sensor_3d_->measurement - model->C * model->posteriori_stateEst_);
        model->it_ps_ = model->Ps_.erase(model->it_ps_);
        model->Ps_.emplace_back(model->C * model->Pe_.back() * model->C.transpose() + sensor_3d_->R);
    } else{
        model->it_s_ = model->anomaly_s_.erase(model->it_s_);
        model->anomaly_s_.emplace_back(sensor_2d_->measurement - model->C * model->posteriori_stateEst_);
        model->it_ps_ = model->Ps_.erase(model->it_ps_);
        model->Ps_.emplace_back(model->C * model->Pe_.back() * model->C.transpose() + sensor_2d_->R);
    }
    cout<<"anomaly_a_"<<model->anomaly_a_.back()<<endl;
    cout<<"anomaly_s_"<<model->anomaly_s_.back()<<endl;
    /// Mode Selection
    model->residual = sensor->measurement - model->C * priori_est;
    cout<<"residual"<<model->residual<<endl;
    Eigen::Matrix3d P_bar = model->C * P_priori_est * model->C.transpose() + sensor->R - model->C * model->G * GMM * sensor->R - sensor->R * GMM.transpose() * model->G.transpose() * model->C.transpose();
//    cout<< P_bar<< endl;

    model->a = model->residual.transpose() * pinv(P_bar) * model->residual;
//    cout<<pinv(P_bar)<<endl;
    vector<double>tmp = pdet_3d(P_bar);
    double N_dist = (1 / (pow(2 * M_PI , tmp[0] / 2) * sqrt(tmp[1]))) * exp(- model->a / 2);
//    ROS_INFO("a is %f, exp is %f",model->a,  exp(- model->a / 2));

    ROS_INFO("mode no. [%d]:normal dist is %f, n is %f",model->mode, N_dist, tmp[0]);
    model->it_mu_ = model->mu_.erase(model->it_mu_);
    model->mu_.emplace_back(model->mu_.back() * N_dist);

    model->residual = Eigen::Vector3d(fabs(model->residual(0)),fabs(model->residual(1)),fabs(model->residual(2)));
}
///
/// \param twist input
void roboADS::controlCB(const geometry_msgs::Twist::ConstPtr& msg){
    ros::NodeHandle nh;
    if(mode_switch_ == 1){
        double s11, s12, s13;///Must decide whether to correct before nuise start. or a1 error will accumulate
        nh.getParam("/s11",s11);nh.getParam("/s12",s12);nh.getParam("/s13",s13);
        nh.setParam("/s11",model_2d_->anomaly_s_.back()(0));
        nh.setParam("/s12",model_2d_->anomaly_s_.back()(1));
        nh.setParam("/s13",model_2d_->anomaly_s_.back()(2));
//            nh.setParam("/s13",model_2d_->anomaly_s_.at(model_2d_->anomaly_s_.size()-2)(2)+model_2d_->anomaly_s_.back()(2));
        double a1, a2;///Must decide whether to correct before nuise start. or a1 error will accumulate
        nh.getParam("/a1",a1);nh.getParam("/a2",a2);
        ROS_INFO("a1a2, %f, %f",a1,a2);
//            nh.setParam("/a1",model_2d_->anomaly_a_.at(model_2d_->anomaly_a_.size()-2)(0)+model_2d_->anomaly_a_.back()(0));
        nh.setParam("/a1",model_2d_->anomaly_a_.back()(0));
        nh.setParam("/a2",model_2d_->anomaly_a_.back()(1));
    }else{
        double s21, s22, s23;///Must decide whether to correct before nuise start. or a1 error will accumulate
        nh.getParam("/s21",s21);nh.getParam("/s22",s22);nh.getParam("/s23",s23);
        nh.setParam("/s21",model_3d_->anomaly_s_.back()(0));
        nh.setParam("/s22",model_3d_->anomaly_s_.back()(1));
        nh.setParam("/s23",model_3d_->anomaly_s_.back()(2));
        double a1, a2;
        nh.getParam("/a1",a1);nh.getParam("/a2",a2);
        ROS_INFO("a1a2, %f, %f",a1,a2);
        nh.setParam("/a1",model_3d_->anomaly_a_.back()(0));
        nh.setParam("/a2",model_3d_->anomaly_a_.back()(1));
    }

//    ROS_INFO("enter callback");
    geometry_msgs::Twist twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    /// Lock Thread
//    boost::mutex::scoped_lock lock(nuise_mutex_);

    /// Measurement
    ros::Rate r(10);
    while (true){
        try{
            listener_.lookupTransform("camera_init", "velodyne",
                                      ros::Time(0), transform_3d_);
        }
        catch (tf::TransformException &ex) {
            //ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
        }

        try{
            listener_.lookupTransform("velodyne_init", "velodyne",
                                      ros::Time(0), transform_2d_);
            break;
        }
        catch (tf::TransformException &ex) {
            ROS_INFO("fuckedup");
            //ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
        }
        r.sleep();
    }
    double ts = ros::Time::now().toSec() - previous_t_;
    previous_t_ = ros::Time::now().toSec();
//    if(mode_switch_ == 1){
//        sensor_2d_->measurement = Eigen::Vector3d(transform_2d_.getOrigin().getX(), transform_2d_.getOrigin().getY(), tf::getYaw(transform_2d_.getRotation()));
//        sensor_3d_->measurement = Eigen::Vector3d(transform_3d_.getOrigin().getX()-model_2d_->anomaly_s_.back()(0), transform_3d_.getOrigin().getY()-model_2d_->anomaly_a_.back()(1), tf::getYaw(transform_3d_.getRotation())-model_2d_->anomaly_s_.back()(2));
//        model_3d_->posteriori_stateEst_ = model_2d_->posteriori_stateEst_;
//    }else{
//        sensor_2d_->measurement = Eigen::Vector3d(transform_2d_.getOrigin().getX()-model_3d_->anomaly_s_.back()(0), transform_2d_.getOrigin().getY()-model_3d_->anomaly_s_.back()(1), tf::getYaw(transform_2d_.getRotation())-model_3d_->anomaly_s_.back()(2));
//        sensor_3d_->measurement = Eigen::Vector3d(transform_3d_.getOrigin().getX(), transform_3d_.getOrigin().getY(), tf::getYaw(transform_3d_.getRotation()));
//        model_2d_->posteriori_stateEst_ = model_3d_->posteriori_stateEst_;
//    }
    sensor_2d_->measurement = Eigen::Vector3d(transform_2d_.getOrigin().getX(), transform_2d_.getOrigin().getY(), tf::getYaw(transform_2d_.getRotation()));
    sensor_3d_->measurement = Eigen::Vector3d(transform_3d_.getOrigin().getX(), transform_3d_.getOrigin().getY(), tf::getYaw(transform_3d_.getRotation()));

    /// Update Model
    process_2d_->setQ(sensor_2d_->measurement(2));
    process_3d_->setQ(sensor_3d_->measurement(2));

    /// Linearize
//    cout<<"houyan"<<model_2d_->posteriori_stateEst_<<endl;
//    cout<<"ts"<<ts<<endl;

    model_2d_->linearize(ts, twist);
    model_3d_->linearize(ts, twist);
//    if(mode_switch_ == 1){
//        shared_ptr<Sensor> com = make_shared<Sensor>(*sensor_3d_);
//        com->measurement = com->measurement - model_2d_->anomaly_s_.back();
//        NUISE(ts, twist, model_2d_, sensor_2d_, process_2d_);
//        NUISE(ts, twist, model_3d_, com, process_3d_);
//    }else{
//        shared_ptr<Sensor> com = make_shared<Sensor>(*sensor_2d_);
//        com->measurement = com->measurement - model_3d_->anomaly_s_.back();
//        NUISE(ts, twist, model_2d_, com, process_2d_);
//        NUISE(ts, twist, model_3d_, sensor_3d_, process_3d_);
//    }
    Eigen::Vector3d re = model_2d_->residual - model_3d_->residual;
    if(model_2d_->a > 10 && model_3d_->a > 10){
        model_3d_->posteriori_stateEst_ = sensor_3d_->measurement;
        model_2d_->posteriori_stateEst_ = sensor_2d_->measurement;
    }
    /// In order to quickly converge, every 2 sec make the wrong sensor posteriori estimation equals the other's.
    /// Once wrong happens, residual will go large, so does the difference. So only when mistakes take place(residual > 0.3), manpower will engage
    /// If still in error, residual is still large, so this procedure won't affect correct mode choice.
    if(fabs(re(0))+ fabs(re(1))+ fabs(re(2))>0.5){
        mode_trans = ros::Time::now().toSec();
        model_2d_->mu_.back()=0.5;
        model_3d_->mu_.back()=0.5;

//        ROS_INFO("%f",ros::Time::now().toSec() - mode_trans);
        if(mode_switch_ == 1){
            ROS_WARN("mode 1, 3d got initialized by 2d");
            model_3d_->posteriori_stateEst_ = model_2d_->posteriori_stateEst_;
        }else{
            ROS_WARN("mode 2, 2d got initialized by 3d");
            model_2d_->posteriori_stateEst_ = model_3d_->posteriori_stateEst_;
        }
    }else ROS_WARN("%f",ros::Time::now().toSec() - mode_trans);
    ///residual is small means estimator recognize the posteriori value(measurement - pre is small)
    ///only in this condition and mu > 0.95, which means hard to converge, we will force posteriori value to be measurement value
    if(ros::Time::now().toSec() - mode_trans > 5.0 && fabs(re(0))+ fabs(re(1))+ fabs(re(2))<0.5 && (model_2d_->mu_.back()>0.99 || model_3d_->mu_.back()>0.99)){
        mode_trans = ros::Time::now().toSec();
//        model_2d_->mu_.back()=0.5;
//        model_3d_->mu_.back()=0.5;
        if(mode_switch_ == 1){
            ROS_WARN("mode 1, 3d got initialized by 2d");
            model_3d_->posteriori_stateEst_ = sensor_3d_->measurement;
        }else{
            ROS_WARN("mode 2, 2d got initialized by 3d");
            model_2d_->posteriori_stateEst_ = sensor_2d_->measurement;
        }
    }

    NUISE(ts, twist, model_2d_, sensor_2d_, process_2d_);
    NUISE(ts, twist, model_3d_, sensor_3d_, process_3d_);

    double m1 = model_2d_->mu_.back();
    double m2 = model_3d_->mu_.back();
    ROS_INFO("%f",m1);
    ROS_INFO("%f",m2);

    double threshold = 0.0001;
//    ROS_INFO("2d %f, 3d %f", model_2d_->mu_.back(), model_3d_->mu_.back());

    if (m1 > threshold){
        if (m2 > threshold){
            model_2d_->mu_.back() = m1 / (m1 + m2);
            model_3d_->mu_.back() = m2 / (m1 + m2);
//            ROS_INFO("111 2d %f, 3d %f", model_2d_->mu_.back(), model_3d_->mu_.back());
        }
        else{
            m2 = threshold;
            model_2d_->mu_.back() = m1 / (m1 + m2);
            model_3d_->mu_.back() = m2 / (m1 + m2);
//            ROS_INFO("222 2d %f, 3d %f", model_2d_->mu_.back(), model_3d_->mu_.back());
        }
    }else{
        m1 = threshold;
        if (m2 > threshold){
            model_2d_->mu_.back() = m1 / (m1 + m2);
            model_3d_->mu_.back() = m2 / (m1 + m2);
//            ROS_INFO("333 2d %f, 3d %f", model_2d_->mu_.back(), model_3d_->mu_.back());
        }
        else{
            m2 = threshold;
            model_2d_->mu_.back() = m1 / (m1 + m2);
            model_3d_->mu_.back() = m2 / (m1 + m2);
//            ROS_INFO("444 2d %f, 3d %f", model_2d_->mu_.back(), model_3d_->mu_.back());
        }
    }

    if (model_2d_->mu_.back() > model_3d_->mu_.back()){
        if(first){
            first = false;
            mode_switch_ = 1;
        }else if(model_3d_->a < 100) {
            ROS_INFO("Mode 1 , %f", model_2d_->mu_.back());
            chi_test_a(model_2d_->anomaly_a_, model_2d_->Pa_);
            chi_test_s(model_2d_->anomaly_s_, model_2d_->Ps_);
            mode_switch_ = 1;
        }else{
            ROS_ERROR("Mode 1, 2 Probably Wrong");
            chi_test_a(model_2d_->anomaly_a_, model_2d_->Pa_);
            chi_test_s(model_2d_->anomaly_s_, model_2d_->Ps_);
            mode_switch_ = 1;
        }
    }else {
        if(first){
            first = false;
            mode_switch_ = 2;
        }else if(model_2d_->a < 100){
            ROS_INFO("Mode 2 , %f", model_3d_->mu_.back());
            chi_test_a(model_3d_->anomaly_a_,model_3d_->Pa_);
            chi_test_s(model_3d_->anomaly_s_,model_3d_->Ps_);
            mode_switch_ = 2;
    }else{
        ROS_ERROR("Mode 1, 2 Probably Wrong");
        chi_test_a(model_2d_->anomaly_a_, model_2d_->Pa_);
        chi_test_s(model_2d_->anomaly_s_, model_2d_->Ps_);
        mode_switch_ = 1;
    }
    }
///    ROS_INFO("Actuator Test");
///    ROS_INFO("Sensor Test");
}

void roboADS::getTransform(){
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