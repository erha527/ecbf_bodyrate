/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/jerk_tracking_control.h"
#include "geometric_controller/nonlinear_attitude_control.h"
#include "geometric_controller/nonlinear_geometric_control.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE) {
  referenceSub_ =
      nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  imu_sub_ = 
     nh_.subscribe("/mavros/imu/data", 1, &geometricCtrl::imuCallback, this, ros::TransportHints().tcpNoDelay());
  
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.005), &geometricCtrl::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this);  // Define timer for constant loop rate
  //tiaoshi2Pub_ = nh_.advertise<geometry_msgs::PoseStamped>("filter/data", 1);
 // tiaoshiPub_ = nh_.advertise<geometry_msgs::PoseStamped>("function/cbf", 1);
 tiaoshiPub_ = nh_.advertise<controller_msgs::RecordData>("record/data", 1);
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);

  double dx, dy, dz;
  nh_private_.param<double>("drag_dx", dx, 0.0);
  nh_private_.param<double>("drag_dy", dy, 0.0);
  nh_private_.param<double>("drag_dz", dz, 0.0);
  D_ << dx, dy, dz;

  double attctrl_tau;
  nh_private_.param<double>("attctrl_constant", attctrl_tau, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<int>("posehistory_window", posehistory_window_,10000);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);
////////////////////////////////////ECBF PARAM/////////////////
  nh_private_.param<double>("Komega_qp_0", Komega_qp_0, 1.5);
  nh_private_.param<double>("Komega_qp_1", Komega_qp_1, 1.5);
  nh_private_.param<double>("Komega_qp_2", Komega_qp_2, 1.5);
  nh_private_.param<double>("Komega_qp_3", Komega_qp_3, 1.5);
  nh_private_.param<double>("Kthrust_qp_0", Kthrust_qp_0, 1.5);
  nh_private_.param<double>("Kthrust_qp_1", Kthrust_qp_1, 1.5);
  nh_private_.param<bool>("is_Ecbf",  is_Ecbf, false);
  nh_private_.param<double>("safe_d",  safe_d, 1.0);
  //////////////////////////////////////////////////////
  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  flag_h = false;
  bool jerk_enabled = false;
  if (!jerk_enabled) {
    if (ctrl_mode_ == ERROR_GEOMETRIC) {
      controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    } else {
      controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    }
  } else {
    controller_ = std::make_shared<JerkTrackingControl>();
  }
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}
void geometricCtrl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sensor_msgs::Imu imu_data_ = *msg;
    mavAcc_ = toEigen(msg.linear_acceleration);
    // mavAcc_(0) = imu_data_.linear_acceleration.y;
    // mavAcc_(1) = -  imu_data_.linear_acceleration.x;
    // mavAcc_(2) = imu_data_.linear_acceleration.z;
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
  
  
  targetPos_ = toEigen(msg.position)+init;
  targetVel_ = toEigen(msg.velocity);

  if (msg.type_mask == 1) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 2) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;
  
  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    init = toEigen(msg.pose.position);
    ob_x = init(0);
    ob_y = init(1);
    ob_z = init(2);
    //init<< ob_x,ob_y,ob_z;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  
  mavPos_ = toEigen(msg.pose.position);
  record.position.x = mavPos_(0);
  record.position.y = mavPos_(1);
  record.position.z = mavPos_(2);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
   record.velocity.x = mavVel_(0);
  record.velocity.y = mavVel_(1);
  record.velocity.z = mavVel_(2);
  time = ros::Time::now().toSec();
  // double dt  = time - time_;
  // time_ =time;
  //  mavAcc_ =( mavVel_ - mavVel_d) / dt;
  //  mavVel_d = mavVel_;
  mavRate_ = toEigen(msg.twist.angular);
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  node_state = LANDING;
  return true;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = MISSION_EXECUTION;
      break;

    case MISSION_EXECUTION: {
      Eigen::Vector3d desired_acc;
      if (feedthrough_enable_) {
        desired_acc = targetAcc_;
      } else {
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
      }
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      //////////ecbf_filter
      record.output.x = cmdBodyRate_(0);
      record.output.y = cmdBodyRate_(1);
      record.output.z = drone_mass * thrust_command;
       pubReferencePose(targetPos_, q_des);
      if(is_Ecbf){
        jishu_++;
        //////////////////////test after stable 
        if(jishu_ >= 4000){   
          //fagn.pose.orientation.z = 1;
          record.flag = 1;
          ecbf_thrust(cmdBodyRate_);
          ecbf_cmdbodyrate  = ecbf_compute(cmdBodyRate_);
          pubRateCommands(ecbf_cmdbodyrate, q_des);
          cout << " success" << endl;
        } else {
              record.flag = 0;
              //ecbf_cmdbodyrate  = ecbf_compute(cmdBodyRate_);
              record.output_filter.x = cmdBodyRate_(0);
              record.output_filter.y =cmdBodyRate_(1);
              record.output_filter.z =drone_mass * thrust_command;
              pubRateCommands(cmdBodyRate_, q_des);
        }
      }
      pubRateCommands(cmdBodyRate_, q_des);
      tiaoshiPub_.publish(record);
      appendPoseHistory();
      pubPoseHistory();
      break;
    }

    case LANDING: {
      geometry_msgs::PoseStamped landingmsg;
      landingmsg.header.stamp = ros::Time::now();
      landingmsg.pose = home_pose_;
      landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
      target_pose_pub_.publish(landingmsg);
      node_state = LANDED;
      ros::spinOnce();
      break;
    }
    case LANDED:
      ROS_INFO("Landed. Please set to position control and disarm.");
      cmdloop_timer_.stop();
      break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    mavros_msgs::SetMode offb_set_mode;
    arm_cmd_.request.value = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x =  max(-2.0, min(2.0 , cmd(0)));
  msg.body_rate.y =  max(-2.0, min(2.0 , cmd(1)));
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);
  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}
Eigen::Vector4d geometricCtrl::ecbf_compute(Eigen::Vector4d &cmd){
  Eigen::Vector4d ecbf_bodyrate;
  Eigen::Vector2d result_xy;
  ecbf_bodyrate(3)  =  cmd(3);ecbf_bodyrate(2)  =  cmd(2); //reserve omega_z  and  thrust
  result_xy = body_qp(mavPos_,mavVel_,mavAcc_,mavAtt_,  cmd);
  ecbf_bodyrate(0)  =  result_xy(0);
  ecbf_bodyrate(1)  =  result_xy(1);
  return ecbf_bodyrate;
}
void geometricCtrl::ecbf_thrust(Eigen::Vector4d &cmd){
  thrust_qp(mavPos_,mavVel_,mavAcc_,mavAtt_,cmd);
}
void geometricCtrl::thrust_qp(Eigen::Vector3d &mavPos_, Eigen::Vector3d &mavVel_ , 
                                                                        Eigen::Vector3d &mavAcc_,Eigen::Vector4d &mavAtt, Eigen::Vector4d &cmd){
        int maxiter=50;double Errorbound=0.001;Eigen::MatrixXd H(1,1);
        Eigen::Matrix3d rotmat;  //current rotmat
        H<<1;
rotmat = quat2RotMatrix(mavAtt_);
Eigen::Vector3d mavaccel;
Eigen::Vector2d LfH_collect , k; 
 thrust_mid = drone_mass * thrust_command;
Eigen::Vector3d zb = rotmat.col(2); 
Eigen::Vector3d yb = rotmat.col(1); 
Eigen::Vector3d xb = rotmat.col(0); 
double z = mavPos_(2);    
 double vz = mavVel_(2);   
 record.output.z = thrust_mid;
  //fagn.pose.position.z = thrust_mid;
double R33 = zb(2);
//double ob_z = 0;
double h = 1 - pow(( (z-ob_z) / safe_d ) , 4 );
double lfh = - 4 * pow( (z-ob_z) , 3 )/pow( safe_d , 4) * vz ;
double lg2h =-4 * R33/ drone_mass * pow( (z-ob_z) , 3 )/pow( safe_d , 4);
double lf2h =  4 *9.81* pow( (z-ob_z) , 3 )/pow( safe_d , 4)   - 12 * pow( (z-ob_z) , 2 )/pow( safe_d , 4) * pow( vz , 2 ) ;
 k << Kthrust_qp_0, Kthrust_qp_1;
//test.pose.position.y = h;
 LfH_collect <<  h,  lfh;
 Eigen::VectorXd b,A;
 b.resize(1);
 b(0) = k.dot(LfH_collect)+lf2h; 
 Eigen::VectorXd thrust_init  ;
 thrust_init.resize(1);
 thrust_init(0)=-thrust_mid;
 A.resize(1);
A(0) =  lg2h;
// cout << z-ob_z<<  " "  <<h << ' '<< lfh <<' '<< lf2h<< endl;
// cout  << lg2h <<' '<<  b(0) <<' '<< thrust_mid  << endl;
// cout << vz << endl;
//Eigen::VectorXd  thrust_solve = thrust_hildreth(H,  thrust_init,  A,  b, maxiter, Errorbound);
if(thrust_mid <=fabs(b(0)/A(0)))thrust_mid =thrust_mid;
else{
  thrust_mid = fabs(b(0)/A(0));
}
record.output_filter.z = thrust_mid;

//thrust_solve.resize(1);cout <<  thrust_solve(0)  << endl;
       //cmd(3) =   std::max(0.0, std::min(0.95, 0.038*(thrust_solve(0))+0.1));    
       cmd(3) =   std::max(0.0, std::min(0.95, 0.038*(thrust_mid)+0.1));                                      
}
//  ////////////////////////////////////////
 
/////////////////////////////////////

Eigen::VectorXd  geometricCtrl::thrust_hildreth(const Eigen::MatrixXd &e_, const Eigen::VectorXd &f_, const Eigen::VectorXd &m_, 
                                                const Eigen::VectorXd &b_, int &maxiter, double &Errorbound){
                Eigen::VectorXd f=f_;
    Eigen::VectorXd b=b_;
    Eigen::VectorXd m=m_;
    Eigen::MatrixXd e = e_;
    e.resize(1,1);
    b.resize(1);
    f.resize(1);
    m.resize(1);
    Eigen::VectorXd H;
    H.resize(1);
    H=m.transpose()*e.inverse()*m;
    Eigen::VectorXd K;
    Eigen::VectorXd u_ini ;
    Eigen::VectorXd lamda,temp,u,compare;
    K.resize(1);u_ini.resize(1);lamda.resize(1);temp.resize(1);u.resize(1);compare.resize(1);
    K=b+m.transpose()*e.inverse()*f;
    u_ini = -e.inverse()*f;
    //u = f;
    lamda << 0.0;temp << 0.0;
    bool flag[1],temp1[1];
    int i,j,count=0,n,flag1=1;
    n=b.rows();
    double sum,aa,bb;
    compare=m.transpose()*u_ini;
    //ROS_INFO("DIRECT_OUTPUT1");
    for(i=0;i<=0;i++){      
       if(compare(i)<=b(i)){flag[i]=true;}
    }
    if(flag[0]==true){u=u_ini;}//ROS_INFO("DIRECT_OUTPUT");}
    else{
         while (flag1)
         {
            count=count+1;
            for(i=1;i<=n;i++){
            sum=0;
            if((i-1)>=0){
              for(j=1;j<=(i-1);j++){
              sum=H(i-1,j-1)*lamda(j-1)+sum;
              //ROS_INFO("acc1.0");
              }
            }
            if ((i+1)<=n){
                for(j=i+1;j<=n;j++){
                    sum=H(i-1,j-1)*lamda(j-1)+sum;
                    //ROS_INFO("acc2.0");
                }
            }
            temp(i-1)=max((-1/H(i-1,i-1))*(sum+K(i-1)),0.0);
        
            }
            if(temp==lamda){
                temp1[0]=true;
                
            }
            //for(i=0;i<=n-1;i++){
                if(temp1[0]==false || count!=maxiter){
                    flag1=1;
                    //ROS_INFO("acc4");
                }
            //}
            if(flag1 || (temp-lamda).norm()<=Errorbound){
                lamda=temp;
                flag1=0;
                //ROS_INFO("acc3");
            }
         }
         u=-e.inverse()*(f+m*lamda(0));
        
    }
    //ROS_INFO("count =%f",count);
    return u;
}
Eigen::Vector2d geometricCtrl::body_qp(Eigen::Vector3d &mavPos_, Eigen::Vector3d &mavVel_ , 
                                                                        Eigen::Vector3d &mavAcc_,Eigen::Vector4d &mavAtt, Eigen::Vector4d &cmd){

////solver param
int maxiter=50;double Errorbound=0.001;
Eigen::Matrix3d rotmat;  //current rotmat
Eigen::Matrix2d W;            
rotmat = quat2RotMatrix(mavAtt_);
Eigen::Vector3d mavaccel;
Eigen::Vector3d LfH_collect , k; 
Eigen::Matrix2d H,  f;
Eigen::Vector3d zb = rotmat.col(2); 
Eigen::Vector3d yb = rotmat.col(1); 
Eigen::Vector3d xb = rotmat.col(0); 
double R33 = zb(2);
// double R12 = xb(1);double R11 = xb(0);
// double R22 = yb(1);double R21 = yb(0);
double R12 = yb(0);double R11 = xb(0);
double R22 = yb(1);double R21 = xb(1);
W << -R12,R11, - R22,R21 ;
//W << R21, -R11, R22,-R12;
//W << 0, -R11, R22,0;
H << 1.0,0.0,0.0,1.0;
///////////////////
 double x = mavPos_(0);     double y =  mavPos_(1);
 double vx = mavVel_(0);    double vy = mavVel_(1);
 double ax = mavAcc_(0);    double ay = mavAcc_(1);
  // vx_ = vx;     vy_ = vy ;

//double ob_x = 0,  ob_y = 0;  ///obstacle 

///////////////////Lie 
double h = 1 - pow(( (x-ob_x) / safe_d ) , 4 ) - pow(( (y-ob_y) / safe_d) , 4 );

double lfh = - 4 * pow( (x-ob_x) , 3 )/pow( safe_d , 4) * vx 
                           - 4 * pow( (y-ob_y) ,3 )/pow( safe_d , 4) * vy;

double lf2h = - 4 * pow( (x-ob_x) , 3 )/pow( safe_d , 4) * ax   
                            - 4 * pow( (y-ob_y) ,3 )/pow( safe_d , 4) * ay   
                          - 12 * pow( (x-ob_x) , 2 )/pow( safe_d , 4) * pow( vx , 2 ) 
                          - 12 * pow( (y-ob_y),2 ) / pow( safe_d , 4) * pow( vy , 2 ); 

// double lf3h = -36*pow((x-ob_x),2)/pow( safe_d , 4)*pow(ax,2) 
//                             -24*pow((x-ob_x),1)/pow( safe_d , 4)*pow(vx,3) 
//                            - 36*pow((y-ob_y),2)/pow( safe_d , 4)*pow(ay,2)
//                             -24*pow((y-ob_y),1)/pow( safe_d , 4)*pow(vy,3); 
double lf3h = -36*pow((x-ob_x),2)/pow( safe_d , 4)*ax*vx
                            -24*pow((x-ob_x),1)/pow( safe_d , 4)*pow(vx,3) 
                           - 36*pow((y-ob_y),2)/pow( safe_d , 4)*ay*vy
                            -24*pow((y-ob_y),1)/pow( safe_d , 4)*pow(vy,3); 
//////////////////////////////////Lgh
Eigen::Vector2d aita,  A,  lg2h;
//double thrust = drone_mass * thrust_command;
double thrust = thrust_mid;
//aita << 4 * R33* thrust / drone_mass * pow( (x-ob_x) , 3 )/pow( safe_d , 4) ,  4 * R33 * thrust / drone_mass * pow( (y-ob_y) , 3 )/pow( safe_d , 4);
aita << 4 * thrust / drone_mass * pow( (x-ob_x) , 3 )/pow( safe_d , 4) ,  4 * thrust / drone_mass * pow( (y-ob_y) , 3 )/pow( safe_d , 4);


//lg2h =  aita.transpose() * W.inverse();
lg2h =  aita.transpose() * W;
 //////////////////////////////////////////
 A =lg2h;
 k << Komega_qp_0, Komega_qp_1, Komega_qp_2;
 
 LfH_collect <<  h,  lfh,  lf2h;
 Eigen::VectorXd b;
 b.resize(1);
 b(0) = k.dot(LfH_collect)+Komega_qp_3*lf3h;
 Eigen::Vector2d omega_init(-cmd(0), -cmd(1));
 //cout  <<b(0) <<' '<<h << ' '<< lfh <<' '<< lf2h<< ' '<< A(0) <<' '<<A(1) <<' '<< endl;
//  ////////////////////////////////////////
 Eigen::VectorXd  omega_solve = acc_hildreth(H,  omega_init,  A,  b, maxiter, Errorbound);
/////////////////////////////////////
record.output.x = cmd(0);
record.output.y = cmd(1);

record.output_filter.x = omega_solve(0);
record.output_filter.y =omega_solve(1);

omega_solve(0) = max(-1.0, min(1.0 , omega_solve(0)));
omega_solve(1) = max(-1.0, min(1.0 , omega_solve(1)));
Eigen::Vector2d   omega_ecbf = omega_solve;
return omega_solve;
}


Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  record.refpose.x = target_pos(0);
  record.refpose.y = target_pos(1);
  record.refpose.z = target_pos(2);
  record.refvel.x = target_vel(0);
  record.refvel.y = target_vel(1);
  record.refvel.z = target_vel(2);
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;
  return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);

  controller_->Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
  bodyrate_cmd.head(3) = controller_->getDesiredRate();
   thrust_command = controller_->getDesiredThrust().z();
  bodyrate_cmd(3) =
  //std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
    //                                  norm_thrust_offset_));
      std::max(0.0, std::min(0.95, 0.038*(drone_mass * thrust_command )+0.1));  // Calculate thrustcontroller_->getDesiredThrust()(3);
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large
  return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;
  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}
Eigen::VectorXd geometricCtrl::acc_hildreth(const Eigen::Matrix2d &e, const Eigen::VectorXd &f_, const Eigen::VectorXd &m_, 
                                                const Eigen::VectorXd &b_, int &maxiter, double &Errorbound)
{
    Eigen::VectorXd f=f_;
    Eigen::VectorXd b=b_;
    Eigen::VectorXd m=m_;
    b.resize(1);
    f.resize(2);
    m.resize(2);
    Eigen::VectorXd H;
    H.resize(1);
    H=m.transpose()*e.inverse()*m;
    Eigen::VectorXd K;
    Eigen::VectorXd u_ini ;
    Eigen::VectorXd lamda,temp,u,compare;
    K.resize(1);u_ini.resize(2);lamda.resize(1);temp.resize(1);u.resize(2);compare.resize(1);
    K=b+m.transpose()*e.inverse()*f;
    u_ini = -e.inverse()*f;
    //u = f;
    lamda << 0.0;temp << 0.0;
    bool flag[1],temp1[1];
    int i,j,count=0,n,flag1=1;
    n=b.rows();
    double sum,aa,bb;
    compare=m.transpose()*u_ini;
    //ROS_INFO("DIRECT_OUTPUT1");
    for(i=0;i<=0;i++){      
       if(compare(i)<=b(i)){flag[i]=true;}
    }
    if(flag[0]==true){u=u_ini;}//ROS_INFO("DIRECT_OUTPUT");}
    else{
         while (flag1)
         {
            count=count+1;
            for(i=1;i<=n;i++){
            sum=0;
            if((i-1)>=0){
              for(j=1;j<=(i-1);j++){
              sum=H(i-1,j-1)*lamda(j-1)+sum;
              //ROS_INFO("acc1.0");
              }
            }
            if ((i+1)<=n){
                for(j=i+1;j<=n;j++){
                    sum=H(i-1,j-1)*lamda(j-1)+sum;
                    //ROS_INFO("acc2.0");
                }
            }
            temp(i-1)=max((-1/H(i-1,i-1))*(sum+K(i-1)),0.0);
        
            }
            if(temp==lamda){
                temp1[0]=true;
                
            }
            //for(i=0;i<=n-1;i++){
                if(temp1[0]==false || count!=maxiter){
                    flag1=1;
                    //ROS_INFO("acc4");
                }
            //}
            if(flag1 || (temp-lamda).norm()<=Errorbound){
                lamda=temp;
                flag1=0;
                //ROS_INFO("acc3");
            }
         }
         u=-e.inverse()*(f+m*lamda(0));
        
    }
    //ROS_INFO("count =%f",count);
    return u;
}
void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
}
