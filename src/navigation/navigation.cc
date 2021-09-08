//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    arc_curvature_(0),
    arc_distance_(0),
    arc_vel_(0),
    act_latency_(0),
    sens_latency_(0),
    timer_1_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  previous_vel_.setZero();
  previous_curv_.setZero();
  curv_set_ << -10,-5,-2,-1,-0.5,-0.2,-0.1,-0.05,0,0.05,0.1,0.2,0.5,1,2,5,10;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_complete_ = false;
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
  
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  timer_1_++;

  /*
  if (timer_1_ == 1){
    double c;
    double d;
    
    std::cout << "Enter curvature: ";
    std::cin >> c;
    std::cout << "Enter distance: ";
    std::cin >> d;
    std::cout << "\n";
    

    #if 0
    double x;
    double y;
    double eucl_dist;
    std::cout << "Enter x: ";
    std::cin >> x;
    std::cout << "Enter y: ";
    std::cin >> y;
    std::cout << "\n";
    c = 2*y/(pow(x,2) + pow(y,2));
    eucl_dist = sqrt(pow(x,2) + pow(y,2));
    d = acos(1-(pow(c,2)/2)*eucl_dist)/c;
    #endif

    arc_curvature_ = c;
    arc_distance_ = d;
  }
  */
  
  if (timer_1_ < 5) return;

  Eigen::Matrix2f R;
  R(0,0) = cos(-odom_start_angle_);
  R(0,1) = -sin(-odom_start_angle_);
  R(1,0) = sin(-odom_start_angle_);
  R(1,1) = cos(-odom_start_angle_);

  Eigen::Vector2f fixed_odom_loc_ = R*(odom_loc_-odom_start_loc_);
  //std::cout << "\n" << fixed_odom_loc_.transpose() << "\n";

  UpdateLocation(fixed_odom_loc_,odom_angle_-odom_start_angle_);
  //std::cout << odom_start_angle_ << "  ----  " << odom_angle_ << "\n";
  //std::cout << odom_start_loc_.transpose() << "  ----  " << odom_loc_.transpose() << "\n";

  if (nav_complete_) return;

  // Compute arc to use
  Eigen::Vector2f rel_nav_loc_ = nav_goal_loc_-robot_loc_;
  //Eigen::Matrix2f R;
  R(0,0) = cos(-robot_angle_);
  R(0,1) = -sin(-robot_angle_);
  R(1,0) = sin(-robot_angle_);
  R(1,1) = cos(-robot_angle_);

  //std::cout << rel_nav_loc_.transpose() << "\n";

  rel_nav_loc_ = R*rel_nav_loc_;

  //std::cout << nav_goal_loc_.transpose() << "\n" << robot_loc_.transpose() << "\n";

  double eucl_dist = sqrt(pow(rel_nav_loc_(0),2) + pow(rel_nav_loc_(1),2));
  
  if (eucl_dist < 0.05){
    nav_complete_ = true;
    return;
  }

  float del_theta = atan2(rel_nav_loc_(1),rel_nav_loc_(0));

  /* // Code for determining curvature and distance without discrete grid
  double c = 10*del_theta;
  c = std::min(c,5.0);
  
  double d;
  if (abs(c) < 0.05){
    d = eucl_dist;
  }else{
    d = del_theta/c;
  }
  //d = std::min(d,eucl_dist);
  */

  int num_curves = curv_set_.rows();
  Eigen::VectorXf dist(num_curves);
  Eigen::VectorXf cost(num_curves);

  dist = abs(del_theta)/curv_set_.array();
  dist = dist.array().min(eucl_dist);

  cost = pow(curv_set_.array() - 5*del_theta,2);

  int min_ind = 0;
  for (int i=1; i<num_curves; i++){
    if (cost(i) < cost(min_ind)){
      min_ind = i;
    }
  }
  double c = curv_set_(min_ind);
  double d = eucl_dist;//dist(min_ind);
  /*
  if (abs(c) < 0.05){
    d = eucl_dist;
  }else{
    d = del_theta/c;
  }
  */

  arc_curvature_ = c;
  arc_distance_ = d;

  // std::cout << d << "\n\n";

  float odom_vel = pow(pow(robot_vel_(0),2) + pow(robot_vel_(1),2),0.5);

  /*
  // Delay calibration
  if (timer_1_ < 300){
      // Send message
      drive_msg_.curvature = 0;
      drive_msg_.velocity = sin(double(timer_1_)/30);

      // Add timestamps to all messages.
      local_viz_msg_.header.stamp = ros::Time::now();
      global_viz_msg_.header.stamp = ros::Time::now();
      drive_msg_.header.stamp = ros::Time::now();
      // Publish messages.
      viz_pub_.publish(local_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      drive_pub_.publish(drive_msg_);
      
      std::cout << odom_vel << "\n";

      return;
  } else if(timer_1_ == 100){

  }
  */

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:

  // 1-D TOC
  
  // Determine whether to be at max speed or min speed

  // std::cout << "Odom: " << odom_loc_.transpose() << "\n";

  //std::cout << pow(robot_vel_(0),2) + pow(robot_vel_(1),2);
  //std::cout << "\n";

  // Sensor data capture, now forward-predict
  float total_latency = sens_latency_+act_latency_;
  float est_cur_vel = 0;
  float est_prev_vel = odom_vel;
  float est_del_arc_distance = 0;
  float est_total_arc_distance = 0;
  //float est_cur_arc_distance = 0;
  //float est_prev_arc_distance = arc_distance_;
  float est_cur_angle = 0;
  float est_prev_angle = robot_angle_;
  Eigen::Vector2f est_cur_loc;
  est_cur_loc(0) = 0;
  est_cur_loc(1) = 0;
  Eigen::Vector2f est_prev_loc;
  est_prev_loc = robot_loc_;

  // Forward-Predict
  for(int i=total_latency-1;i>=act_latency_-1;i--){

    if(previous_vel_(i) > est_prev_vel){
      // accelerating
      est_cur_vel = std::min(std::min(previous_vel_(i),MAX_VEL), est_prev_vel + MAX_ACCEL/20);
    }else{
      // decelerating
      est_cur_vel = std::max(std::max(previous_vel_(i),-MAX_VEL), est_prev_vel - MAX_DECEL/20);
    }

    est_del_arc_distance = est_cur_vel/20;
    est_total_arc_distance = est_total_arc_distance + est_del_arc_distance;
    est_cur_angle = est_prev_angle + est_del_arc_distance*previous_curv_(i);

    // std::cout << est_cur_vel << "  --  ";

    // Approximate small angles as straight
    if (abs(previous_curv_(i)) < 1000){
      est_cur_loc(0) = est_prev_loc(0) + cos(est_cur_angle)*est_del_arc_distance;
      est_cur_loc(1) = est_prev_loc(1) + sin(est_cur_angle)*est_del_arc_distance;
    }//else{
      //est_cur_loc(0) = est_prev_loc(0) + sin(est_del_arc_distance*previous_curv_(i))/previous_curv_(i);
      //est_cur_loc(1) = est_prev_loc(1) + (1 - cos(est_del_arc_distance*previous_curv_(i)))/previous_curv_(i);
    //}

    // Update
    est_prev_vel = est_cur_vel;
    est_prev_loc = est_cur_loc;
    est_prev_angle = est_cur_angle;

  }

  // std::cout << "Est change in pose: " << est_cur_angle << "  -  " << est_cur_loc.transpose() << "\n";

  arc_distance_ = arc_distance_ - est_total_arc_distance;

  // 1-D TOC
  float stopping_distance = pow(est_cur_vel,2)/(2*MAX_DECEL);
  float target_vel = 0;

  if(arc_distance_ > stopping_distance){
    target_vel = 100;
  }else{
    target_vel = 0;
  }

  float target_curvature = arc_curvature_;

  // std::cout << "Angle: " << odom_angle_ << "\n";

  //std::cout << point_cloud_[0] << "\n";

  int pc_len = point_cloud_.size();
  for (int i=0; i<pc_len; i++){
    //visualization::DrawPoint(point_cloud_[i],0xde0000,local_viz_msg_);
  }

  // Send message
  drive_msg_.curvature = target_curvature;
  drive_msg_.velocity = target_vel;

  // shift previous values
  for(int i=8;i>=0;i--){
    previous_vel_(i+1) = previous_vel_(i);
    previous_curv_(i+1) = previous_curv_(i);
  }
  previous_vel_(0) = drive_msg_.velocity;
  previous_curv_(0) = drive_msg_.curvature;

  //std::cout << previous_vel_.transpose();
  //std::cout << "\n";
  //std::cout << std::flush;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  // std::cout << "\n\n";
}

}  // namespace navigation
