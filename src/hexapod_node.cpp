/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, John Jordan
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rhoeby nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include "ros/ros.h"
#include "hexapod_node.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

namespace hexapod_ros
{

/*----------------------------------------------------------
 * Constructor()
 *--------------------------------------------------------*/

HexapodNode::HexapodNode(const bool use_phone, const bool use_sample_rejection) : 
  use_phone_(use_phone), 
  use_sample_rejection_(use_sample_rejection),
  quit_(false)
{
  ros::NodeHandle private_node_handle_("~");
  
  ROS_INFO("Hi from HexapodNode!\r");

  private_node_handle_.param("robot_port_name", robot_port_name_, std::string("/dev/rfcomm0"));
  private_node_handle_.param("phone_port_name", phone_port_name_, std::string("/dev/ttyS1" /* rfcomm1 */));
  
  cmd_vel_sub_ = node_.subscribe("cmd_vel", 100, &HexapodNode::cmdVelCallback, this);
  
  imu_pub_ = node_.advertise<sensor_msgs::Imu>("imu_data", 50);
  laser_pub_ = node_.advertise<sensor_msgs::LaserScan>("laser_data", 20);
  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 50);
  
  robot_.open(robot_port_name_);

  if (use_phone_)
  {
    p_phone_ = new hexapod_ros::Phone();
    // TODO: may need to move this 'open()' call to outside the c'tor
    p_phone_->open(phone_port_name_);
    ROS_DEBUG("Using phone\r");
  }

  if (use_sample_rejection_)
  {
    robot_.writeCmd('R');
    ROS_DEBUG("Using sample rejection\r");
  }

  thread_ = boost::thread(&HexapodNode::processThread, this);
}

/*----------------------------------------------------------
 * Destructor()
 *--------------------------------------------------------*/

HexapodNode::~HexapodNode()
{
  robot_.close();

  thread_.interrupt();
  thread_.join();

  if (use_phone_)
  {
    delete p_phone_;
  }
}

/*------------------------------------------------------------
 * processThread() - thread method
 *----------------------------------------------------------*/

void HexapodNode::processThread()
{
  while (ros::ok()) 
  {
    if (processConsole() == 'q') break;
    
    process();
  
    ros::Duration(0.02).sleep();
  }

  quit_ = true;
}

/*----------------------------------------------------------
 * process()
 *--------------------------------------------------------*/

void HexapodNode::process()
{
  hexapod_ros::RobotStatus_t robot_status;
  
  odomPublish();

  robot_.readMsg();
  robot_.getStatus(&robot_status);
  if (robot_status.scan_data_available_)
  {
    scanPublish();
  }

  if (use_phone_)
  {
    p_phone_->readMsg();
    if (p_phone_->msgWaiting())
    {
      imuPublish();
    }
  }
}

/*----------------------------------------------------------
 * processConsole()
 *--------------------------------------------------------*/

int8_t HexapodNode::processConsole(void)
{
  int8_t kbd_ch = 0;

  if (console_.kbHit())
  {
    kbd_ch = console_.getCh();
    ROS_DEBUG("Got kbd char: %c\r", kbd_ch);
    switch (kbd_ch)
    {
    default:
      break;
    }

    robot_.writeCmd(kbd_ch);
  }

  return kbd_ch;
}

/*------------------------------------------------------------
 * getQuit()
 *----------------------------------------------------------*/

bool HexapodNode::getQuit()
{
  return quit_;
}

/*----------------------------------------------------------
 * cmdVelCallback()
 *--------------------------------------------------------*/

void HexapodNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  robot_.setVelocities(twist->linear.x, twist->linear.y, twist->angular.z);

  ROS_DEBUG("twist linear.x: %f, y: %f, angular.z: %f\r", 
            twist->linear.x, twist->linear.y, twist->angular.z);  
}

/*----------------------------------------------------------
 * imuPublish()
 *--------------------------------------------------------*/

void HexapodNode::imuPublish(void)
{
  ros::Time current_time = ros::Time::now();
  static ros::Time last_time = current_time;
  double dt = (current_time - last_time).toSec();
  sensor_msgs::Imu imu;
  static double z = 0;

  imu.header.stamp = current_time;
  imu.header.frame_id = "base_dms"; // borrow this for now

  z += p_phone_->getAngularZ() * dt;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(z);
  imu.orientation = q;
  for (int32_t i=0; i<9; i++)
  {
    imu.orientation_covariance[i] = 0;
  }

  imu.orientation_covariance[0] = 0.01; 
  imu.orientation_covariance[4] = 0.01; 
  imu.orientation_covariance[8] = 0.1; 

  imu_pub_.publish(imu);

  last_time = current_time;
}


/*----------------------------------------------------------
 * scanPublish()
 *--------------------------------------------------------*/

void HexapodNode::scanPublish(void)
{
  uint32_t i;
  ros::Time current_time = ros::Time::now();
  sensor_msgs::LaserScan scan;
  hexapod_ros::RobotStatus_t robot_status;

  scan.header.frame_id = "base_laser";

  robot_.getStatus(&robot_status);
  scan.scan_time = robot_status.scan_period_ / 1000.0;
  scan.range_min = 0.2;
  scan.range_max = 3.6576;

  scan.header.stamp = current_time - ros::Duration(scan.scan_time);
  scan.angle_max = 3.142;
  scan.angle_min = -scan.angle_max;

  scan.time_increment = scan.scan_time / robot_status.samples_per_scan_;
  scan.angle_increment = 2 * 3.142 / robot_status.samples_per_scan_;
  scan.ranges.resize(robot_status.samples_per_scan_);

  robot_.getScanData(scan_data_);
  for (i=0; i<robot_status.samples_per_scan_; i++)
  {
    scan.ranges[i] = scan_data_[i];
  }

  laser_pub_.publish(scan);
}

/*----------------------------------------------------------
 * odomPublish()
 *--------------------------------------------------------*/

void HexapodNode::odomPublish(void)
{
  double vx;
  double vy;
  double vth;
  static double x = 0;
  static double y = 0;
  static double th = 0;
  ros::Time current_time = ros::Time::now();
  static ros::Time last_time = current_time;

  vx = robot_.getVelocityX();
  vy = robot_.getVelocityY();
  vth = robot_.getVelocityTheta();

#if 0
  ROS_INFO("odom vx: %f, vy: %f, vth: %f\r", 
           vx,
           vy,
           vth);
#endif

  // compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  // integrate to track position
  x += delta_x;
  y += delta_y;
  th += delta_th;

  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  if (!use_phone_)
  {
    //send the odom-->base_footprint transform: only if NOT using robot_pose_ekf!
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom_combined";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);
#if 0
    ROS_INFO("odom x: %f, y: %f, z-rot: %f\r", 
             x,
             y,
             th);
#endif
  }

  // publish the odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom_combined";
  odom.child_frame_id = "base_footprint";

  // set the position and orientation
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // TODO: these were repurposed from nxt_ros:: base_odometry.py, check the validity
  odom.pose.covariance[0] = 0.00001;    // x - are all these correct?
  odom.pose.covariance[7] = 0.00001;    // y
  odom.pose.covariance[14] = 10;    // z (set high because robot does not move up/down much?)
  odom.pose.covariance[21] = 1;   // rot x
  odom.pose.covariance[28] = 1;   // rot y
  odom.pose.covariance[35] = 1;   // rot z

  // set the velocity
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  last_time = current_time;

  // publish the odom message
  odom_pub_.publish(odom);
}

} // namespace hexapod_ros

