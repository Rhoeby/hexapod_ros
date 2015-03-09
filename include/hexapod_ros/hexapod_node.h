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

#ifndef _HEXAPOD_NODE_
#define _HEXAPOD_NODE_

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TwistStamped.h"
#include "hexapod_robot.h"
#include "hexapod_phone.h"
#include "hexapod_console.h"

#define USE_HEXAPOD_NODE 1

namespace hexapod_ros
{

using std::string;
using geometry_msgs::Twist;

class HexapodNode
{
public:
  HexapodNode();
  HexapodNode(const bool use_phone, const bool use_sample_rejection);
  virtual ~HexapodNode();
  bool getQuit();

private:
  bool use_phone_;
  bool use_sample_rejection_;
  std::string robot_port_name_;
  std::string phone_port_name_;

  ros::NodeHandle node_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher laser_pub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;
  
  hexapod_ros::Robot robot_;
  hexapod_ros::Console console_;
  hexapod_ros::Phone *p_phone_;
  boost::thread thread_;

  double scan_data_[1024];
  bool quit_;

  void processThread();
  void process();
  int8_t processConsole();

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist);

  void imuPublish();
  void scanPublish();
  void odomPublish();
};

} // namespace hexapod_ros

#endif // _HEXAPOD_NODE_
