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

#include <fcntl.h>

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "hexapod_phone.h"

namespace hexapod_ros
{

using std::string;

#define PHONE_MSG_MAX_SIZE 128

/*----------------------------------------------------------
 * Constructor()
 *--------------------------------------------------------*/

Phone::Phone()
{
  msg_buffer_ = new int8_t[PHONE_MSG_MAX_SIZE];

  msg_parser_state_ = 0;
  msg_byte_count_ = 0;
  field_count_ = 0;
  sensor_id_ = 0;

  msg_waiting_ = false;

  fd_ = -1;
}

/*----------------------------------------------------------
 * Destructor()
 *--------------------------------------------------------*/

Phone::~Phone()
{
  if (fd_ != -1)
  {
    ::close(fd_);
  }

  delete msg_buffer_;
}

/*----------------------------------------------------------
 * open()
 *--------------------------------------------------------*/

void Phone::open(const std::string port_name)
{
  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1)
  {
    ROS_ERROR("%s open failed!\r", port_name.c_str());
    exit(-1);
  }
  else
  {
    fcntl(fd_, F_SETFL, 0);
  }
}

/*----------------------------------------------------------
 * close() - close the port
 *--------------------------------------------------------*/

void Phone::close()
{
  int32_t rc;

  if (fd_ != -1)
  {
    rc = ::close(fd_);
    if (rc) ROS_ERROR("close failed!");
    fd_ = -1;
  }
}

/*----------------------------------------------------------
 * readMsg()
 *--------------------------------------------------------*/

void Phone::readMsg(void)
{
  int32_t loop_count = 0;

  while (dataWaiting() && loop_count<100)
  {
    loop_count++;

    switch (msg_parser_state_)
    {
    case 0:     // looking for sync byte
      read(fd_, &msg_buffer_[0], 1);
      if (msg_buffer_[0] == '>')
      {
        //ROS_INFO("Got sync byte ('>')!\r");
        field_count_ = 0;
        msg_byte_count_ = 0;
        msg_parser_state_ = 1;
      }
      break;
    case 1:     // searching for end of field
      read(fd_, &msg_buffer_[msg_byte_count_], 1);
      if (msg_buffer_[msg_byte_count_] == ',' ||
          msg_buffer_[msg_byte_count_] == '\n')
      {
        //ROS_INFO("Got field separator!\r");
        msg_buffer_[msg_byte_count_] = '\n';
        if (field_count_==0)
        {
          sensor_id_ = atoi((const char *)msg_buffer_);
          if (sensor_id_ != 1 && sensor_id_ != 4)
          {
            ROS_WARN("incorrect sensor_id_!\r");
            msg_parser_state_ = 0;
          }
        }
        else if (field_count_==1)
        {
          // not needed
        }
        else if (field_count_==2)
        {
          if (sensor_id_==1)
            twist_.linear.x = atof((const char *)msg_buffer_);
          else if (sensor_id_==4)
            twist_.angular.x = atof((const char *)msg_buffer_);
        }
        else if (field_count_==3)
        {
          if (sensor_id_==1)
            twist_.linear.y = atof((const char *)msg_buffer_);
          else if (sensor_id_==4)
            twist_.angular.y = atof((const char *)msg_buffer_);
        }
        else if (field_count_==4)
        {
          if (sensor_id_==1)
          {
            twist_.linear.z = atof((const char *)msg_buffer_);
          }
          else if (sensor_id_==4)
          {
            twist_.angular.z = atof((const char *)msg_buffer_);
            msg_waiting_ = true;
#if 0
            {
              ros::Time current_time = ros::Time::now();
              static ros::Time last_time;
              double dt = (current_time - last_time).toSec();

              ROS_INFO("Got phone msg at: %f, dt: %f\r", current_time.toSec(), dt);
              last_time = current_time;
            }
#endif

#if 0
            ROS_INFO("twist_.angular.x: %f, y: %f, z: %f, accel.x: %f, y: %f, z: %f\r",
                     twist_.angular.x, twist_.angular.y, twist_.angular.z,
                     twist_.linear.x, twist_.linear.y, twist_.linear.z);
#endif
          }
          loop_count = 0;
          msg_parser_state_ = 0;
        }
        field_count_++;
        msg_byte_count_ = 0;
      }
      else
      {
        msg_byte_count_++;
        if (msg_byte_count_ >= PHONE_MSG_MAX_SIZE)
        {
          ROS_WARN("Can't find end of record!\r");
          msg_byte_count_ = 0;
          msg_parser_state_ = 0;
        }
      }
      break;
    }
  } 

  if (loop_count >= 100) ROS_WARN("loop_count >= 100!, msg_parser_state_: %d\r", msg_parser_state_);
}

/*----------------------------------------------------------
 * msgWaiting()
 *--------------------------------------------------------*/

bool Phone::msgWaiting(void)
{
  return msg_waiting_;
}

/*----------------------------------------------------------
 * getAngularZ()
 *--------------------------------------------------------*/

double Phone::getAngularZ(void)
{
  msg_waiting_ = false;

  return twist_.angular.z;
}

/*----------------------------------------------------------
 * dataWaiting()
 *--------------------------------------------------------*/

bool Phone::dataWaiting(void)
{
  struct timeval tv = { 0L, 0L};
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  select(fd_+1, &fds, NULL, NULL, &tv);

  return FD_ISSET(fd_, &fds);
}

} // namespace hexapod_ros

