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
#include <termios.h>

#include "ros/ros.h"
#include "hexapod_robot.h"

namespace hexapod_ros
{

// options
#define USE_IMPROVED_PORT_SETUP 1

//#define ROBOT_MSG_SIZE_MAX 1024
#define ROBOT_MSG_SIZE_MAX 2048
#define ROBOT_CMD_SIZE_MAX 5

//#define SMOOTHING_BUFFER_SIZE 4 // golden
#define SMOOTHING_BUFFER_SIZE 2

typedef struct SmoothingBuffer_t
{
  double buffer_[SMOOTHING_BUFFER_SIZE];
  uint32_t index_;
  double sum_; 
} SmoothingBuffer_t;

typedef struct VelocitySmoothingBuffers_t
{
  SmoothingBuffer_t x_;
  SmoothingBuffer_t y_;
  SmoothingBuffer_t th_;
} VelocitySmoothingBuffers_t;

uint8_t calcChecksum(const uint8_t *data, const uint32_t length);

/*----------------------------------------------------------
 * Constructor()
 *--------------------------------------------------------*/

Robot::Robot()
{
  msg_buffer_ = new uint8_t[ROBOT_MSG_SIZE_MAX];
  cmd_buffer_ = new uint8_t[ROBOT_CMD_SIZE_MAX];
  scan_data_buffer_ = new double[ROBOT_MSG_SIZE_MAX / 2];
  smoothing_buffers_ = new VelocitySmoothingBuffers_t;

  memset(&smoothing_buffers_->x_, 0, sizeof(SmoothingBuffer_t));
  memset(&smoothing_buffers_->y_, 0, sizeof(SmoothingBuffer_t));
  memset(&smoothing_buffers_->th_, 0, sizeof(SmoothingBuffer_t));
  
  msg_parser_state_ = 0;
  msg_payload_byte_count_ = 0;

  vx_ = 0.0;
  vy_ = 0.0;
  vth_ = 0.0;

  status_.samples_per_scan_ = 200;
  status_.scan_period_ = 1000;
  status_.scan_data_available_ = false;
}

/*----------------------------------------------------------
 * Destructor()
 *--------------------------------------------------------*/

Robot::~Robot()
{
  int32_t rc;

  if (fd_ != -1)
  {
    rc = ::close(fd_);
    if (rc) ROS_ERROR("close failed!");
  }

  delete msg_buffer_;
  delete cmd_buffer_;
  delete scan_data_buffer_;
  delete smoothing_buffers_;
}

/*----------------------------------------------------------
 * open()
 *--------------------------------------------------------*/
void Robot::open(const std::string port_name)
{
  int32_t n;
  int32_t i;
  uint8_t ch;
#if USE_IMPROVED_PORT_SETUP
  struct termios new_termios;
#endif

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

#if USE_IMPROVED_PORT_SETUP
  tcgetattr(fd_, &new_termios);
  cfmakeraw(&new_termios);
  cfsetospeed(&new_termios, B115200);
  tcsetattr(fd_, TCSANOW, &new_termios);
#endif

  ROS_INFO("Polling (for BT) before executing commands...\r");
  for (i=0; i< 10; i++)
  {
    // TODO: is this still needed? replace with ros version?
    //ros::Duration(0.01).sleep();
    sleep(1);

    ROS_DEBUG("g\r");
    n = ::write(fd_, "g", 1);

    while (dataWaiting())
    {
      n = ::read(fd_, &ch, 1);
      if (ch=='g')
      {
        ROS_DEBUG("Got response!\r");
        goto done;
      }
    }
  }
  done:

  // enter logging mode
  ROS_DEBUG("L\r");
  n = ::write(fd_, "L", 1);
  logging_mode_on_ = true;

  // set scanner speed
  ROS_DEBUG("4\r");
//  n = ::write(fd_, "4", 1);
  n = ::write(fd_, "3", 1);

  // enter holonomic mode
  ROS_DEBUG("n\r");
  n = ::write(fd_, "n", 1);

#if 0
  // set sample rejection mode: on
  ROS_DEBUG("R\r");
  n = ::write(fd_, "R", 1);
#endif

#if 0
  // disable collision avoidance
  ROS_DEBUG("a\r");
  n = ::write(fd_, "a", 1);
#endif
}

/*----------------------------------------------------------
 * close() - close the port
 *--------------------------------------------------------*/

void Robot::close()
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
 * getVelocityX()
 *--------------------------------------------------------*/

double Robot::getVelocityX(void)
{
  return vx_;
}

/*----------------------------------------------------------
 * getVelocityY()
 *--------------------------------------------------------*/

double Robot::getVelocityY(void)
{
  return vy_;
}

/*----------------------------------------------------------
 * getVelocityTheta()
 *--------------------------------------------------------*/

double Robot::getVelocityTheta(void)
{
  return vth_;
}

/*----------------------------------------------------------
 * getStatus()
 *--------------------------------------------------------*/

void Robot::getStatus(RobotStatus_t *status)
{
  memcpy(status, &status_, sizeof(status_));
}

/*----------------------------------------------------------
 * getScanData()
 *--------------------------------------------------------*/

void Robot::getScanData(double *data)
{
  memcpy(data, scan_data_buffer_, sizeof(double) * status_.samples_per_scan_);

  status_.scan_data_available_ = false;
}

/*----------------------------------------------------------
 * calcVelocities()
 *--------------------------------------------------------*/

void Robot::calcVelocities(void)
{
  vy_ = (int8_t)msg_buffer_[5] / -1000.0; // SWAPPED!!!
  vx_ = (int8_t)msg_buffer_[6] / 1075.0; 
  vth_ = (int8_t)msg_buffer_[7] / 151.25;

#if 0
  ROS_INFO("msg_buffer[5]: %d, [6]: %d, [7]: %d\r", 
           msg_buffer_[5],
           msg_buffer_[6],
           msg_buffer_[7]);
#endif
}

/*----------------------------------------------------------
 * calcScanData()
 *--------------------------------------------------------*/

void Robot::calcScanData(void)
{
  int32_t i;
  uint32_t scan_value;
  double tmp;

  for (i=0; i<msg_payload_length_; i+=2)
  {
    scan_value = msg_buffer_[5+i] | (msg_buffer_[6+i] << 8);

    tmp = scan_value / 1000.0;

//    ROS_INFO("scan_value: %d, range (in meters/inches) %f / %f\r", scan_value, tmp, (tmp * 100/2.54));

    scan_data_buffer_[i/2] = tmp;
  }
//  ROS_INFO("\r");

  status_.samples_per_scan_ = msg_payload_length_ / 2;

  status_.scan_data_available_ = true;
}

/*----------------------------------------------------------
 * parseScannerStatus()
 *--------------------------------------------------------*/

void Robot::parseScannerStatus(void)
{
  uint32_t samples_per_scan = msg_buffer_[5] | (msg_buffer_[6] << 8);
  uint8_t flags = msg_buffer_[9];

  status_.scan_period_ = msg_buffer_[7] | (msg_buffer_[8] << 8);

#if 0
  ROS_INFO("Got status message\r");
  ROS_INFO("  samplesPerScan: %d\r", samples_per_scan);
  ROS_INFO("  scanPeriod: %d\r", status_.scan_period_);
  ROS_INFO("  flags: %d\r", flags);
#endif
}

/*----------------------------------------------------------
 * readMsg()
 *--------------------------------------------------------*/

void Robot::readMsg(void)
{
  int32_t loop_count = 0;
  ssize_t ret = 0;
  uint8_t tmp;

  while (dataWaiting() && loop_count<300)
  {

    if (!logging_mode_on_)
    {
      uint8_t ch;

      ret = ::read(fd_, &ch, 1);
//      ROS_INFO("%c\r", ch);
      continue;
    }

    switch (msg_parser_state_)
    {
    case 0:     // looking for sync byte 0
      ret = ::read(fd_, &msg_buffer_[0], 1);
      if (msg_buffer_[0] == 0xFF)
        msg_parser_state_ = 1;
      else if (loop_count < 4)
        ROS_DEBUG("Out of sync! \r");
      break;
    case 1:     // looking for sync byte 1
      ret = ::read(fd_, &msg_buffer_[1], 1);
      if (msg_buffer_[1] == 0xFF)
        msg_parser_state_ = 2;
      else
        msg_parser_state_ = 0;
      break;
    case 2:
      ret = ::read(fd_, &msg_buffer_[2], 1);
      msg_type_ = msg_buffer_[2];
      msg_parser_state_ = 3;
      break;
    case 3:
      ret = ::read(fd_, &msg_buffer_[3], 1);
      msg_payload_length_ = msg_buffer_[3] << 8;
      msg_parser_state_ = 4;
      break;
    case 4:
      ret = ::read(fd_, &msg_buffer_[4], 1);
      msg_payload_length_ |= msg_buffer_[4];
      if (msg_payload_length_)
      {
        msg_parser_state_ = 5;
        msg_payload_byte_count_ = 0;
      }
      else
      {
        msg_parser_state_ = 6;
      }
      break;
    case 5:
      ret = ::read(fd_, &msg_buffer_[5 + msg_payload_byte_count_], 1);
      msg_payload_byte_count_++;
      if (msg_payload_byte_count_ >= msg_payload_length_)
      {
        msg_parser_state_ = 6;
      }
      break;
    case 6:
      uint8_t checksum = calcChecksum(msg_buffer_, msg_payload_byte_count_ + 5);
      ret = ::read(fd_, &tmp, 1);
      if (tmp == checksum)
      {
        switch (msg_type_)
        {
        case 0:
          // ROS_INFO("Got scan data, msg_payload_length_: %d \r", msg_payload_length_);
          calcScanData();
          break;
        case 1:
          // ROS_INFO("Got scanner status \r");
          parseScannerStatus();
          break;
        case 2:
          // ROS_INFO("Got robot status \r");
          calcVelocities();
          break;
        }
      }
      else
      {
        ROS_WARN("Message (type: %d) checksum failed!\r", msg_type_);
      }      
      msg_parser_state_ = 0;
      break;
    }
#if 0
    if (ret)
    {
      ROS_INFO("Error reading data from robot\r");
      break;
    }
#endif
    loop_count++; // reset when complete msg received
  }

  if (loop_count >= 400) ROS_WARN("loop_count >= 400!, msg_parser_state_: %d \r", msg_parser_state_);
}

/*----------------------------------------------------------
 * writeCmd()
 *--------------------------------------------------------*/

void Robot::writeCmd(const int8_t cmd)
{
  if (cmd == 'L')
  {
    logging_mode_on_ = (~logging_mode_on_) & 0x1;
    ROS_DEBUG("Toggling logging_mode_on_: %d\r", logging_mode_on_); 
  }

  ssize_t ret = ::write(fd_, &cmd, 1);
}

/*----------------------------------------------------------
 * calcSmoothed()
 *--------------------------------------------------------*/

double calcSmoothed(SmoothingBuffer_t *buffer, double new_value)
{
  buffer->sum_ += new_value;
  buffer->sum_ -= buffer->buffer_[buffer->index_];
  buffer->buffer_[buffer->index_] = new_value;

  buffer->index_++;
  if (buffer->index_ >= SMOOTHING_BUFFER_SIZE) buffer->index_ = 0;

  return buffer->sum_ / SMOOTHING_BUFFER_SIZE;
}

/*----------------------------------------------------------
 * setVelocities()
 *--------------------------------------------------------*/

void Robot::setVelocities(const double x, const double y, const double th)
{
  double remapped_x = -y;
  double remapped_y = x;
  double smoothed_x;
  double smoothed_y;
  double smoothed_th;

  if (fabs(remapped_x) > 0.050001) ROS_WARN("X is out-of-range: %f in robotSetVelocities()\r", remapped_x);
  if (fabs(remapped_y) > 0.100001) ROS_WARN("Y is out-of-range: %f in robotSetVelocities()\r", remapped_y);
  if (fabs(th) > 0.300001) ROS_WARN("Theta is out-of-range: %f in robotSetVelocities()\r", th);

  smoothed_x = calcSmoothed(&smoothing_buffers_->x_, remapped_x);
  smoothed_y = calcSmoothed(&smoothing_buffers_->y_, remapped_y);
  smoothed_th = calcSmoothed(&smoothing_buffers_->th_, th);

  cmd_buffer_[0] = 0xFF;
  cmd_buffer_[1] = 0xFF;
  cmd_buffer_[2] = (int8_t)(smoothed_x * 1000);
  cmd_buffer_[3] = (int8_t)(smoothed_y * 1000);
  cmd_buffer_[4] = (int8_t)(smoothed_th * -167);

#if 0
  ROS_INFO("smoothed x: %f [%f], y: %f [%f], th: %f [%f]\r",
      smoothed_x, remapped_x, 
      smoothed_y, remapped_y,
      smoothed_th, th);
#endif

  ssize_t ret = write(fd_, cmd_buffer_, 5);
}

/*----------------------------------------------------------
 * dataWaiting()
 *--------------------------------------------------------*/

bool Robot::dataWaiting(void)
{
  struct timeval tv = { 0L, 0L};
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  select(fd_+1, &fds, NULL, NULL, &tv);

  return FD_ISSET(fd_, &fds);
}

/*----------------------------------------------------------
 * calcChecksum()
 *--------------------------------------------------------*/

uint8_t calcChecksum(const uint8_t *data, const uint32_t length)
{
  uint32_t i;
  uint8_t checksum = 0;

  for (i=0; i<length; i++)
  {
    checksum += data[i];
  }

  return ~checksum;
}

} // namespace hexapod_ros

