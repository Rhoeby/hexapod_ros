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

#ifndef _HEXAPOD_ROBOT_
#define _HEXAPOD_ROBOT_

namespace hexapod_ros
{

using std::string;

typedef struct RobotStatus_t {
	uint32_t samples_per_scan_;
  uint32_t scan_period_;
	bool scan_data_available_;
} RobotStatus_t;

struct VelocitySmoothingBuffers_t;

class Robot
{
public:
  Robot();
  virtual ~Robot();

  void open(const std::string port_name);
  void close();

  void readMsg();
  void writeCmd(const int8_t cmd);

  void getStatus(RobotStatus_t *status);

  double getVelocityX();
  double getVelocityY();
  double getVelocityTheta();
  void getScanData(double *data);

  void setVelocities(const double x, const double y, const double th);

private:
  void calcVelocities();
  void calcScanData();
  void parseScannerStatus();
  bool dataWaiting();

  int32_t fd_;
  uint8_t *cmd_buffer_;
  uint8_t *msg_buffer_;
  uint32_t msg_parser_state_;
  uint32_t msg_payload_byte_count_;
  uint32_t msg_type_;
  uint32_t msg_payload_length_;
  bool logging_mode_on_;

  double vx_;
  double vy_;
  double vth_;
  double *scan_data_buffer_;

  RobotStatus_t status_;
  VelocitySmoothingBuffers_t *smoothing_buffers_;
};

} // namespace hexapod_ros

#endif // _HEXAPOD_ROBOT_
