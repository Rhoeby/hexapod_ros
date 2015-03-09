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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hexapod_ros");
  bool use_phone = true;
  bool use_sample_rejection = false;

  ROS_INFO("Welcome to hexapod_ros app!\r");

  // process command line
  if (argc > 1)
  {
    for (int32_t i=1; i<argc; i++)
    {
      if (argv[i][0] == '-')
      {
        if (strcmp(&argv[i][1], "np")==0)
        {
          use_phone = false;
        }
        else if (strcmp(&argv[i][1], "sr")==0)
        {
          use_sample_rejection = true;
        }
      }
    }
  }

  hexapod_ros::HexapodNode hexapod_node(use_phone, use_sample_rejection);

  ros::Rate loop_rate(10);

  while (ros::ok()) 
  {
    if (hexapod_node.getQuit()==true)
    {
      ROS_INFO("Got quit from node");
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

