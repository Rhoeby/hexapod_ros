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

#include <termios.h>
#include <sys/select.h>
#include <string.h>
#include <unistd.h>

#include "ros/ros.h"
#include "hexapod_console.h"

namespace hexapod_ros
{

/*----------------------------------------------------------
 * Constructor()
 *--------------------------------------------------------*/

Console::Console()
{
  setTerminalMode();
}

/*----------------------------------------------------------
 * Destructor()
 *--------------------------------------------------------*/

Console::~Console()
{
  resetTerminalMode();
}

/*----------------------------------------------------------
 * resetTerminalMode()
 *--------------------------------------------------------*/
void Console::resetTerminalMode()
{
  tcsetattr(0, TCSANOW, &orig_termios_);
}

/*----------------------------------------------------------
 * setTerminalMode()
 *--------------------------------------------------------*/
void Console::setTerminalMode()
{
  struct termios new_termios;

  /* take two copies - one for now, one for later */
  tcgetattr(0, &orig_termios_);
  memcpy(&new_termios, &orig_termios_, sizeof(new_termios));

  /* set the new terminal mode */
  cfmakeraw(&new_termios);
  tcsetattr(0, TCSANOW, &new_termios);
}

/*----------------------------------------------------------
 * kbHit()
 *--------------------------------------------------------*/
int32_t Console::kbHit()
{
  struct timeval tv = { 0L, 0L};

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

/*----------------------------------------------------------
 * getCh()
 *--------------------------------------------------------*/
int32_t Console::getCh()
{
  int32_t r;
  uint8_t c;

  if ((r = read(0, &c, sizeof(c))) < 0)
  {
    return r;
  }
  else
  {
    return c;
  }
}

} // namespace hexapod_ros

