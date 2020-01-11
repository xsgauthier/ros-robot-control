/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <rrbot_control/rrbot_hw_interface.h>
#include <stdio.h>
#include <pigpiod_if2.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>

#define LEFT_WHEEL_ID   0
#define RIGHT_WHEEL_ID  1

#define PIN_L_ENB       13
#define PIN_L_A         27
#define PIN_L_B         22

#define PIN_R_ENB       12
#define PIN_R_A         23
#define PIN_R_B         24

typedef struct {
  long count;
  signed dir;
  int id;
}tachdata;

static     tachdata left_tach, right_tach;

static void tachometer_cb(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void * userdata)
{
  tachdata* tach = (tachdata*)userdata;
  //printf("%d\n", level);
  if (level == 1)
        tach->count += tach->dir;
}

static int get_vel_pwm(int cmd)
{
  int cc = (int)(255.0 * abs(cmd) / 4.0);
  if (cc > 255)cc = 255;
  if (cc < 0)cc = 0;
  return cc;
}

static int io_init(void)
{
   int handle = pigpio_start(NULL, NULL);

   if (handle < 0 /*gpioInitialise() < 0*/)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return 1;
   }

    set_mode(handle, PIN_L_A, PI_OUTPUT);
    set_mode(handle, PIN_L_B, PI_OUTPUT);
    set_mode(handle, PIN_R_A, PI_OUTPUT);
    set_mode(handle, PIN_R_B, PI_OUTPUT);

    gpio_write(handle, PIN_L_A, 0);
    gpio_write(handle, PIN_L_B, 0);
    gpio_write(handle, PIN_R_A, 0);
    gpio_write(handle, PIN_R_B, 0);

    set_PWM_frequency(handle, PIN_L_ENB, 100);
    set_PWM_frequency(handle, PIN_R_ENB, 100);

    set_PWM_dutycycle(handle, PIN_L_ENB, 0);
    set_PWM_dutycycle(handle, PIN_R_ENB, 0);

   return handle;
}

static void io_deinit(int handle)
{
  pigpio_stop(handle);
}

namespace rrbot_control
{

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    left_tach.count = 0;
    right_tach.count = 0;
  init_ok = io_init();
  if (init_ok >= 0){
        set_glitch_filter(init_ok, 5, 10000);
        set_glitch_filter(init_ok, 6, 10000);
        left_tach.id = callback_ex(init_ok, 5, RISING_EDGE, tachometer_cb, &left_tach);
        right_tach.id = callback_ex(init_ok, 6, RISING_EDGE, tachometer_cb, &right_tach);

    ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
  } else {
    ROS_ERROR_NAMED("rrbot_hw_interface", "RRBotHWInterface Failed to initalize.");
  }
}

RRBotHWInterface::~RRBotHWInterface(){
  io_deinit(init_ok);
}

void RRBotHWInterface::read(ros::Duration &elapsed_time)
{
  if (init_ok >= 0)
  {
   printf("TACH: %5d %5d",  left_tach.count, right_tach.count);
    joint_position_[LEFT_WHEEL_ID] = left_tach.count;
    joint_position_[RIGHT_WHEEL_ID] = right_tach.count;
  }
}

void RRBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  if (init_ok >= 0)
  {
    GenericHWInterface::printCommandHelper();

    //joint_velocity_command_[joint_id]
    if (joint_velocity_command_[LEFT_WHEEL_ID] < 0)
    {
      gpio_write(init_ok, PIN_L_A, 0);
      gpio_write(init_ok, PIN_L_B, 1);
      set_PWM_dutycycle(init_ok, PIN_L_ENB, get_vel_pwm(joint_velocity_command_[LEFT_WHEEL_ID]));
      left_tach.dir = -1;
    }
    else if (joint_velocity_command_[LEFT_WHEEL_ID] > 0)
    {
      gpio_write(init_ok, PIN_L_A, 1);
      gpio_write(init_ok, PIN_L_B, 0);
      set_PWM_dutycycle(init_ok, PIN_L_ENB, get_vel_pwm(joint_velocity_command_[LEFT_WHEEL_ID]));
      left_tach.dir = 1;
    }
    else if (joint_velocity_command_[LEFT_WHEEL_ID] == 0)
    {
      gpio_write(init_ok, PIN_L_A, 0);
      gpio_write(init_ok, PIN_L_B, 0);
      set_PWM_dutycycle(init_ok, PIN_L_ENB, 255);
      left_tach.dir = 0;
    }

    joint_velocity_[LEFT_WHEEL_ID] = joint_velocity_command_[LEFT_WHEEL_ID];    

    if (joint_velocity_command_[RIGHT_WHEEL_ID] < 0)
    {
      gpio_write(init_ok, PIN_R_A, 0);
      gpio_write(init_ok, PIN_R_B, 1);
      set_PWM_dutycycle(init_ok, PIN_R_ENB, get_vel_pwm(joint_velocity_command_[RIGHT_WHEEL_ID]));
      right_tach.dir = -1;
    }
    else if (joint_velocity_command_[RIGHT_WHEEL_ID] > 0)
    {
      gpio_write(init_ok, PIN_R_A, 1);
      gpio_write(init_ok, PIN_R_B, 0);
      set_PWM_dutycycle(init_ok, PIN_R_ENB, get_vel_pwm(joint_velocity_command_[RIGHT_WHEEL_ID]));
      right_tach.dir = 1;
    }
    else if (joint_velocity_command_[RIGHT_WHEEL_ID] == 0)
    {
      gpio_write(init_ok, PIN_R_A, 0);
      gpio_write(init_ok, PIN_R_B, 0);
      set_PWM_dutycycle(init_ok, PIN_R_ENB, 255);
      right_tach.dir = 0;
    }

    joint_velocity_[RIGHT_WHEEL_ID] = joint_velocity_command_[RIGHT_WHEEL_ID];    

  }
}

void RRBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
