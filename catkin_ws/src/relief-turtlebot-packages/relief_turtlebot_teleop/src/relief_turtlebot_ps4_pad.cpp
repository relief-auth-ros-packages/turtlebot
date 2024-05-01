/*
 * rb1_base_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the roboy controller, sending the messages received from the joystick device
 */


#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS	 16
#define DEFAULT_AXIS_LINEAR_X		 1
#define DEFAULT_AXIS_LINEAR_Y    0
#define DEFAULT_AXIS_ANGULAR		 2
#define DEFAULT_AXIS_LINEAR_Z    3
#define DEFAULT_SCALE_LINEAR		 1.0
#define DEFAULT_SCALE_ANGULAR		 2.0
#define DEFAULT_SCALE_LINEAR_Z   1.0
#define ITERATIONS_AFTER_DEADMAN 3.0
#define JOY_ERROR_TIME					 1.0

//!//////////////////////////////////////////////////////////////////////
//!                               NOTE:                                //
//! This configuration is made for a PS4 Dualshock                     //
//!   please feel free to modify to adapt for your own joystick.       //
//! 								       //
//!//////////////////////////////////////////////////////////////////////


class RELIEFPS4PAD
{
  public:
    RELIEFPS4PAD();
    void Update();

  private:
    void padCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_x_, linear_y_, linear_z_, angular_;
    double l_scale_, a_scale_, l_scale_z_;
    //! It will publish into command velocity (for the robot)
    ros::Publisher vel_pub_;
    //! It will be subscribed to the joystick
    ros::Subscriber pad_sub_;
    //! Name of the topic where it will be publishing the velocity
    std::string cmd_topic_vel_;
    //! Name of the service where it will be modifying the digital outputs
    std::string cmd_service_io_;
    //! If it is True, it will check the timeout message
    bool check_message_timeout_;
    double current_vel;
    //! Number of the DEADMAN button
    int dead_man_button_;
    //! Number of the button for increase or decrease the speed max of the joystick
    int speed_up_button_, speed_down_button_;
    int button_output_1_, button_output_2_;
    int output_1_, output_2_;
    bool bOutput1, bOutput2;
    //! button to change kinematic mode
    int button_kinematic_mode_;
    //! kinematic mode
    int kinematic_mode_;
    //! Service to modify the kinematic mode
    ros::ServiceClient setKinematicMode;
    //! Name of the service to change the mode
    std::string cmd_set_mode_;
    //! button to start the homing service
    int button_home_;
    //! Name of the service to do the homing
    std::string cmd_home_;

    //! Service to modify the digital outputs
    ros::ServiceClient set_digital_outputs_client_;
    //! Number of buttons of the joystick
    int num_of_buttons_;
    //! Pointer to a vector for controlling the event when pushing the buttons
    bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
    // DIAGNOSTICS
    //! Diagnostic to control the frequency of the published command velocity topic
    diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq;
    //! Diagnostic to control the reception frequency of the subscribed joy topic
    diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq;
    //! General status diagnostic updater
    diagnostic_updater::Updater updater_pad;
    //! Diagnostics min freq
    double min_freq_command, min_freq_joy; //
    //! Diagnostics max freq
    double max_freq_command, max_freq_joy; //
    //! Flag to enable/disable the communication with the publishers topics
    // bool bEnable;
    //! Client of the sound play service
    //  sound_play::SoundClient sc;
    //! Pan & tilt increment (degrees)
    int pan_increment_, tilt_increment_;
};



/*******************************************************************************
*/
RELIEFPS4PAD::RELIEFPS4PAD():
  linear_x_(1),
  linear_y_(0),
  angular_(2),
  linear_z_(3)
{
  current_vel = 0.1;
  //
  nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

  // MOTION CONF
  nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
  nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
  nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
  nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
  nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
  nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
  nh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
  nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
  nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
  nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
  nh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster

  // DIGITAL OUTPUTS CONF
  nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
  nh_.param("button_output_1", button_output_1_, button_output_1_);
  nh_.param("button_output_2", button_output_2_, button_output_2_);
  nh_.param("output_1", output_1_, output_1_);
  nh_.param("output_2", output_2_, output_2_);

  nh_.param("button_home", button_home_, button_home_);
  nh_.param("cmd_service_home", cmd_home_, cmd_home_);
  nh_.param("check_message_timeout", check_message_timeout_, check_message_timeout_);

  ROS_INFO("RELIEFPS4PAD num_of_buttons_ = %d", num_of_buttons_);
  for(int i = 0; i < num_of_buttons_; i++){
    bRegisteredButtonEvent[i] = false;
    ROS_INFO("bREG %d", i);
  }

  // Publish through the node handle Twist type messages to the guardian_controller/command topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);

  // Listen through the node handle sensor_msgs::Joy messages from joystick
  // (these are the references that we will sent to cmd_vel)
  pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RELIEFPS4PAD::padCallback, this);

  bOutput1 = bOutput2 = false;

  // Diagnostics
  updater_pad.setHardwareID("None");
  // Topics freq control
  min_freq_command = min_freq_joy = 5.0;
  max_freq_command = max_freq_joy = 50.0;
  sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

  pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel_.c_str(), updater_pad,
    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));
}


/*******************************************************************************
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void RELIEFPS4PAD::Update(){
  updater_pad.update();
}


/*******************************************************************************
 */
void RELIEFPS4PAD::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  static int send_iterations_after_dead_man = 0;

  // Checks the ROS time to avoid noise in the pad
  if(check_message_timeout_ && ((ros::Time::now() - joy->header.stamp).toSec() > JOY_ERROR_TIME))
    return;

  vel.angular.x = 0.0;  vel.angular.y = 0.0; vel.angular.z = 0.0;
  vel.linear.x = 0.0;   vel.linear.y = 0.0; vel.linear.z = 0.0;

  // Actions dependant on dead-man button
  if (joy->buttons[dead_man_button_] == 1) {
    //ROS_ERROR("RELIEFPS4PAD::padCallback: DEADMAN button %d", dead_man_button_);
    // Set the current velocity level
    if ( joy->buttons[speed_down_button_] == 1 ){
      if(!bRegisteredButtonEvent[speed_down_button_])
        if(current_vel > 0.01){
          current_vel = current_vel - 0.01;
          bRegisteredButtonEvent[speed_down_button_] = true;
          ROS_INFO("Velocity: %f%%", current_vel*100.0);
          char buf[50]="\0";
          int percent = (int) (current_vel*100.0);
          sprintf(buf," %d percent", percent);
          // sc.say(buf);
        }
    }else{
      bRegisteredButtonEvent[speed_down_button_] = false;
    }

    if (joy->buttons[speed_up_button_] == 1){
      if(!bRegisteredButtonEvent[speed_up_button_])
        if(current_vel < 0.9){
          current_vel = current_vel + 0.01;
          bRegisteredButtonEvent[speed_up_button_] = true;
          ROS_INFO("Velocity: %f%%", current_vel*100.0);
          char buf[50]="\0";
          int percent = (int) (current_vel*100.0);
          sprintf(buf," %d percent", percent);
          // sc.say(buf);
        }

    }else{
      bRegisteredButtonEvent[speed_up_button_] = false;
    }

    vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
    vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
    vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
    vel.linear.x = current_vel*l_scale_*joy->axes[linear_x_];
    vel.linear.y = current_vel*l_scale_*joy->axes[linear_y_];
    vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_];
  }
  else {
    vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
    vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
  }

  sus_joy_freq->tick();	// Ticks the reception of joy events

  // Publish only with deadman button pushed for twist use
  if (joy->buttons[dead_man_button_] == 1) {
    send_iterations_after_dead_man = ITERATIONS_AFTER_DEADMAN;
    vel_pub_.publish(vel);
    pub_command_freq->tick();
  } else { // send some 0 if deadman is released
    if (send_iterations_after_dead_man >0) {
      send_iterations_after_dead_man--;
      vel_pub_.publish(vel);
      pub_command_freq->tick();
    }
  }
}


/*******************************************************************************
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "relief_turtlebot_ps4_pad");
  RELIEFPS4PAD relief_turtlebot_ps4_pad;

  ros::Rate r(50.0);

  while( ros::ok() ){
    // UPDATING DIAGNOSTICS
    relief_turtlebot_ps4_pad.Update();
    ros::spinOnce();
    r.sleep();
  }
}
