/*
 * control.hpp
 *
 *  Created on: 27 июля 2014 г.
 *      Author: tuuzdu
 */

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "math.h"
#include <tf/tf.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <copter_msgs/PilotCommands.h>
#include <nav_msgs/Odometry.h>
#include <mavros/RCIn.h>

#include <control_toolbox/pid.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Control {
	public:
		Control();
		bool init();
		void controlLoop();

	private:
		ros::NodeHandle node;
		double rate;
		double max_pitch, max_roll, max_z_vel, min_z_vel, max_xy_vel;
		copter_msgs::PilotCommands pilot_msg;
		bool enable_xy_stab;
		int rc_enable_xy_ch;
		bool rc_enable_xy;

		// times
		ros::Time current_time;
		ros::Time last_time;

		geometry_msgs::Vector3 goalPosition;
		geometry_msgs::Vector3 currentPosition;
		geometry_msgs::Vector3 goalVelocity;
		geometry_msgs::Vector3 currentVelocity;
		double currentRoll, currentPitch, currentYaw;

		// PIDs
		control_toolbox::Pid pid_z;
		control_toolbox::Pid pid_x;
		control_toolbox::Pid pid_y;
		control_toolbox::Pid pid_velocity_goal_x;
		control_toolbox::Pid pid_velocity_goal_y;

		// PIDs Params
		double xy_p_gain, xy_i_gain, xy_d_gain, xy_i_max, xy_i_min;
		double xy_vel_goal_p_gain, xy_vel_goal_i_gain, xy_vel_goal_d_gain, xy_vel_goal_i_max, xy_vel_goal_i_min;
		double z_p_gain, z_i_gain, z_d_gain, z_i_max, z_i_min;

		ros::Publisher pilot_cmd_pub;
		ros::Subscriber odometry_sub;
		ros::Subscriber goalpos_sub;
		ros::Subscriber rc_in_sub;

		void zStabilize ();
		void xyStabilize ();
		void odometryCallback (const nav_msgs::OdometryPtr &odom);
		void goalPositionCallback (const geometry_msgs::Vector3Ptr &goal);
		void RCInCallback (const mavros::RCInConstPtr &rcin);

};

Control::Control(){}

#endif /* CONTROL_HPP_ */
