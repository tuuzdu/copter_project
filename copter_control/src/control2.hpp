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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <copter_msgs/PilotCommands.h>
#include <nav_msgs/Odometry.h>

#include <control_toolbox/pid.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class Control {
	public:
		Control();
		bool init();
		void controlLoop();

	private:
		ros::NodeHandle node;
//		std::string root_name, tip_name;
//		std::vector<KDL::Frame> frames;
//		double fi;
//		crab_msgs::LegsJointsState legs;
//		crab_msgs::GetLegIKSolver srv;
//		crab_msgs::GaitCommand gait_command;
//		const static unsigned int num_joints = NUM_JOINTS;
//		const static unsigned int num_legs = NUM_LEGS;
//		double trap_low_r, trap_high_r, trap_h, trap_z;
		double rate;
		copter_msgs::PilotCommands pilot_msg;

		// times
		ros::Time current_time;
		ros::Time last_time;

		geometry_msgs::Pose goalPosition;
		geometry_msgs::Pose currentPosition;
		geometry_msgs::Vector3 goalVelocity;
		geometry_msgs::Vector3 currentVelocity;
		geometry_msgs::Vector3 pidPosition;
		geometry_msgs::Pose oldPosition;

		// PIDs
		control_toolbox::Pid pid_z;
		control_toolbox::Pid pid_x;
		control_toolbox::Pid pid_y;
		control_toolbox::Pid pid_velocity_goal_x;
		control_toolbox::Pid pid_velocity_goal_y;
//
//		ros::ServiceClient client;
		ros::Publisher pilot_cmd_pub;
		ros::Subscriber odometry_sub;
		ros::Subscriber goalpos_sub;
//
//		bool loadModel(const std::string xml);
//		bool callService (KDL::Vector* vector);
		void zStabilize ();
		void xyStabilize ();
		void odometryCallback (const nav_msgs::OdometryPtr &odom);
		void goalPositionCallback (const geometry_msgs::Vector3Ptr &goal);

};

Control::Control(){}

#endif /* CONTROL_HPP_ */
