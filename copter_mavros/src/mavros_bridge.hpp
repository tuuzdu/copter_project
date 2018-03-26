/*
 * mavros_bridge.hpp
 *
 *  Created on: 06 марта 2015 г.
 *      Author: tuuzdu
 */

#ifndef COPTER_MAVROS_SRC_MAVROS_BRIDGE_HPP_
#define COPTER_MAVROS_SRC_MAVROS_BRIDGE_HPP_

#include "ros/ros.h"
#include <copter_msgs/PilotCommands.h>
#include <mavros/OverrideRCIn.h>
#include <mavros/RCIn.h>
#include <mavros/CommandBool.h>

class MavBridge {
	public:
		MavBridge();
		bool init();

	private:
		ros::NodeHandle node;

		mavros::OverrideRCIn override_msg;

		ros::Publisher rc_override_pub;
		ros::Subscriber pilot_cmd_sub;
		ros::Subscriber rc_in_sub;
		ros::ServiceClient mavcmd_client;

		int rc_control_ch;
		bool arming_flag, rc_control, enable_xy_stab;
		double throttle_z_vel, throttle_center, pitch_deg, pitch_center, roll_deg, roll_center;

		void pilotCmdCallback (const copter_msgs::PilotCommandsPtr &pcmd);
		bool callArmingService (bool arming);
		void RCInCallback (const mavros::RCInConstPtr &rcin);

};

MavBridge::MavBridge(){}


#endif /* COPTER_MAVROS_SRC_MAVROS_BRIDGE_HPP_ */
