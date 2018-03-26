/*
 * mavros_bridge.cpp
 *
 *  Created on: 06 марта 2015 г.
 *      Author: tuuzdu
 */
#include "mavros_bridge.hpp"




bool MavBridge::init(){

	node.param("/copter_control/enable_xy_stab", enable_xy_stab, false);
	node.param("/copter_mavros/rc_control_ch", rc_control_ch, 7);

	node.param("/copter_mavros/throttle_z_vel", throttle_z_vel, 1250.0);
	node.param("/copter_mavros/throttle_center", throttle_center, 1500.0);
	node.param("/copter_mavros/pitch_deg", pitch_deg, 1250.0);
	node.param("/copter_mavros/pitch_center", pitch_center, 1500.0);
	node.param("/copter_mavros/roll_deg", roll_deg, 1250.0);
	node.param("/copter_mavros/roll_center", roll_center, 1500.0);

	mavcmd_client = node.serviceClient<mavros::CommandBool>("/mavros/cmd/arming");
	rc_override_pub = node.advertise<mavros::OverrideRCIn>("mavros/rc/override", 1);
	pilot_cmd_sub = node.subscribe("/pilot_command", 1, &MavBridge::pilotCmdCallback, this);
	rc_in_sub = node.subscribe("/mavros/rc/in", 1, &MavBridge::RCInCallback, this);

	arming_flag = false;
	rc_control = false;

	return true;
}

void MavBridge::pilotCmdCallback(const copter_msgs::PilotCommandsPtr &pcmd){

	if (rc_control == false){
		if (pcmd->engine_power == true && arming_flag == false){
			if (callArmingService(true)) arming_flag = true;
		}
		else if (pcmd->engine_power == false && arming_flag == true){
			if (callArmingService(false)) arming_flag = false;
		}
		if (arming_flag == true){
			if (enable_xy_stab){
				override_msg.channels[0] = uint16_t (roll_center + pcmd->roll * roll_deg);
				override_msg.channels[1] = uint16_t (pitch_center - pcmd->pitch * pitch_deg);
			}
			override_msg.channels[2] = uint16_t (throttle_center + pcmd->throttle * throttle_z_vel);
		}
	}
	else {
		override_msg.channels[0] = 0;
		override_msg.channels[1] = 0;
		override_msg.channels[2] = 0;
		override_msg.channels[3] = 0;
	}

	rc_override_pub.publish(override_msg);

}

void MavBridge::RCInCallback (const mavros::RCInConstPtr &rcin){
	if (rcin->channels[rc_control_ch] < 1500) rc_control = true;
	else rc_control = false;
	}

bool MavBridge::callArmingService(bool arming){
	mavros::CommandBool cmd;

	if (arming == true) {
		cmd.request.value = true;
		if (mavcmd_client.call(cmd)){
			if (cmd.response.success == true) return true;
		}
	}

	if (arming == false) {
		cmd.request.value = false;
		if (mavcmd_client.call(cmd)){
			if (cmd.response.success == true) return true;
		}
	}

	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavbridge_node");
	MavBridge mb;
    if (mb.init()<0) {
        ROS_ERROR("Could not initialize control node");
        return -1;
    }

    ros::spin();
    return 0;
}

