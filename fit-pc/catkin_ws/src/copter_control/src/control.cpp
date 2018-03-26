#include "control.hpp"


bool Control::init() {

	node.param("/copter_control/rate", rate, 30.0);
	node.param("/copter_control/enable_xy_stab", enable_xy_stab, false);
	node.param("/copter_control/rc_enable_xy_ch", rc_enable_xy_ch, 5);

    // common
	node.param("/copter_control/max_pitch", max_pitch, 3.0);
	node.param("/copter_control/max_roll", max_roll, 3.0);
	node.param("/copter_control/max_xy_vel", max_xy_vel, 0.8);
	node.param("/copter_control/max_z_vel", max_z_vel, 0.2);
	node.param("/copter_control/min_z_vel", min_z_vel, -0.2);

    // X-Y PID
	node.param("/copter_control/xy_p_gain", xy_p_gain, 4.0);
	node.param("/copter_control/xy_i_gain", xy_i_gain, 1.0);
	node.param("/copter_control/xy_d_gain", xy_d_gain, 0.05);
	node.param("/copter_control/xy_i_max", xy_i_max, 1.0);
	node.param("/copter_control/xy_i_min", xy_i_min, -1.0);

    // X-Y Velocities goal PID
	node.param("/copter_control/xy_vel_goal_p_gain", xy_vel_goal_p_gain, 1.0);
	node.param("/copter_control/xy_vel_goal_i_gain", xy_vel_goal_i_gain, 0.06);
	node.param("/copter_control/xy_vel_goal_d_gain", xy_vel_goal_d_gain, 1.2);
	node.param("/copter_control/xy_vel_goal_i_max", xy_vel_goal_i_max, 0.1);
	node.param("/copter_control/xy_vel_goal_i_min", xy_vel_goal_i_min, -0.1);

    // Z PID
	node.param("/copter_control/z_p_gain", z_p_gain, 2.0);
	node.param("/copter_control/z_i_gain", z_i_gain, 1.2);
	node.param("/copter_control/z_d_gain", z_d_gain, 0.6);
	node.param("/copter_control/z_i_max", z_i_max, 0.03);
	node.param("/copter_control/z_i_min", z_i_min, -0.03);

//	pid_x.initPid(4, 1, 0.05, 1, -1);
//	pid_y.initPid(4, 1, 0.05, 1, -1);
//	pid_z.initPid(2, 1.2, 0.6, 0.03, -0.03);
//
//	pid_velocity_goal_x.initPid(1, 0.06, 1.2, 0.1, -0.1);
//	pid_velocity_goal_y.initPid(1, 0.06, 1.2, 0.1, -0.1);

	pid_x.initPid(xy_p_gain, xy_i_gain, xy_d_gain, xy_i_max, xy_i_min);
	pid_y.initPid(xy_p_gain, xy_i_gain, xy_d_gain, xy_i_max, xy_i_min);
	pid_z.initPid(z_p_gain, z_i_gain, z_d_gain, z_i_max, z_i_min);

	pid_velocity_goal_x.initPid(xy_vel_goal_p_gain, xy_vel_goal_i_gain, xy_vel_goal_d_gain, xy_vel_goal_i_max, xy_vel_goal_i_min);
	pid_velocity_goal_y.initPid(xy_vel_goal_p_gain, xy_vel_goal_i_gain, xy_vel_goal_d_gain, xy_vel_goal_i_max, xy_vel_goal_i_min);

	pilot_cmd_pub = node.advertise<copter_msgs::PilotCommands>("pilot_command", 1);
	odometry_sub = node.subscribe("/odometry/filtered", 1, &Control::odometryCallback, this);
	goalpos_sub = node.subscribe("/goal_prosition", 1, &Control::goalPositionCallback, this);
	rc_in_sub = node.subscribe("/mavros/rc/in", 1, &Control::RCInCallback, this);

	rc_enable_xy = false;

	return true;
}

void Control::controlLoop(){

	while (node.ok()){

		current_time = ros::Time::now();

		if (currentPosition.z < 0.21 && goalPosition.z < 0.21){
			pilot_msg.engine_power = 0;
			pilot_msg.pitch = 0;
			pilot_msg.roll = 0;
			pilot_msg.throttle = 0;
			pilot_msg.yaw = 0;
			pid_x.reset();
			pid_y.reset();
			pid_z.reset();
			pid_velocity_goal_x.reset();
			pid_velocity_goal_y.reset();
		}
		else {
			pilot_msg.engine_power = 1;

			zStabilize();
			if (enable_xy_stab && rc_enable_xy)	xyStabilize();
			else if (enable_xy_stab){
				pid_x.reset();
				pid_y.reset();
				pid_velocity_goal_x.reset();
				pid_velocity_goal_y.reset();
			}
		}

		pilot_cmd_pub.publish(pilot_msg);

		last_time = current_time;

		ros::spinOnce();
		ros::Rate(rate).sleep();
	}

}

void Control::zStabilize (){

//	double ie, pe, de;
    double error = goalPosition.z - currentPosition.z; // P-error
    error = constrain(error, -1.0, 1.0);

//    ROS_INFO ("%f %f", goalPosition.z, currentPosition.z);

    pilot_msg.throttle = pid_z.computeCommand(error, current_time-last_time);

    pilot_msg.throttle = constrain(pilot_msg.throttle, min_z_vel, max_z_vel);

//    pid_z.getCurrentPIDErrors(&pe, &ie, &de);

//    ROS_INFO ("\nP: \t\t%f \tI: \t\t%f \tD: \t\t%f", pe, ie, de);

    ROS_INFO ("\nthrottle: \t\t%f \terror: \t\t%f", pilot_msg.throttle, error);
}

void Control::xyStabilize (){

    ros::Duration delta_t = current_time - last_time;

    double x_error = (currentPosition.x - goalPosition.x)*cos(currentYaw) + (currentPosition.y - goalPosition.y)*sin(currentYaw);
    double y_error = (currentPosition.x - goalPosition.x)*sin(currentYaw)*(-1.0) + (currentPosition.y - goalPosition.y)*cos(currentYaw);

    x_error = constrain(x_error, -5, 5);
    y_error = constrain(y_error, -5, 5);

    // get goals velocities as result of pid, based on position errors.
    double x_vel_goal = pid_velocity_goal_x.computeCommand(x_error, delta_t);
    double y_vel_goal = pid_velocity_goal_y.computeCommand(y_error, delta_t);

    x_vel_goal = constrain(x_vel_goal, -max_xy_vel, max_xy_vel);
    y_vel_goal = constrain(y_vel_goal, -max_xy_vel, max_xy_vel);

    double x_vel_current = currentVelocity.x;
    double y_vel_current = currentVelocity.y;

	// control pitch and roll as result of "velocity" PID
    pilot_msg.pitch = pid_x.computeCommand(-x_vel_current - x_vel_goal, delta_t);
    pilot_msg.roll = pid_y.computeCommand(y_vel_current + y_vel_goal, delta_t);

    pilot_msg.pitch = constrain(pilot_msg.pitch, -max_pitch, max_pitch);
    pilot_msg.roll = constrain(pilot_msg.roll, -max_roll, max_roll);
    ROS_INFO ("\npitch: \t\t%f \troll: \t\t%f \nx_vel_goal: \t%f \ty_vel_goal: \t%f \nx_error: \t%f \ty_error: \t%f \ncurrentYaw: \t%f",
	      pilot_msg.pitch, pilot_msg.roll, x_vel_goal, y_vel_goal, x_error, y_error, 57.32 * currentYaw);

}


void Control::odometryCallback (const nav_msgs::OdometryPtr &odom){
	currentPosition.x = odom->pose.pose.position.x;
	currentPosition.y = odom->pose.pose.position.y;
	currentPosition.z = odom->pose.pose.position.z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(currentRoll, currentPitch, currentYaw);

	currentVelocity.x = odom->twist.twist.linear.x;
	currentVelocity.y = odom->twist.twist.linear.y;
}

void Control::goalPositionCallback (const geometry_msgs::Vector3Ptr &goal){
	goalPosition.x = goal->x;
	goalPosition.y = goal->y;
	goalPosition.z = goal->z;
}

void Control::RCInCallback (const mavros::RCInConstPtr &rcin){
	if (rcin->channels[rc_enable_xy_ch] < 1500) rc_enable_xy = true;
	else rc_enable_xy = false;
	}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_node");
	Control c;
    if (c.init()<0) {
        ROS_ERROR("Could not initialize control node");
        return -1;
    }
    c.controlLoop();

    ros::spin();
    return 0;
}
