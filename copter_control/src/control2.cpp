#include "control.hpp"


bool Control::init() {

	node.param("rate", rate, 25.0);

	pid_z.initPid(2, 1.2, 0.6, 0.03, -0.03);
	pid_x.initPid(4, 1, 0.05, 1, -1);
	pid_y.initPid(4, 1, 0.05, 1, -1);

	pid_velocity_goal_x.initPid(1, 0.06 , 1.2, 0.1, -0.1);
	pid_velocity_goal_y.initPid(1, 0.06 , 1.2, 0.1, -0.1);

//	client = node.serviceClient<crab_msgs::GetLegIKSolver>("/crab_leg_kinematics/get_ik");
	pilot_cmd_pub = node.advertise<copter_msgs::PilotCommands>("pilot_command", 1);
	odometry_sub = node.subscribe("/odometry/filtered", 1, &Control::odometryCallback, this);
	goalpos_sub = node.subscribe("/goal_prosition", 1, &Control::goalPositionCallback, this);

	return true;
}

void Control::controlLoop(){

//	goalPosition.z = 2;

	while (node.ok()){
		// z stabilization
		current_time = ros::Time::now();

		zStabilize();
		xyStabilize();
//		pilot_cmd_pub.publish(pilot_msg);

		last_time = current_time;
		ros::spinOnce();
		ros::Rate(rate).sleep();
	}

}

void Control::zStabilize (){

    double error = goalPosition.position.z - currentPosition.position.z; // P-error
    error = constrain(error, -0.5, 0.5);
//    double d_part = constrain(estimatedZVel, -50, 50); // D-part

    //update PID, sending velocity as D-part
    pidPosition.z = pid_z.computeCommand(error, current_time-last_time);

    // check throttle value for min\max
    pidPosition.z = constrain(pidPosition.z, -0.4, 0.4);

    pilot_msg.throttle = pidPosition.z;

}

void Control::xyStabilize (){

    ros::Duration delta_t = current_time - last_time;
    if (oldPosition.position.x == currentPosition.position.x && oldPosition.position.y == currentPosition.position.y) return;

//    double x_error = currentPosition.x - goalPosition.x;
//    double y_error = currentPosition.y - goalPosition.y;

    // transform postition errors to robot body
    double x_error=(currentPosition.position.x - goalPosition.position.x)*cos(currentPosition.orientation.z) +
    		(currentPosition.position.y - goalPosition.position.y)*sin(currentPosition.orientation.z);
    double y_error=(currentPosition.position.x - goalPosition.position.x)*sin(currentPosition.orientation.z)*(-1.0) +
    		(currentPosition.position.y - goalPosition.position.y)*cos(currentPosition.orientation.z);

    x_error = constrain(x_error, -5, 5);
    y_error = constrain(y_error, -5, 5);
//    ROS_INFO ("%f %f ", x_error, y_error);

    // transfer velocity to robot body
    double x_vel_current=((currentPosition.position.x - oldPosition.position.x)*cos(currentPosition.orientation.z) +
    		(currentPosition.position.y - oldPosition.position.y)*sin(currentPosition.orientation.z))/delta_t.toSec();
    double y_vel_current=((currentPosition.position.x - oldPosition.position.x)*sin(currentPosition.orientation.z)*(-1.0) +
    		(currentPosition.position.y - oldPosition.position.y)*cos(currentPosition.orientation.z))/delta_t.toSec();
    ROS_INFO ("%f %f ", x_vel_current, y_vel_current);

    // get goals velocities as result of pid, based on position errors.
    double x_vel_goal = pid_velocity_goal_x.computeCommand(x_error, delta_t);
    double y_vel_goal = pid_velocity_goal_y.computeCommand(y_error, delta_t);

    x_vel_goal = constrain(x_vel_goal, -0.8, 0.8);
    y_vel_goal = constrain(y_vel_goal, -0.8, 0.8);
//     ROS_INFO ("%f %f %f %f", x_error, y_error, x_vel_goal, y_vel_goal);

//    double x_vel_current = (currentPosition.x - oldPosition.x) / delta_t.toSec();
//    double y_vel_current = (currentPosition.y - oldPosition.y) / delta_t.toSec();
//    ROS_INFO ("%f %f ", currentVelocity.x, currentVelocity.y);
	// control pitch and roll as result of "velocity" PID
    pilot_msg.pitch = pid_x.computeCommand(x_vel_current - x_vel_goal, delta_t);
    pilot_msg.roll = pid_y.computeCommand(y_vel_current - y_vel_goal, delta_t);
//    ROS_INFO ("%f %f ", -(currentVelocity.y - y_vel_goal), -(currentVelocity.x - x_vel_goal));

    pilot_msg.pitch = constrain(pilot_msg.pitch, -3, 3);
    pilot_msg.roll = constrain(pilot_msg.roll, -3, 3);

    oldPosition = currentPosition;

}


void Control::odometryCallback (const nav_msgs::OdometryPtr &odom){
	currentPosition.position.x = odom->pose.pose.position.x;
	currentPosition.position.y = odom->pose.pose.position.y;
	currentPosition.position.z = odom->pose.pose.position.z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(currentPosition.orientation.x, currentPosition.orientation.y, currentPosition.orientation.z);

	currentVelocity.x = odom->twist.twist.linear.x;
	currentVelocity.y = odom->twist.twist.linear.y;
}

void Control::goalPositionCallback (const geometry_msgs::Vector3Ptr &goal){
	goalPosition.position.x = goal->x;
	goalPosition.position.y = goal->y;
	goalPosition.position.z = goal->z;
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
