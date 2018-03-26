#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>

#include <tf/transform_broadcaster.h>



ros::Publisher imu_pub;
ros::Publisher sonar_pub;
sensor_msgs::Imu imu_msg;
geometry_msgs::PoseWithCovarianceStamped sonar_msg;

void chatterSensor (const geometry_msgs::QuaternionConstPtr& data){
	tf::Quaternion q;
	q.setRPY(data->x, -data->y, -data->z);
	imu_msg.orientation.w = q.w();
	imu_msg.orientation.x = q.x();
	imu_msg.orientation.y = q.y();
	imu_msg.orientation.z = q.z();
//	imu_msg.orientation_covariance[0] = 0.01;
//	imu_msg.orientation_covariance[4] = 0.01;
//	imu_msg.orientation_covariance[8] = 0.01;
	imu_msg.header.frame_id = "frame";
	imu_msg.header.stamp = ros::Time::now();
	imu_pub.publish(imu_msg);

	sonar_msg.pose.pose.position.z = data->w/100;
//	sonar_msg.pose.covariance[14] = 5000;
	sonar_msg.header.frame_id = "odom";
	sonar_msg.header.stamp = ros::Time::now();
	sonar_pub.publish(sonar_msg);

	//Visualization
//	static tf::TransformBroadcaster br;
//	tf::Transform transform;
//	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//	transform.setRotation(q);
//	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "frame"));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "copter_arduino");
	ros::NodeHandle node;

	imu_pub = node.advertise<sensor_msgs::Imu>("imu_data", 1);
	sonar_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("sonar_data", 1);
//	ros::Rate loop_rate(20);
	ros::Subscriber imu_sub = node.subscribe("imu_sonar_raw", 1, chatterSensor);

	ros::spin();
	return 0;
}
