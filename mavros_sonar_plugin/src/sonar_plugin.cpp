
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace mavplugin {

class SonarPlugin : public MavRosPlugin {
public:
	SonarPlugin() :
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		sonar_nh = ros::NodeHandle(nh, "sonar");
		sonar_nh.param<std::string>("frame_id", frame_id, "odom");
		sonar_data_pub = sonar_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("data", 10);
	}

	std::string const get_name() const {
		return "Sonar";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_RANGEFINDER, &SonarPlugin::handle_rangefinder)
		};
	}

private:
	UAS *uas;

	ros::NodeHandle sonar_nh;
	ros::Publisher sonar_data_pub;

	std::string frame_id;

	void handle_rangefinder(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_rangefinder_t rngfnd;
		mavlink_msg_rangefinder_decode(msg, &rngfnd);

//		ROS_INFO("Sonar data (m): %1.3f",
//				rngfnd.distance);

		geometry_msgs::PoseWithCovarianceStampedPtr pose_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

		pose_msg->header.frame_id = frame_id;
		pose_msg->header.stamp = ros::Time::now();
		pose_msg->pose.pose.position.z = rngfnd.distance;
		sonar_data_pub.publish(pose_msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SonarPlugin, mavplugin::MavRosPlugin)
