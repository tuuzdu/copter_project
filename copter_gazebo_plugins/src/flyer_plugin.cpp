#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include <control_toolbox/pid.h>
#include <boost/bind.hpp>
#include <copter_msgs/PilotCommands.h>
#include "wind.hpp"

namespace gazebo {
	class FlyerPlugin : public ModelPlugin{

		ros::Time current_time, last_time;
		croc::Wind wind;

		public: FlyerPlugin() {
			// Init ROS
			std::string name = "flyer_plugin_node";
			int argc = 0;
			ros::init(argc, NULL, name);
			ROS_INFO("Creat flyer_plugin_node");
		}

		public: ~FlyerPlugin() {
			delete this->node;
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			this->model = _parent;
			this->node = new ros::NodeHandle("~");

			// Init ROS Subscribers and publishers
			this->subCmd = this->node->subscribe("/pilot_command", 100, &FlyerPlugin::cmdCallback, this);

			// Get Gait Settings From Parameter Server
//			node->param("damping", damping, );
//			node->param("yaw_speed_abstract", yaw_speed_abstract, 50.0);

	        // init wind (now with zero forces)
	        wind.initWind(math::Vector3(1, 1, 1), 5, 15, 1, 2 , 0.1, 0.2, 1, 2);

//	    	hover_throttle = 6.9;

	    	z_vel_goal = 0;
	        pitch_goal = 0;
	        roll_goal = 0;
	        yaw_goal = 90;  // spawn yaw
	        engine_power = 0;

	        //init pids
	        pitch_pid.initPid(120, 0, 40, 100, -100);
	        roll_pid.initPid(120, 0, 40, 100, -100);
	        yaw_pid.initPid(4, 0.4, 2, 0.2, -0.2);
	        z_velocity_pid.initPid(-2, -0.5, 0.1, 0.2, -0.2);

	        ros::WallDuration(1).sleep();

			last_time = ros::Time::now();

	        // max cmd values init
	        max_pitch_roll_cmd=3000.0;
	        max_yaw_cmd=30.0;

			ros::WallDuration(0.01).sleep();

	        // Listen to the update event. This event is broadcast every
	        // simulation iteration.
	        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	                   boost::bind(&FlyerPlugin::OnExternalUpdate, this));
		}

	    // Called by the world update start event
	    public: void OnExternalUpdate() {
	        // get current model pose and get current pitch, roll and yaw
	        math::Pose current_pose = this->model->GetRelativePose();
	        current_yaw_deg = rad_to_deg(current_pose.rot.GetYaw());
	        double current_pitch = rad_to_deg(current_pose.rot.GetPitch());
	        double current_roll = rad_to_deg(current_pose.rot.GetRoll());
	        math::Vector3 linear_vel = this->model->GetRelativeLinearVel();
	        double current_z_vel = linear_vel.z;

	        this->model->GetChildLink("copter::frame")->SetLinearDamping (0.001);


	        current_time = ros::Time::now();
	        if (engine_power){
	        	hover_throttle = 6.8;
	        // emulate controller to work with frequency of 300Hz
	        if ((current_time-last_time).toSec()>0.003){

	            ros::Duration delta_t = current_time-last_time;
	            last_time = current_time;

	            // get pitch/roll/yaw commands as values of pids
	            pitch_cmd = pitch_pid.computeCommand(current_pitch - pitch_goal, delta_t);
	            roll_cmd = roll_pid.computeCommand(current_roll - roll_goal, delta_t);
	            yaw_cmd = yaw_pid.computeCommand(current_yaw_deg - yaw_goal, delta_t);
	            z_vel_cmd = z_velocity_pid.computeCommand(current_z_vel - z_vel_goal, delta_t);

//	            ROS_INFO ("%f %f %f", z_vel_cmd, current_z_vel, z_vel_goal);
//	            ROS_INFO ("%f %f %f", pitch_cmd, roll_cmd, yaw_cmd);
//	            ROS_INFO ("%f %f %f", current_pitch, current_roll, current_yaw_deg);

	            if(pitch_cmd>max_pitch_roll_cmd) pitch_cmd = max_pitch_roll_cmd;
	            if(pitch_cmd<-max_pitch_roll_cmd) pitch_cmd = -max_pitch_roll_cmd;
	            if(roll_cmd>max_pitch_roll_cmd) roll_cmd = max_pitch_roll_cmd;
	            if(roll_cmd<-max_pitch_roll_cmd) roll_cmd = -max_pitch_roll_cmd;
	            if(yaw_cmd>max_yaw_cmd) yaw_cmd = max_yaw_cmd;
	            if(yaw_cmd<-max_yaw_cmd) yaw_cmd = -max_yaw_cmd;
	            if(z_vel_cmd>max_pitch_roll_cmd) z_vel_cmd = max_pitch_roll_cmd;
	            if(z_vel_cmd<-max_pitch_roll_cmd) z_vel_cmd = -max_pitch_roll_cmd;

	        }

	    	// pitch and roll command values are relative forces wich added (or substitite) to main throttle on propelleres platform (pretty fair)
	    	this->model->GetChildLink("copter::fake_motor_front")->AddRelativeForce(math::Vector3(0.0, 0.0, hover_throttle+z_vel_cmd+pitch_cmd));
	    	this->model->GetChildLink("copter::fake_motor_back")->AddRelativeForce(math::Vector3(0.0, 0.0, hover_throttle+z_vel_cmd-pitch_cmd));
	    	this->model->GetChildLink("copter::fake_motor_left")->AddRelativeForce(math::Vector3(0.0, 0.0, hover_throttle+z_vel_cmd-roll_cmd));
	    	this->model->GetChildLink("copter::fake_motor_right")->AddRelativeForce(math::Vector3(0.0, 0.0, hover_throttle+z_vel_cmd+roll_cmd));

//			// yaw cmd is simply relative angular velocity (not so fair, but ok)
			this->model->GetChildLink("copter::frame")-> SetAngularVel(math::Vector3(0.0, 0.0, -yaw_cmd/50.0));

	        // Add WIND
	        // Wind - is relative force, periodic in strength and direction
	        // This force applies to center of robot
	        math::Vector3 currentWind = wind.getCurrentWind();
	        this->model->GetChildLink("copter::frame")->AddRelativeForce(currentWind);
	        }
	        else hover_throttle = 0;
	    }

	    // command callback
	    private: void cmdCallback (const copter_msgs::PilotCommandsConstPtr msg)  {
	        // get new main throttle & pitch\roll\yaw goals
	    	z_vel_goal = msg->throttle;
	        pitch_goal = msg->pitch;
	        roll_goal = msg->roll;
	        yaw_goal = current_yaw_deg+msg->yaw;
	        engine_power = msg->engine_power;
//	        ROS_INFO ("%f", z_vel_goal);
	    }

	    // radians to degree transformation
	    private: double rad_to_deg(double rad)
	    {
	          return rad*57.2957795;
	    }

		private: physics::ModelPtr model;
		private: ros::NodeHandle* node;
	    private: event::ConnectionPtr updateConnection;

	    private: double max_pitch_roll_cmd; // max pitch and roll command (force)
	    private: double max_yaw_cmd;        // max yaw command (force)


	    private: double throttle;           // current main throttle
	    private: double pitch_goal;         // curent pithc goal
	    private: double roll_goal;          // current roll goal
	    private: double yaw_goal;           // current yaw goal relative to current yaw position
	    private: double current_yaw_deg;    // current yaw position
	    private: double z_vel_goal;

	    double pitch_cmd;                   // pitch command (force)
	    double roll_cmd;                    // roll command (force)
	    double yaw_cmd;                     // yaw command (force)
	    double z_vel_cmd;
	    double hover_throttle;
	    bool engine_power;

//	    private: double damping, yaw_speed_abstract;

	    private: control_toolbox::Pid pitch_pid;
	    private: control_toolbox::Pid roll_pid;
	    private: control_toolbox::Pid yaw_pid;
	    private: control_toolbox::Pid z_velocity_pid;

	    // ROS Subscribers and Publishers
	    ros::Subscriber subCmd;
};

GZ_REGISTER_MODEL_PLUGIN(FlyerPlugin)
}
