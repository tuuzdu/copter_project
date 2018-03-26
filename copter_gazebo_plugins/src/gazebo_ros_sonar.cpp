
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "gazebo/math/Pose.hh"
#include "math.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "copter_msgs/UltraSonicRange.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


namespace gazebo
{
  class GazeboRosSonar : public ModelPlugin
  {

    public: GazeboRosSonar()
    {

      // init ROS
      std::string name = "sonar_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }


    public: ~GazeboRosSonar()
    {
      delete this->node;
    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

          // ROS Nodehandle
        this->node = new ros::NodeHandle("~");

        this->pub_sonar = this->node->advertise<geometry_msgs::PoseWithCovarianceStamped>("/sonar_data",1000);

        last_value_changed_time = ros::Time::now();
        last_time = ros::Time::now();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                   boost::bind(&GazeboRosSonar::OnUpdate, this));

    }


    // Called by the world update start event
    public: void OnUpdate()
    {
          // get current pose of model
          math::Pose current_pose = this->model->GetRelativePose();
          current_time = ros::Time::now();

          // will work in rate ~ 50Hz
          if ((current_time-last_time).toSec()>0.02)
          {
              // publish ground distance
              // topic published in frequency of 100Hz, but sonar data is changed
              // in frequency of 10Hz only
              if ((current_time-last_value_changed_time).toSec()>0.1)
              {
                  ground_distance = current_pose.pos.z + GaussianKernel(0,0.01)/* + HangOut(100, current_pose.pos.z)*/;
                  if (ground_distance<0.01) ground_distance = 0.01;
                  last_value_changed_time = current_time;
              }

              sonar_msg.pose.pose.position.z = ground_distance;
              sonar_msg.pose.covariance[14] = 5000;
              sonar_msg.header.frame_id = "odom";
              sonar_msg.header.stamp = ros::Time::now();

              pub_sonar.publish(sonar_msg);

              last_time=current_time;
          }

     }


      // Utility for adding noise
      double GaussianKernel(double mu,double sigma)
      {
        double U = (double)rand()/(double)RAND_MAX;
        double V = (double)rand()/(double)RAND_MAX;
        double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
       // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
      }


      // hangout emulation
      // emulate mirrored ultrasonic signal
      // int tick - how often it should happen (for example 100 ticks in 10hz sonar - once in nearly 10 seconds. Bat it's random too!)
      // double CurrentHeight - current altitude of copter
      double HangOut(int tick, double currentHeight)
      {
        int v = rand() % tick+1;
        int m = rand() % 2+1; // values of 1 and 2;
        if (v==tick)
        {
            ROS_INFO("Hangout! %f", currentHeight*m);
            return currentHeight*m+0.15*m;
        }
        else
            return 0;
      }


    // times
    ros::Time current_time, last_time, last_value_changed_time;

    // Pointer to the model
    private: physics::ModelPtr model;

    private: geometry_msgs::PoseWithCovarianceStamped sonar_msg;
    private: double ground_distance;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

    ros::Publisher pub_sonar;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosSonar)
}
