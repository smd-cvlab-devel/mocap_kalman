#ifndef _mocap_kalman_hpp
#define _mocap_kalman_hpp

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistWithCovariance.h>

#define WINDOW_SIZE 50

namespace mocap_kalman
{
	class MocapKalman : public nodelet::Nodelet
	{
	public:
		MocapKalman( );
		~MocapKalman( );
		virtual void onInit( );
	private:
		void spin( );
		void spinOnce( );

		// ROS Interface
		ros::NodeHandle nh;
		ros::Publisher odom_pub;
		ros::Rate r;
		tf::TransformListener li;

		// Previous iteration
		tf::StampedTransform last_transform;
		tf::Quaternion last_quat;
		geometry_msgs::TwistWithCovariance last_twist;
		geometry_msgs::TwistWithCovariance last_delta_twist;

		// Kalman State Variable
		geometry_msgs::TwistWithCovariance xdot;

		// Parameters
		bool local_frame;
		double linear_process_variance;
		double angular_process_variance;
		double linear_observation_variance;
		double angular_observation_variance;
		std::string frame_base;
		std::string frame_id;

		boost::thread spin_thread;
	};
}

#endif /* _mocap_kalman_hpp */
