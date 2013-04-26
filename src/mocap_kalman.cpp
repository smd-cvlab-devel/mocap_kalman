#include "mocap_kalman/mocap_kalman.hpp"

#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

PLUGINLIB_DECLARE_CLASS(mocap_kalman, MocapKalman, mocap_kalman::MocapKalman, nodelet::Nodelet)

namespace mocap_kalman
{
	MocapKalman::MocapKalman( )
		: r( 100 )
		, last_quat( 1, 0, 0, 0 )
		, frame_base( "map" )
		, frame_id( "trackable" )
	{
		last_delta_twist.twist.linear.x = 0;
		last_delta_twist.twist.linear.y = 0;
		last_delta_twist.twist.linear.z = 0;
		last_delta_twist.twist.angular.x = 0;
		last_delta_twist.twist.angular.y = 0;
		last_delta_twist.twist.angular.z = 0;
	}

	MocapKalman::~MocapKalman( )
	{
		spin_thread.interrupt( );
		if( spin_thread.joinable( ) )
			spin_thread.join( );
	}

	void MocapKalman::onInit( )
	{
		int rate;
		nh = getNodeHandle( );
		ros::NodeHandle nh_priv = getPrivateNodeHandle( );

		nh_priv.param( "local_frame", local_frame, true );
		nh_priv.param( "linear_process_variance", linear_process_variance, 0.01 );
		nh_priv.param( "angular_process_variance", angular_process_variance, 0.01 );
		nh_priv.param( "linear_observation_variance", linear_observation_variance, 0.2 );
		nh_priv.param( "angular_observation_variance", angular_observation_variance, 0.2 );
		nh_priv.param( "frame_base", frame_base, (std::string)"map" );
		nh_priv.param( "frame_id", frame_id, (std::string)"trackable" );
		nh_priv.param( "rate", rate, 100 );

		r = ros::Rate( rate );

		odom_pub = nh.advertise<nav_msgs::Odometry>( frame_id + "/mocap_odom", 1, false );

		spin_thread = boost::thread( &MocapKalman::spin, this );
	};

	void MocapKalman::spin( )
	{
		while( 1 )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
			r.sleep( );
		}
	}

	void MocapKalman::spinOnce( )
	{
		const static double dt = (double)r.expectedCycleTime( ).nsec / 1000000000 + r.expectedCycleTime( ).sec;

		static double K[36] = { 0 };
		static double F[36] = { 0 };
		static geometry_msgs::PoseWithCovariance residual;
		static geometry_msgs::Vector3 rpy;
		static tf::Quaternion curr_quat;
		static tf::StampedTransform tr;

		// Get the transform
		try
		{
			li.lookupTransform( frame_base, frame_id, ros::Time(0), tr );
		}
		catch( tf::TransformException ex )
		{
			ROS_INFO( "Missed a transform...chances are that we are still OK" );
			return;
		}
		if( tr.getOrigin( ).x( ) != tr.getOrigin( ).x( ) )
		{
			ROS_WARN( "NaN DETECTED" );
			return;
		}

		nav_msgs::OdometryPtr odom_msg( new nav_msgs::Odometry );

		odom_msg->header.frame_id = frame_base;
		odom_msg->header.stamp = ros::Time::now( );
		odom_msg->child_frame_id = frame_id;

		// Get the RPY from this iteration
		curr_quat = tr.getRotation( );
		tf::Matrix3x3( ( curr_quat * last_quat.inverse( ) ).normalize( ) ).getRPY(rpy.x, rpy.y, rpy.z);

		// Step 1:
		// x = F*x
		// We construct F based on the acceleration previously observed
		// We will assume that dt hasn't changed much since the last calculation
		F[0] = ( xdot.twist.linear.x < 0.001 ) ? 1 : ( xdot.twist.linear.x + last_delta_twist.twist.linear.x ) / xdot.twist.linear.x;
		F[7] = ( xdot.twist.linear.y < 0.001 ) ? 1 : ( xdot.twist.linear.y + last_delta_twist.twist.linear.y ) / xdot.twist.linear.y;
		F[14] = ( xdot.twist.linear.z < 0.001 ) ? 1 : ( xdot.twist.linear.z + last_delta_twist.twist.linear.z ) / xdot.twist.linear.z;
		F[21] = ( xdot.twist.angular.x < 0.001 ) ? 1 : ( xdot.twist.angular.x + last_delta_twist.twist.angular.x ) / xdot.twist.angular.x;
		F[28] = ( xdot.twist.angular.y < 0.001 ) ? 1 : ( xdot.twist.angular.y + last_delta_twist.twist.angular.y ) / xdot.twist.angular.y;
		F[35] = ( xdot.twist.angular.z < 0.001 ) ? 1 : ( xdot.twist.angular.z + last_delta_twist.twist.angular.z ) / xdot.twist.angular.z;

		// Step 2:
		// P = F*P*F' + Q
		// Since F is only populated on the diagonal, F=F'
		xdot.covariance[0] += linear_process_variance;
		xdot.covariance[7] += linear_process_variance;
		xdot.covariance[14] += linear_process_variance;
		xdot.covariance[21] += angular_process_variance;
		xdot.covariance[28] += angular_process_variance;
		xdot.covariance[35] += angular_process_variance;

		// Step 3:
		// y = z - H*x
		// Because x estimates z directly, H = I
		residual.pose.position.x = ( tr.getOrigin( ).x( ) - last_transform.getOrigin( ).x( ) ) / dt - xdot.twist.linear.x;
		residual.pose.position.y = ( tr.getOrigin( ).y( ) - last_transform.getOrigin( ).y( ) ) / dt - xdot.twist.linear.y;
		residual.pose.position.z = ( tr.getOrigin( ).z( ) - last_transform.getOrigin( ).z( ) ) / dt - xdot.twist.linear.z;
		// HACK for some odd discontinuity in the rotations...
		//if( rpy.x > .4 || rpy.y > .4 || rpy.z > .4 || rpy.x < -.4 || rpy.y < -.4 || rpy.z < -.4 )
		//	rpy.x = rpy.y = rpy.z = 0;
		residual.pose.orientation.x = rpy.x / dt - xdot.twist.angular.x;
		residual.pose.orientation.y = rpy.y / dt - xdot.twist.angular.y;
		residual.pose.orientation.z = rpy.z / dt - xdot.twist.angular.z;

		// Step 4:
		// S = H*P*H' + R
		// Again, since H = I, S is simply P + R
		residual.covariance[0] = xdot.covariance[0] + linear_observation_variance;
		residual.covariance[7] = xdot.covariance[7] + linear_observation_variance;
		residual.covariance[14] = xdot.covariance[14] + linear_observation_variance;
		residual.covariance[21] = xdot.covariance[21] + angular_observation_variance;
		residual.covariance[28] = xdot.covariance[28] + angular_observation_variance;
		residual.covariance[35] = xdot.covariance[35] + angular_observation_variance;

		// Step 5:
		// K = P*H'*S^(-1)
		// Again, since H = I, and since S is only populated along the diagonal,
		// we can invert each element along the diagonal
		K[0] = xdot.covariance[0] / residual.covariance[0];
		K[7] = xdot.covariance[7] / residual.covariance[7];
		K[14] = xdot.covariance[14] / residual.covariance[14];
		K[21] = xdot.covariance[21] / residual.covariance[21];
		K[28] = xdot.covariance[28] / residual.covariance[28];
		K[35] = xdot.covariance[35] / residual.covariance[35];

		// Step 6:
		// x = x + K*y
		xdot.twist.linear.x += K[0] * residual.pose.position.x;
		xdot.twist.linear.y += K[7] * residual.pose.position.y;
		xdot.twist.linear.z += K[14] * residual.pose.position.z;
		xdot.twist.angular.x += K[21] * residual.pose.orientation.x;
		xdot.twist.angular.y += K[28] * residual.pose.orientation.y;
		xdot.twist.angular.z += K[35] * residual.pose.orientation.z;

		// Step 7:
		// P = (I - K*H)*P
		// H is still I, so (I-K) * P
		xdot.covariance[0] *= -K[0];
		xdot.covariance[7] *= -K[7];
		xdot.covariance[14] *= -K[14];
		xdot.covariance[21] *= -K[21];
		xdot.covariance[28] *= -K[28];
		xdot.covariance[35] *= -K[35];

		// Populate Message
		odom_msg->pose.pose.position.x = tr.getOrigin( ).x( );
		odom_msg->pose.pose.position.y = tr.getOrigin( ).y( );
		odom_msg->pose.pose.position.z= tr.getOrigin( ).z( );
		tf::quaternionTFToMsg( tr.getRotation( ), odom_msg->pose.pose.orientation );

		odom_msg->twist = xdot;

		if( local_frame )
		{
			// Rotate velocity vector to the local frame
			tf::Vector3 tmp;
			tf::Vector3 tmp2;
			tf::vector3MsgToTF( odom_msg->twist.twist.linear, tmp );
			tf::vector3MsgToTF( odom_msg->twist.twist.angular, tmp2 );
			tf::vector3TFToMsg( tf::quatRotate( curr_quat.inverse( ), tmp ), odom_msg->twist.twist.linear );
			tf::vector3TFToMsg( tf::quatRotate( curr_quat.inverse( ), tmp2 ), odom_msg->twist.twist.angular );
		}

		// Done, Publish
		odom_pub.publish( odom_msg );

		// Record when this message was for next time
		last_delta_twist.twist.linear.x = odom_msg->twist.twist.linear.x - last_twist.twist.linear.x;
		last_delta_twist.twist.linear.y = odom_msg->twist.twist.linear.y - last_twist.twist.linear.y;
		last_delta_twist.twist.linear.z = odom_msg->twist.twist.linear.z - last_twist.twist.linear.z;
		last_delta_twist.twist.angular.x = odom_msg->twist.twist.angular.x - last_twist.twist.angular.x;
		last_delta_twist.twist.angular.y = odom_msg->twist.twist.angular.y - last_twist.twist.angular.y;
		last_delta_twist.twist.angular.z = odom_msg->twist.twist.angular.z - last_twist.twist.angular.z;
		last_transform = tr;
		last_twist = odom_msg->twist;
		last_quat = curr_quat;
	};
}
