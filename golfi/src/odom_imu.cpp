#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu;
double yaw;

void imuCallback(const sensor_msgs::Imu &imu_msg)
{
	imu = imu_msg;
	yaw = tf::getYaw(imu.orientation);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber imu_sub = n.subscribe("/android/imu", 10, imuCallback);
    ros::Time new_time;
		ros::Time old_time = ros::Time::now();
		ros::Duration dt_ros;

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);

    //message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

		double x = 0;
		double y = 0;
		double vx = 0;
		double vy = 0;
		double dt;

    while(ros::ok()) {
        new_time = ros::Time::now();
				dt_ros = new_time - old_time;
				dt = dt_ros.toSec();

				vx = imu.linear_acceleration.x * dt;
				vy = imu.linear_acceleration.y * dt;
				x += vx * dt;
				y += vy * dt;

        //update transform
        odom_trans.header.stamp  = new_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        //filling the odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = new_time;
        odom.header.frame_id  = "odom";
        odom.child_frame_id = "base_link";

        //position
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        odom.pose.covariance[0] = (1e-3);
        odom.pose.covariance[7] = (1e-3);
        odom.pose.covariance[14] = (1e-6);
        odom.pose.covariance[21] = (1e-6);
        odom.pose.covariance[28] = (1e-6);
        odom.pose.covariance[35] = (1e-3);
        //velocity
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
    		odom.twist.twist.linear.z = 0.0;
    		odom.twist.twist.angular.x = 0.0;
    		odom.twist.twist.angular.y = 0.0;
    		odom.twist.twist.angular.z = imu.angular_velocity.z;
    		odom.twist.covariance[0] = 1e-3;
    		odom.twist.covariance[7] = 1e-3;
    		odom.twist.covariance[14] = 1e-3;
    		odom.twist.covariance[21] = 1e-3;
    		odom.twist.covariance[35] = 1e-3;
				old_time = new_time;

        // publishing the odometry and the new tf
    		broadcaster.sendTransform(odom_trans);
    		odom_pub.publish(odom);
    		ros::spinOnce();
    		loop_rate.sleep();
    }
    return 0;
}
