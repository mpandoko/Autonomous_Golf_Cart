// udah bagusssssssss udah ditambah percepatan, kalo mau pake percepatan uncomment aja
#include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

long double wheel_radius = 0.4572;
long double v = 0;
long double vth = 0;
long double vx,vy,yaw;
sensor_msgs::Imu imu;
nav_msgs::Odometry utm;
long double x,y;
long double pi = 3.14159265359;
typedef Matrix<long double, Dynamic, Dynamic> Mat;
typedef Matrix<long double, Dynamic, 1> Vec;
Vec u(2);
Mat rot_mat(2,2);

long double wrap_angle(long double yaw) {
  while (yaw > pi) {
    yaw -= 2 * pi;
  }
  while (yaw < -pi) {
    yaw += 2 * pi;
  }
  return (yaw);
}

void tachCallback(const geometry_msgs::Twist &vel_msg) {
    // geometry_msgs::Twist twist = twist_aux;
    double vel = vel_msg.linear.x;
    ROS_INFO ("TACHO masuk");
    // ROS_INFO("tacho masuk");
    v = vel; //ini dikali 5 biar gerakan mobilnya cepet aja wkwkwk
 }

void imuCallback(const sensor_msgs::Imu &imu_msg)
{
	imu = imu_msg;
  ROS_INFO ("IMU masuk");
  yaw = tf::getYaw(imu.orientation);
  yaw = wrap_angle(yaw + pi/2);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "spll");
    ros::NodeHandle n;
    ros::Subscriber tach_sub = n.subscribe("sensor_velocity", 10, tachCallback);
    ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("spll", 10);
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(50);
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::Odometry odom;
    long double th, delta_x, delta_y, dt;
    bool init = false;

    //initial position
    x = 0.0;
    y = 0.0;
    th = 0.0;

    while(ros::ok()) {
      current_time = ros::Time::now();
      dt = (current_time - last_time).toSec();

      vx = v * cos(yaw);
      vy = v * sin(yaw);
      u << imu.linear_acceleration.x, -imu.linear_acceleration.y;
      rot_mat << cos(yaw), -sin(yaw),
                  sin(yaw), cos(yaw);
      u = rot_mat * u;

      // delta_x = vx * dt + u[0] * dt * dt / 2;
      // delta_y = vy * dt + u[1] * dt * dt / 2;
      delta_x = vx * dt;
      delta_y = vy * dt;

      x = x + delta_x;
      y = y + delta_y;

      //update transform
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.header.stamp  = current_time;
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
      // odom_trans.transform.rotation.x = imu.orientation.x;
      // odom_trans.transform.rotation.y = imu.orientation.y;
      // odom_trans.transform.rotation.z = imu.orientation.z;
      // odom_trans.transform.rotation.w = imu.orientation.w;

      //filling the odometry
      odom.header.stamp = current_time;
      odom.header.frame_id  = "odom";
      odom.child_frame_id = "base_link";

      //position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      // odom.pose.pose.orientation.x = imu.orientation.x;
      // odom.pose.pose.orientation.y = imu.orientation.y;
      // odom.pose.pose.orientation.z = imu.orientation.z;
      // odom.pose.pose.orientation.w = imu.orientation.w;

      odom.pose.covariance[0] = 0.5;
      odom.pose.covariance[7] = 0.5;
      odom.pose.covariance[14] = (1e-6);
      odom.pose.covariance[21] = 0.5;
      odom.pose.covariance[28] = 0.5;
      odom.pose.covariance[35] = 0.5;
      //velocity
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
  		odom.twist.twist.linear.z = 0.0;
  		odom.twist.twist.angular.x = 0.0;
  		odom.twist.twist.angular.y = 0.0;
  		odom.twist.twist.angular.z = 0.0;
  		odom.twist.covariance[0] = 0.5;
  		odom.twist.covariance[7] = 0.5;
  		odom.twist.covariance[14] = 1e-3;
  		odom.twist.covariance[21] = 1e-3;
      odom.twist.covariance[28] = 1e-3;
  		odom.twist.covariance[35] = 1e-3;

      // publishing the odometry and the new tf
  		broadcaster.sendTransform(odom_trans);
  		odom_pub.publish(odom);
  		ros::spinOnce();
  		loop_rate.sleep();
      last_time = current_time;
    }
    return 0;
}
