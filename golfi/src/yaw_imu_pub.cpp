#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265

double yaw;
sensor_msgs::Imu imu;

void imuCallback(const sensor_msgs::Imu &imu_msg)
{
	imu = imu_msg;
  // yaw = tf::getYaw(imu.orientation);
	yaw = yaw = (long double) atan2(2 * (imu.orientation.w * imu.orientation.z + imu.orientation.x * imu.orientation.y), 1 - 2 * (imu.orientation.y*imu.orientation.y + imu.orientation.z*imu.orientation.z));
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "yaw_imu_pub");
    ros::NodeHandle n;
		ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
    ros::Publisher yaw_imu_pub = n.advertise<geometry_msgs::Twist>("yaw_imu", 10);
    geometry_msgs::Twist yaw_imu;
		ros::Rate loop_rate(50);

    while(ros::ok()) {
      yaw_imu.linear.x = yaw;
  		yaw_imu_pub.publish(yaw_imu);
  		ros::spinOnce();
			loop_rate.sleep();
    }
    return 0;
}
