#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu; //bikin variabel message Imu
double vth, yaw;

void imuCallback(const sensor_msgs::Imu &imu_msg) //copy message imu dari /android/imu ke variabel yang udah dibikin
{
	imu = imu_msg;
  yaw = tf::getYaw(imu.orientation);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "imu_data");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Subscriber imu_sub = n.subscribe("/android/imu", 10, imuCallback);
    ros::Time current_time;

    // tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);

    //message declarations
    // geometry_msgs::TransformStamped imu_trans;
    // imu_trans.header.frame_id = "base_link";
    // imu_trans.child_frame_id = "/imu";

    while(ros::ok()) {
        current_time = ros::Time::now();

        //update transform
        // imu_trans.header.stamp  = current_time;
        // imu_trans.transform.translation.x = 0.0;
        // imu_trans.transform.translation.y = 0.0;
        // imu_trans.transform.translation.z = 0.0;
        // imu_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        //bikin message Imu yang isinya sama persis sama /android/imu, tp gapake roll pitch
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = current_time;
        imu_data.header.frame_id  = "imu_copy";

				imu_data.orientation = tf::createQuaternionMsgFromYaw(yaw);
				imu_data.angular_velocity.x = 0;
				imu_data.angular_velocity.y = 0;
				imu_data.angular_velocity.z = imu.angular_velocity.z;
				imu_data.linear_acceleration.x = 0;
				imu_data.linear_acceleration.y = 0;
				imu_data.linear_acceleration.z = 0;

        // publishing the imu message and the new tf
    		// broadcaster.sendTransform(imu_trans);
    		imu_pub.publish(imu_data);
    		ros::spinOnce();
    		loop_rate.sleep();
    }
    return 0;
}
