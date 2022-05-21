#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <math.h>

// ros::init(argc, argv, "gps_data");
// ros::NodeHandle n;
// ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data", 10);
sensor_msgs::Imu imu; //bikin variabel message gps
float yaw;
void imuCallback(const sensor_msgs::Imu &imu_msg) //copy message gps dari /android/fix ke variabel yang udah dibikin
{
	imu = imu_msg;
	yaw = atan2(2 * (imu.orientation.w * imu.orientation.z + imu.orientation.x * imu.orientation.y), 1 - 2 * (pow(imu.orientation.y, 2) + pow(imu.orientation.z, 2)));
	// yaw_pub.publish(yaw);
	// gps.header.frame_id = "gps_copy";
	// gps_pub.publish(gps);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "plot_yaw");
    ros::NodeHandle n;
    ros::Publisher yaw_pub = n.advertise<std_msgs::Float32>("yaw", 10);
    ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
    ros::Time current_time;

    // tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50); //APA INI

    while(ros::ok()) {
        // current_time = ros::Time::now();

      	//bikin message gps yang isinya sama persis sama /android/fix
        // sensor_msgs::NavSatFix gps_data;
				// gps_data = gps;
        // gps_data.header.stamp = gps.header.stamp;
        // gps_data.header.frame_id  = "gps_copy";

				// gps_data.status = gps.status;
				// gps_data.latitude = gps.latitude;
				// gps_data.longitude = gps.longitude;
				// gps_data.altitude = gps.altitude;
				// gps_data.position_covariance = gps.position_covariance;
				// gps_data.position_covariance_type = gps.position_covariance_type;

        // publishing the gps message
    		yaw_pub.publish(yaw);
    		ros::spinOnce();
    		loop_rate.sleep();
    }
    return 0;
}
