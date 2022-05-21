#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>

// ros::init(argc, argv, "gps_data");
// ros::NodeHandle n;
// ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data", 10);
sensor_msgs::NavSatFix gps; //bikin variabel message gps

void gpsCallback(const sensor_msgs::NavSatFix &gps_msg) //copy message gps dari /android/fix ke variabel yang udah dibikin
{
	gps = gps_msg;
	// gps.header.frame_id = "gps_copy";
	// gps_pub.publish(gps);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "gps_data");
    ros::NodeHandle n;
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data", 10);
    ros::Subscriber gps_sub = n.subscribe("/fix", 10, gpsCallback);
    ros::Time current_time;

    // tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(5);

    while(ros::ok()) {
        current_time = ros::Time::now();

      	//bikin message gps yang isinya sama persis sama /android/fix
        sensor_msgs::NavSatFix gps_data;
				gps_data = gps;
        gps_data.header.stamp = gps.header.stamp;
        gps_data.header.frame_id  = "fix_baru";

				// gps_data.status = gps.status;
				// gps_data.latitude = gps.latitude;
				// gps_data.longitude = gps.longitude;
				// gps_data.altitude = gps.altitude;
				// gps_data.position_covariance = gps.position_covariance;
				// gps_data.position_covariance_type = gps.position_covariance_type;

        // publishing the gps message
    		gps_pub.publish(gps_data);
    		ros::spinOnce();
    		loop_rate.sleep();
    }
    return 0;
}
