#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener &listener) {
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped odom_point;
    odom_point.header.frame_id = "odom";

    //we'll just use the most recent transform available for our simple example
    odom_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    // odom_point.point.x = 1.0;
    // odom_point.point.y = 2.0;
    // odom_point.point.z = 0.0;

    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", odom_point, base_point);

    ROS_INFO("odom: (%.2f, %.2f, %.2f) ---> base_link: (%.2f, %.2f, %.2f) at time %.2f ",
    odom_point.point.x, odom_point.point.y, odom_point.point.z,
    base_point.point.x, base_point.point.y, base_point.point.z,
    base_point.header.stamp.toSec());

    //ROS_ERROR("Received an exception trying to transform a point from
    //\"base_laser\" to \"base_link\": %s", ex.what());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle n;

    tf::TransformListener listener(ros::Duration(10));

    //we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();
}
