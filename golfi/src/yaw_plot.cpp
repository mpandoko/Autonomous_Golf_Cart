#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <math.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    // pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    yaw_pub = n.advertise<std_msgs::Float32>("/yaw", 10);

    //Topic you want to subscribe
    // sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    imu_sub = n.subscribe("/imu", 10, &SubscribeAndPublish::imuCallback, this);
  }

  void imuCallback(const sensor_msgs::Imu &imu_msg)
  {
    // PUBLISHED_MESSAGE_TYPE output;
    std_msgs::Float32 yaw;
    //.... do something with the input and generate the output...
    yaw.data = (float) atan2(2 * (imu_msg.orientation.w * imu_msg.orientation.z + imu_msg.orientation.x * imu_msg.orientation.y), 1 - 2 * (pow(imu_msg.orientation.y, 2) + pow(imu_msg.orientation.z, 2)));
    // imu.orientation.y = 0;
    // imu.orientation.z = 0;
    // imu.orientation.w = 0;
    yaw_pub.publish(yaw);
    // pub_.publish(output);
  }

private:
  ros::NodeHandle n;
  ros::Publisher yaw_pub;
  ros::Subscriber imu_sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "yaw_plot");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
