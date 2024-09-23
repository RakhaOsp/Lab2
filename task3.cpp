#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "sine_wave_publisher_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/end/command", 10);
  ros::Rate rate(10);  // 10 Hz

  double amplitude = 1.0;
  double frequency = 0.1;  // Hz

  while (ros::ok()) {
    double time = ros::Time::now().toSec();
    double value = amplitude * sin(2 * M_PI * frequency * time);

    std_msgs::Float64 msg;
    msg.data = value;
    pub.publish(msg);

    ROS_INFO("Publishing: %f", msg.data);
    rate.sleep();
  }

  return 0;
}
