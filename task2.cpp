#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "square_wave_publisher_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/end/command", 10);
  ros::Rate rate(1);  // 1 Hz
  double switch_time = 5.0;  // seconds

  double start_time = ros::Time::now().toSec();
  
  while (ros::ok()) {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    double value = (static_cast<int>(elapsed_time / switch_time) % 2 == 0) ? 1.0 : 0.0;

    std_msgs::Float64 msg;
    msg.data = value;
    pub.publish(msg);

    ROS_INFO("Publishing: %f", msg.data);
    rate.sleep();
  }

  return 0;
}
