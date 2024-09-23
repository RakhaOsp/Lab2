// src/snake_movement_node.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "snake_movement_node");
  ros::NodeHandle nh;

  // Create publishers for each joint
  std::vector<ros::Publisher> publishers;
  publishers.push_back(nh.advertise<std_msgs::Float64>("/motortom2m/command", 10)); // Joint 2
  publishers.push_back(nh.advertise<std_msgs::Float64>("/joint2/command", 10));   // Joint 2
  publishers.push_back(nh.advertise<std_msgs::Float64>("/joint4/command", 10));   // Joint 4
  publishers.push_back(nh.advertise<std_msgs::Float64>("/joint6/command", 10));   // Joint 6
  publishers.push_back(nh.advertise<std_msgs::Float64>("/end/command", 10));      // End-effector

  ros::Rate rate(10);  // 10 Hz
  double amplitude = 0.2;
  double frequency = 0.25;  // Hz

  while (ros::ok()) {
    double time = ros::Time::now().toSec();

    // Send sinusoidal commands to each joint with a phase offset to simulate snake-like motion
    for (size_t i = 0; i < publishers.size(); ++i) {
      double value = amplitude * sin(2 * M_PI * frequency * time + (i * M_PI / 5));
      std_msgs::Float64 msg;
      msg.data = value;
      publishers[i].publish(msg);

      // Optionally log the values
      ROS_INFO("Publishing to %s: %f", publishers[i].getTopic().c_str(), value);
    }

    rate.sleep();
  }

  return 0;
}

