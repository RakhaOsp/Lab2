// src/listener_publisher_node.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class ListenerPublisher {
public:
  ListenerPublisher() : previous_value_(-std::numeric_limits<double>::infinity()) {
    // Initialize ROS
    ros::NodeHandle nh;

    // Subscribe to the input topic
    sub_ = nh.subscribe("/task1", 10, &ListenerPublisher::callback, this);

    // Advertise the output topic
    pub_ = nh.advertise<std_msgs::Float64>("/joint2/command", 10);
  }

private:
  void callback(const std_msgs::Float64::ConstPtr& msg) {
    double current_value = msg->data;

    if (current_value > previous_value_) {
      std_msgs::Float64 joint_cmd;
      joint_cmd.data = current_value;
      pub_.publish(joint_cmd);
      ROS_INFO("Publishing: %f", joint_cmd.data);
    }

    previous_value_ = current_value;
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  double previous_value_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener_publisher_node");
  ListenerPublisher listener_publisher;
  ros::spin();
  return 0;
}

