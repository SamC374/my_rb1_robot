#include "geometry_msgs/Twist.h"
#include "my_custom_srv_msg_pkg/RotateRequest.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <my_custom_srv_msg_pkg/Rotate.h>
#include <ros/ros.h>
#include <tf/tf.h>
#define PI 3.14159

class RobotRotator {

private:
  ros::Publisher twist_pub;
  ros::Subscriber odom_sub;
  ros::ServiceServer rotate_service;
  float target_yaw, curr_yaw;
  bool new_rot_target = true;
  geometry_msgs::Twist twist_msg;

public:
  RobotRotator(ros::NodeHandle *nh) {
    curr_yaw = 0.0;
    target_yaw = 0.0;
    twist_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    odom_sub = nh->subscribe("/odom", 10, &RobotRotator::odom_callback, this);

    rotate_service = nh->advertiseService(
        "/rotate_robot", &RobotRotator::rotate_robot_callback, this);
  }

  void odom_callback(const nav_msgs::Odometry &msg) {
    // Extract the quaternion orientation
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

    // Convert quaternion to Euler angles
    tf::Quaternion quat(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 mat(quat);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    curr_yaw = yaw;

    // ROS_INFO("curr_yaw:%.2f", curr_yaw);
  }

  bool rotate_robot_callback(my_custom_srv_msg_pkg::Rotate::Request &req,
                             my_custom_srv_msg_pkg::Rotate::Response &res) {
    int degrees_int = req.degrees;
    int hz = 10;
    ros::Rate rate(hz); // Control loop rate
    float angular_speed =
        static_cast<float>(degrees_int) * static_cast<float>(hz);
    for (int i = 0; i < abs(degrees_int); i++) {
      ROS_INFO("Elapsed yaw:%i", i);
      twist_msg.angular.z = angular_speed;
      twist_pub.publish(twist_msg);
      rate.sleep();    // Wait for the next iteration
      ros::spinOnce(); // Process incoming messages
    }
    twist_msg.angular.z = 0.0;
    twist_pub.publish(twist_msg);
    res.result = "Rotation Completed!";
    return true;
  }

  float normalize_angle(int angle) {
    // Normalize angle to be within the range [-PI, PI]
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_robot_node");
  ros::NodeHandle nh;
  RobotRotator rr = RobotRotator(&nh);
  ros::spin();
}