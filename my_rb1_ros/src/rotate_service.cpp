#include "geometry_msgs/Twist.h"
#include "my_custom_srv_msg_pkg/RotateRequest.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include <cmath>
#include <my_custom_srv_msg_pkg/Rotate.h>
#include <ros/ros.h>
#include <tf/tf.h>

class RobotRotator {

private:
  ros::Publisher twist_pub;
  ros::Subscriber odom_sub;
  ros::ServiceServer rotate_service;
  float target_yaw, curr_yaw;
  bool reached_target_yaw = false;
  geometry_msgs::Twist twist_msg;
  float yaw_tolerance = 0.1; // Degrees
  std::string rotate_service_name = "/rotate_robot";

public:
  RobotRotator(ros::NodeHandle *nh) {
    curr_yaw = 0.0;
    target_yaw = 0.0;
    twist_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    odom_sub = nh->subscribe("/odom", 10, &RobotRotator::odom_callback, this);

    rotate_service = nh->advertiseService(
        rotate_service_name, &RobotRotator::rotate_robot_callback, this);
    ROS_INFO("%s Service Ready!", rotate_service_name.c_str());
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
    int input_degrees_int = req.degrees; // Save input degrees integer
    float input_degrees_float =
        static_cast<float>(input_degrees_int); // Convert to float
    ROS_INFO("%s Service Requested: Rotate robot by %.2f degrees.",
             input_degrees_float,
             rotate_service_name.c_str()); // Service requested message

    target_yaw = normalize_angle(input_degrees_float * M_PI / 180.0 +
                                 curr_yaw); // Normalise target yaw

    ros::Rate rate(10);
    while (!reached_target_yaw && ros::ok()) {
      float yaw_diff =
          normalize_angle(target_yaw - curr_yaw); // Compute remaining yaw_diff
      if (abs(yaw_diff) > yaw_tolerance * M_PI / 180.0) {
        ROS_INFO("%s Service Status: Robot rotating... Remaining %.2f degrees.",
                 yaw_diff, rotate_service_name.c_str());
        twist_msg.angular.z = yaw_diff;
        twist_pub.publish(twist_msg);
        reached_target_yaw = false;
      } else {
        twist_msg.angular.z = 0.0; // Stop rotation
        twist_pub.publish(twist_msg);
        reached_target_yaw = true;
      }
      rate.sleep();
      ros::spinOnce();
    }
    res.result = "Rotation completed successfully.";
    ROS_INFO("%s Service Completed: Rotation complete.",
             rotate_service_name.c_str());
    ROS_INFO("%s Service Ready!", rotate_service_name.c_str());
    reached_target_yaw = false;

    return true;
  }

  float normalize_angle(float angle) {
    // Normalize angle to be within the range [-PI, PI]
    if (angle > M_PI)
      angle -= 2 * M_PI;
    if (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  void rotate_robot() {}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_robot_node");
  ros::NodeHandle nh;
  RobotRotator rr = RobotRotator(&nh);
  ros::spin();
}