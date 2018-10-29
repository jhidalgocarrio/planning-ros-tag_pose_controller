#ifndef TAG_POSE_CONTROLLER_NODE_H
#define TAG_POSE_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <tag_pose_controller/tag_pose_controller.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <cmath>
class TagPoseControllerNode {

public:

    TagPoseControllerNode();
    ~TagPoseControllerNode();
    void TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);
    bool spin();

private:

    //ros specific
    tf::TransformListener listener;
    tf::StampedTransform cam_in_base;
    tf::Pose tf_pose_in;
    tf::Pose tf_pose_out;
    ros::NodeHandle private_nh;
    ros::Subscriber tag_pose_subscriber;
    ros::Publisher  cmd_vel_publisher_;
    ros::Time now;

    //controller specific
    TagPoseController controller_linear_;
    TagPoseController controller_angular_;

    //controller parameters from Parameter Server
    double Kp_, Ki_, Kd_, upper_, lower_, sampling_time_, scaling_factor_;
    double Kp_ang_, Ki_ang_, Kd_ang_, upper_ang_, lower_ang_, sampling_time_ang_, scaling_factor_ang_;

    //user defined setpoints
    double desired_distance_;
    double desired_angle_;

    //output
    geometry_msgs::Twist cmd_vel;

};


#endif