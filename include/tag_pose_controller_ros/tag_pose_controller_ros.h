#ifndef TAG_POSE_CONTROLLER_ROS_H
#define TAG_POSE_CONTROLLER_ROS_H

#include <ros/ros.h>
#include <tag_pose_controller/tag_pose_controller.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Core>


class TagPoseControllerNode {

public:

    TagPoseControllerNode(std::string ns);
    ~TagPoseControllerNode();
    void TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);
    bool spin();

private:
    tf::TransformListener listener;
    tf::StampedTransform cam_in_base;
    tf::Pose tf_pose_in;
    tf::Pose tf_pose_out;
    ros::NodeHandle private_nh;
    TagPoseController controller_;
    ros::Subscriber tag_pose_subscriber;
    ros::Time now;
    double Kp_, Ki_, Kd_, upper_, lower_, sampling_time_, scaling_factor_;
};


#endif
