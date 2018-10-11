#ifndef TAG_POSE_CONTROLLER_ROS_H
#define TAG_POSE_CONTROLLER_ROS_H

#include <ros/ros.h>
#include <tag_pose_controller/tag_pose_controller.h>

class TagPoseControllerNode {

public:

    TagPoseControllerNode();
    ~TagPoseControllerNode();
    void TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);
    bool spin();

private:

    ros::NodeHandle node_;
    TagPoseController controller;
    ros::Subscriber tag_pose_subscriber;
};


#endif
