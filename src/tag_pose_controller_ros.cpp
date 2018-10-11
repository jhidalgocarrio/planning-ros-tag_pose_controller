#include <tag_pose_controller_ros/tag_pose_controller_ros.h>


void TagPoseControllerNode::TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    apriltags2_ros::AprilTagDetection  test;
    if (!msg->detections.empty()){
    test = msg->detections.back();
    controller.setLastDetectedPose(test);
    controller.calculateDistance();
    double distance = controller.getDistance();
    ROS_INFO("%f", distance);
    controller.calculateRPY();
    geometry_msgs::Vector3 rpy = controller.getRPY();
    ROS_INFO("roll:,%f pitch:,%f yaw:,%f", rpy.x,rpy.y,rpy.z);
    }
    else{
      ROS_INFO("No Tag Detected");
    }
}

TagPoseControllerNode::TagPoseControllerNode() :
 node_("~")
{

}

TagPoseControllerNode::~TagPoseControllerNode(){

}

bool TagPoseControllerNode::spin(){

    tag_pose_subscriber =  node_.subscribe("/tag_detections", 1000, &TagPoseControllerNode::TagPoseCallBack, this );

    while (node_.ok())
    {
      ros::spinOnce();
    }

    ROS_INFO("Exit Success");
    return true;

}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"tag_pose_controller_node");

    TagPoseControllerNode node;

    node.spin();

    return EXIT_SUCCESS;

}
