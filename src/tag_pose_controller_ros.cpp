#include <tag_pose_controller_ros/tag_pose_controller_ros.h>

#define DEBUG_INFO

void TagPoseControllerNode::TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    apriltags2_ros::AprilTagDetection  test;
    if (!msg->detections.empty()){
    test = msg->detections.back();

    geometry_msgs::Pose tag_in_camera;
    geometry_msgs::Pose tag_in_base;

    double translation_x,translation_y,translation_z;

    tag_in_camera= test.pose.pose.pose;

    tf::poseMsgToTF(tag_in_camera, tf_pose_in);

    tf_pose_out = cam_in_base * tf_pose_in;

    tf::poseTFToMsg(tf_pose_out, tag_in_base);

    tf::Quaternion quat(tag_in_base.orientation.x,tag_in_base.orientation.y,tag_in_base.orientation.z,tag_in_base.orientation.w);
    tf::Matrix3x3 rotation_matrix(quat);
    double roll,pitch,yaw;
    rotation_matrix.getRPY(roll,pitch,yaw);

#ifdef DEBUG_INFO
    ROS_INFO("roll: %f pitch: %f yaw: %f", roll,pitch,yaw);
#endif

    Eigen::Vector3d position(tag_in_base.position.x, tag_in_base.position.y, tag_in_base.position.z);
    Eigen::Vector3d rotation(roll,pitch,yaw);

    controller_.setLastDetectedPose(position, rotation);

    controller_.calculateDistance();
    double distance = controller_.getDistance();
#ifdef DEBUG_INFO
    ROS_INFO("Distance from base_link is: %f", distance);
#endif
}
    else{
      ROS_INFO("No Tag Detected");
    }
}

TagPoseControllerNode::TagPoseControllerNode(std::string ns) :
 private_nh("~/"+ns)
{     try{
    now = ros::Time(0);
    listener.waitForTransform("dory/base_link", "dory/camera_link",now, ros::Duration(10));
    listener.lookupTransform("dory/base_link", "dory/camera_link", now, cam_in_base);
    }
        catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    private_nh.param("Kp_gain", Kp_, 0.01);
    private_nh.param("Ki_gain", Ki_, 0.01);
    private_nh.param("Kd_gain", Kd_, 0.0);
    private_nh.param("upper_limit", upper_, 1.0);
    private_nh.param("lower_limit", lower_, 1.0);
    private_nh.param("sampling_time", sampling_time_, 0.1);
    private_nh.param("scaling_factor", scaling_factor_, 1.0);

    controller_.setParams(Kp_, Ki_, Kd_, upper_, lower_, sampling_time_, scaling_factor_);

}

TagPoseControllerNode::~TagPoseControllerNode(){}

bool TagPoseControllerNode::spin(){

    tag_pose_subscriber =  private_nh.subscribe("/tag_detections", 1000, &TagPoseControllerNode::TagPoseCallBack, this );

    while (private_nh.ok())
    {
      ros::spinOnce();
    }

    ROS_INFO("Exit Success");
    return true;

}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"tag_pose_controller_node");
 
    std::string ns = "visual_servoing";
     
    TagPoseControllerNode node(ns);

    node.spin();

    return EXIT_SUCCESS;

}
