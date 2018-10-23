#include <tag_pose_controller_ros/tag_pose_controller_ros.h>

#define DEBUG_HEADING_INFO
#define DEBUG_DISTANCE_INFO

void TagPoseControllerNode::TagPoseCallBack(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    apriltags2_ros::AprilTagDetection  test;
    if (!msg->detections.empty()){
    test = msg->detections.back();

    geometry_msgs::Pose tag_in_camera;

    geometry_msgs::Pose tag_in_base;

    double translation_x,translation_y,translation_z;

    tag_in_camera= test.pose.pose.pose;

    double theta_x_axis = atan2(tag_in_camera.position.z , tag_in_camera.position.x );

    int mult = 1;

    if (theta_x_axis < 0)
    {
    mult = -1;
    }

    double heading_z_axis = mult * (1.5708 - theta_x_axis);

    tf::poseMsgToTF(tag_in_camera, tf_pose_in);

    tf_pose_out = cam_in_base * tf_pose_in;

    tf::poseTFToMsg(tf_pose_out, tag_in_base);


    Eigen::Vector3d position(tag_in_base.position.x, tag_in_base.position.y, tag_in_base.position.z);
    controller_linear_.setLastDetectedPose(position);
    controller_linear_.calculateDistance();

    double measured_distance = controller_linear_.getDistance();
    double measured_angle    = heading_z_axis;

    double control_signal_linear = 0.0;
    double control_signal_angular = 0.0;

    controller_linear_.calculatePidOutput(control_signal_linear, desired_distance_, measured_distance);
    controller_angular_.calculatePidOutput(control_signal_angular, measured_angle, desired_angle_);


#ifdef DEBUG_DISTANCE_INFO
    ROS_INFO("********************");
    ROS_INFO("Measured distance: %f", measured_distance);
    ROS_INFO("Desired distance: %f", desired_distance_);
    ROS_INFO("Control signal linear: %f", control_signal_linear);
#endif

#ifdef DEBUG_HEADING_INFO
    ROS_INFO("********************");
    ROS_INFO("Measured angle: %f", heading_z_axis);
    ROS_INFO("Desired angle: %f", desired_angle_);
    ROS_INFO("Control signal angular: %f", control_signal_angular);
#endif

    cmd_vel.linear.x  = control_signal_linear;
    cmd_vel.linear.y  = 0.0;
    cmd_vel.linear.z  = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = control_signal_angular;

    cmd_vel_publisher_.publish(cmd_vel);

}
    else{
      ROS_INFO("No Tag Detected");
    }
}

TagPoseControllerNode::TagPoseControllerNode() :
 private_nh("")
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

    private_nh.param("Kp_gain_ang", Kp_ang_, 0.01);
    private_nh.param("Ki_gain_ang", Ki_ang_, 0.01);
    private_nh.param("Kd_gain_ang", Kd_ang_, 0.0);
    private_nh.param("upper_limit_ang", upper_ang_, 0.176);
    private_nh.param("lower_limit_ang", lower_ang_, -0.176);
    private_nh.param("sampling_time_ang", sampling_time_ang_, 0.1);
    private_nh.param("scaling_factor_ang", scaling_factor_ang_, 1.0);

    private_nh.param("desired_distance", desired_distance_, 1.3);
    private_nh.param("desired_angle", desired_angle_, 0.0);

    controller_linear_.setParams(Kp_, Ki_, Kd_, upper_, lower_, sampling_time_, scaling_factor_);
    controller_angular_.setParams(Kp_ang_, Ki_ang_, Kd_ang_, upper_ang_, lower_ang_, sampling_time_ang_, scaling_factor_ang_);

}

TagPoseControllerNode::~TagPoseControllerNode(){}

bool TagPoseControllerNode::spin(){

    tag_pose_subscriber =  private_nh.subscribe("/tag_detections", 1000, &TagPoseControllerNode::TagPoseCallBack, this );
    cmd_vel_publisher_   =  private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

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
 
     
    TagPoseControllerNode node;

    node.spin();

    return EXIT_SUCCESS;

}
