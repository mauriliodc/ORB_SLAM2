#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/eigen.hpp>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<sensor_msgs/JointState.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include<opencv2/core/core.hpp>
#include <tf_conversions/tf_eigen.h>
#include"../../../include/System.h"

std::string rgbTopic;
std::string depthTopic;
std::string jointTopic;
std::string tf_name;
std::string tf_origin;
std::string vocabulary;
std::string orb_settings;
ORB_SLAM2::System* SLAM;


void callback(const sensor_msgs::Image::ConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    cv_ptrD = cv_bridge::toCvShare(msgD);

    cv::Mat t = SLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Eigen::Isometry3f iso_transform;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mat2eigen;
    cv::cv2eigen(t,mat2eigen);
    iso_transform.linear() = mat2eigen.block<3,3>(0,0);
    iso_transform.translation() = mat2eigen.block<3,1>(0,3);
    iso_transform = iso_transform.inverse();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::transformEigenToTF((Eigen::Isometry3d)iso_transform,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_origin, tf_name));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "so3node",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    //SPARRAMITERS STUFF
    n.param<std::string>("vocabulary",vocabulary, "");
    n.param<std::string>("settings",orb_settings, "");
    n.param<std::string>("rgb",rgbTopic, "/camera/rgb/image_raw");
    n.param<std::string>("tf",tf_name, "camera_pose");
    n.param<std::string>("origin",tf_origin, "camera_link");
    n.param<std::string>("depth",depthTopic, "/camera/depth/image_raw");
    n.param<std::string>("joints",jointTopic, "/joint_state");

    std::cout << "_vocabulary:= "<<vocabulary<<std::endl;
    std::cout << "_settings:= "<<orb_settings<<std::endl;
    std::cout << "_rgb:= "<<rgbTopic<<std::endl;
    std::cout << "_tf:= "<<tf_name<<std::endl;
    std::cout << "_origin:= "<<tf_origin<<std::endl;
    std::cout << "_depth:= "<<depthTopic<<std::endl;
    std::cout << "_joints:= "<<jointTopic<<std::endl;


    //ORB INIT
    SLAM = new ORB_SLAM2::System(vocabulary,orb_settings,ORB_SLAM2::System::RGBD,false);
    ROS_INFO("mangez merde");
    //GO MESSAGE FILTER, GO
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, rgbTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, depthTopic, 1);
    message_filters::Subscriber<sensor_msgs::JointState> joint_sub(n, jointTopic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10),rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(callback,_1,_2));
    ros::spin();


    return 0;
}
