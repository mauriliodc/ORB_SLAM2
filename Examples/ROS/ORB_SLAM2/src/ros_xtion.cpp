
//THIN_TXT_IO STUFF
#include "txt_io/message_reader.h"
#include "txt_io/message_seq_synchronizer.h"
#include "txt_io/base_image_message.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/joint_state_message.h"
#include "txt_io/message_enlister_trigger.h"
#include "globals/system_utils.h"
txt_io::MessageSeqSynchronizer synchronizer;
txt_io::MessageWriter writer;
//

//IL MALE
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

using namespace std;
using namespace txt_io;

std::string rgb_topic;
std::string depth_topic;
std::string joint_topic;
static tf::TransformBroadcaster br;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::JointStateConstPtr& j);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "XTION");
    ros::start();
    writer.open(argv[6]);
    if(argc != 7)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 XTION path_to_vocabulary path_to_settings rgb_topic depth_topic joint_topic logname.txt" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    //save stuff into things
    rgb_topic = argv[3];
    depth_topic = argv[4];
    joint_topic = argv[5];

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[3], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[4], 1);
    message_filters::Subscriber<sensor_msgs::JointState> joint_sub(nh, argv[5], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::JointState> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub, joint_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::JointStateConstPtr& j)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    cv_ptrD = cv_bridge::toCvShare(msgD);

    //get camera pose as stated in Frame.h row 164, remember to invert
    cv::Mat t = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    //txt io data
    PinholeImageMessage rgb_msg(rgb_topic, msgRGB->header.frame_id,  msgRGB->header.seq, msgRGB->header.stamp.toSec());
    PinholeImageMessage depth_msg(depth_topic, msgD->header.frame_id,  msgD->header.seq, msgD->header.stamp.toSec());

    //JOIN STATES :-----------------------------------------
    JointStateMessage joint_msg(joint_topic,j->header.frame_id,j->header.seq, j->header.stamp.toSec());
    joint_msg.joints().resize(2);
    for (size_t i=0; i<2; i++){
        JointStateMessage::JointStatus& joint =joint_msg.joints()[i];
        joint.name=j->name[i];
        joint.position=j->position[i];
        joint.velocity=j->velocity[i];
        joint.effort=j->effort[i];
    }
    //-----------------------------------------------------:
    Eigen::Isometry3f cameraTransform;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> orbT;
    cv::cv2eigen(t,orbT);
    cameraTransform.linear() = orbT.block<3,3>(0,0);
    cameraTransform.translation() = orbT.block<3,1>(0,3);
    cameraTransform = cameraTransform.inverse(); //!Important
    cv::Mat rgb = cv_ptrRGB->image.clone();
    cv::Mat d = cv_ptrD->image.clone();
    //txt io stuff
    rgb_msg.setImage(rgb);
    rgb_msg.setOdometry(cameraTransform);
    depth_msg.setImage(d);
    depth_msg.setOdometry(cameraTransform);
    writer.writeMessage(rgb_msg);
    writer.writeMessage(depth_msg);
    writer.writeMessage(joint_msg);

    tf::Transform transform;
    tf::transformEigenToTF((Eigen::Isometry3d)cameraTransform,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));

}


