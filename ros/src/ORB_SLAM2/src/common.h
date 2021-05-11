#ifndef COMMON_H
#define COMMON_H

/*  The default coordinate of ORB_SLAM2 trajectory is EDN (East(x)-Down(y)-North(z)).
 *  
 *  This matrix change from EDN to ENU
 *        [1, 0, 0,        x        x          x'
 *         0, 0, 1,   *    y   =    z    =     y'
 *         0,-1, 0]        z       -y          z'
 * 
 *  This matrix change from EDN (ORB_SLAM2) to NEU (ROS: REP 103)
 *        [0, 0, 1,        x        z          x'
 *        -1, 0, 0,   *    y   =   -x    =     y'
 *         0,-1, 0]        z       -y          z'
 */

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>

#include "../../../include/System.h"

using namespace std;

double scale_factor;


// define SLAM and publisher for all the systems
class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM2::System *pSLAM, ros::NodeHandle *nh) : mpSLAM(pSLAM), pnh(nh)
  {
    mOdomPub = pnh->advertise<nav_msgs::Odometry>("/orb_slam/odom", 1);
    mPoseStampedPub = pnh->advertise<geometry_msgs::PoseStamped>("/orb_slam/pose", 1);
    mPoseWithCovStampedPub = pnh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam/pose_with_cov", 1);

  }

  void GrabImage(const sensor_msgs::ImageConstPtr &msg);
  void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
  void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

  ORB_SLAM2::System *mpSLAM;
  bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

  ros::NodeHandle *pnh;
  ros::Publisher mOdomPub;
  ros::Publisher mPoseStampedPub;
  ros::Publisher mPoseWithCovStampedPub;


  cv::Mat cvTcw;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::PoseStamped poseStamped_msg;
  geometry_msgs::PoseWithCovarianceStamped poseWithCovStamped_msg;

};

// static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
//     {{0.1, 0, 0, 0, 0, 0,
//       0, 0.1, 0, 0, 0, 0,
//       0, 0, 0.1, 0, 0, 0,
//       0, 0, 0, 0.1, 0, 0,
//       0, 0, 0, 0, 0.1, 0,
//       0, 0, 0, 0, 0, 0.1}};

// To initialize:
// ref: https://stackoverflow.com/questions/31549398/c-eigen-initialize-static-matrix
static const Eigen::Matrix3d cv_to_ros = [] {
  Eigen::Matrix3d tmp;
  tmp << 0, 0, 1,
        -1, 0, 0,
         0,-1, 0;
  return tmp;
}();

namespace common
{
  inline void CreateMsg(nav_msgs::Odometry &odom, geometry_msgs::PoseStamped &poseStamped,
                        geometry_msgs::PoseWithCovarianceStamped &poseWithCovStamped,
                        const sensor_msgs::ImageConstPtr &msgRGB,
                        cv::Mat cvTcw, double scale)
  {
    // Invert to get the cvTwc
    cv::Mat cvTwc = cvTcw.inv();
    // Open CV mat to Eigen matrix (float)
    Eigen::Matrix4d Twc;
    cv::cv2eigen(cvTwc, Twc);

    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot = Twc.block<3, 3>(0, 0);
    Eigen::Vector3d trans = Twc.block<3, 1>(0, 3);

    // Transform from CV coordinate system to ROS coordinate system on camera coordinates
    Eigen::Quaterniond quat(cv_to_ros * rot * cv_to_ros.transpose());
    trans = cv_to_ros * trans;

    // Format the Odom msg
    odom.header.stamp = msgRGB->header.stamp;
    // odom.header.frame_id = msgRGB->header.frame_id;
    odom.header.frame_id = "world";

    odom.pose.pose.position.x = trans(0) * scale;
    odom.pose.pose.position.y = trans(1) * scale;
    odom.pose.pose.position.z = trans(2) * scale;

    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    // for( int i = 0; i < 6; i++ )
    // {
    //   for( int j = 0; j < 6; j++ ) {
    //     odom.pose.covariance[ i*6 + j ] = poseCovariance(i,j);
    //   }
    // }

    // Format the poseStamped msg
    poseStamped.header.stamp = msgRGB->header.stamp;
    poseStamped.header.frame_id = "world";

    poseStamped.pose.position.x = trans(0) * scale;
    poseStamped.pose.position.y = trans(1) * scale;
    poseStamped.pose.position.z = trans(2) * scale;

    poseStamped.pose.orientation.x = quat.x();
    poseStamped.pose.orientation.y = quat.y();
    poseStamped.pose.orientation.z = quat.z();
    poseStamped.pose.orientation.w = quat.w();

    // Create pose message and update it with current camera pose
    poseWithCovStamped.header.stamp = msgRGB->header.stamp;
    poseWithCovStamped.header.frame_id = "odom";
    poseWithCovStamped.pose.pose.position.x = trans(0) * scale;
    poseWithCovStamped.pose.pose.position.y = trans(1) * scale;
    poseWithCovStamped.pose.pose.position.z = trans(2) * scale;
    poseWithCovStamped.pose.pose.orientation.x = quat.x();
    poseWithCovStamped.pose.pose.orientation.y = quat.y();
    poseWithCovStamped.pose.pose.orientation.z = quat.z();
    poseWithCovStamped.pose.pose.orientation.w = quat.w();

    // for (auto & x : poseWithCovStamped.pose.covariance) {
    //     x = 0.0;
    // }

    poseWithCovStamped.pose.covariance[0] = 1.0;
    poseWithCovStamped.pose.covariance[7] = 1.0;
    poseWithCovStamped.pose.covariance[14] = 1.0;
    poseWithCovStamped.pose.covariance[21] = 1.0;
    poseWithCovStamped.pose.covariance[28] = 1.0;
    poseWithCovStamped.pose.covariance[35] = 1.0;

    // Format the transformStamped msg
    // static tf2_ros::TransformBroadcaster tf_br;
    // geometry_msgs::TransformStamped transformStamped_msg;

    // transformStamped_msg.header.stamp = msgRGB->header.stamp;
    // transformStamped_msg.header.frame_id = "world";
    // transformStamped_msg.child_frame_id = "camera";

    // transformStamped_msg.transform.translation.x = trans(0);
    // transformStamped_msg.transform.translation.y = trans(1);
    // transformStamped_msg.transform.translation.z = trans(2);

    // transformStamped_msg.transform.rotation.x = quat.x();
    // transformStamped_msg.transform.rotation.y = quat.y();
    // transformStamped_msg.transform.rotation.z = quat.z();
    // transformStamped_msg.transform.rotation.w = quat.w();

    // tf_br.sendTransform(transformStamped_msg);
  }

}
#endif // COMMON_H
