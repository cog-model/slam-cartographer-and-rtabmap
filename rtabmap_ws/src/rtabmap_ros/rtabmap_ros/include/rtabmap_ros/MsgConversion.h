/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MSGCONVERSION_H_
#define MSGCONVERSION_H_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/LaserScan.h>

#include <rtabmap_ros_msgs/KeyPoint.h>
#include <rtabmap_ros_msgs/Point2f.h>
#include <rtabmap_ros_msgs/Point3f.h>
#include <rtabmap_ros_msgs/MapData.h>
#include <rtabmap_ros_msgs/MapGraph.h>
#include <rtabmap_ros_msgs/NodeData.h>
#include <rtabmap_ros_msgs/OdomInfo.h>
#include <rtabmap_ros_msgs/Info.h>
#include <rtabmap_ros_msgs/UserData.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform);
rtabmap::Transform transformFromTF(const tf::Transform & transform);

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg);
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg);

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg, bool ignoreRotationIfNotSet = false);

cv::KeyPoint keypointFromROS(const rtabmap_ros_msgs::KeyPoint & msg);
void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros_msgs::KeyPoint & msg);

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros_msgs::KeyPoint> & msg);
void keypointsFromROS(const std::vector<rtabmap_ros_msgs::KeyPoint> & msg, std::vector<cv::KeyPoint> & kpts, int xShift=0);
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros_msgs::KeyPoint> & msg);

rtabmap::EnvSensor envSensorFromROS(const rtabmap_ros_msgs::EnvSensor & msg);
void envSensorToROS(const rtabmap::EnvSensor & sensor, rtabmap_ros_msgs::EnvSensor & msg);
rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_ros_msgs::EnvSensor> & msg);
void envSensorsToROS(const rtabmap::EnvSensors & sensors, std::vector<rtabmap_ros_msgs::EnvSensor> & msg);

cv::Point2f point2fFromROS(const rtabmap_ros_msgs::Point2f & msg);
void point2fToROS(const cv::Point2f & kpt, rtabmap_ros_msgs::Point2f & msg);

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros_msgs::Point2f> & msg);
void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros_msgs::Point2f> & msg);

cv::Point3f point3fFromROS(const rtabmap_ros_msgs::Point3f & msg);
void point3fToROS(const cv::Point3f & pt, rtabmap_ros_msgs::Point3f & msg);

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros_msgs::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());
void points3fFromROS(const std::vector<rtabmap_ros_msgs::Point3f> & msg, std::vector<cv::Point3f> & points3, const rtabmap::Transform & transform = rtabmap::Transform());
void points3fToROS(const std::vector<cv::Point3f> & pts, std::vector<rtabmap_ros_msgs::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());

rtabmap::CameraModel cameraModelFromROS(
        const sensor_msgs::CameraInfo & camInfo,
        const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());
void cameraModelToROS(
        const rtabmap::CameraModel & model,
        sensor_msgs::CameraInfo & camInfo);

rtabmap::StereoCameraModel stereoCameraModelFromROS(
        const sensor_msgs::CameraInfo & leftCamInfo,
        const sensor_msgs::CameraInfo & rightCamInfo,
        const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity(),
        const rtabmap::Transform & stereoTransform = rtabmap::Transform());
rtabmap::StereoCameraModel stereoCameraModelFromROS(
        const sensor_msgs::CameraInfo & leftCamInfo,
        const sensor_msgs::CameraInfo & rightCamInfo,
        const std::string & frameId,
        tf::TransformListener & listener,
        double waitForTransform);

rtabmap::Landmarks landmarksFromROS(
        const std::map<int, std::pair<geometry_msgs::PoseWithCovarianceStamped, float> > & tags,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        tf::TransformListener & listener,
        double waitForTransform,
        double defaultLinVariance,
        double defaultAngVariance);

inline double timestampFromROS(const ros::Time & stamp) {return double(stamp.sec) + double(stamp.nsec)/1000000000.0;}

// common stuff
rtabmap::Transform getTransform(
        const std::string & fromFrameId,
        const std::string & toFrameId,
        const ros::Time & stamp,
        tf::TransformListener & listener,
        double waitForTransform);


// get moving transform accordingly to a fixed frame. For example get
// transform of /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
        const std::string & sourceTargetFrame,
        const std::string & fixedFrame,
        const ros::Time & stampSource,
        const ros::Time & stampTarget,
        tf::TransformListener & listener,
        double waitForTransform);

bool convertRGBMsgs(
        const std::vector<sensor_msgs::ImageConstPtr> & imageMsgs,
        const std::vector<sensor_msgs::CameraInfoConstPtr> & cameraInfoMsgs,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        std::vector<cv::Mat> & rgbs,
        std::vector<rtabmap::CameraModel> & cameraModels,
        tf::TransformListener & listener,
        double waitForTransform);

bool convertRGBDMsgs(
        const std::vector<sensor_msgs::ImageConstPtr> & imageMsgs,
        const std::vector<sensor_msgs::ImageConstPtr> & depthMsgs,
        const std::vector<sensor_msgs::CameraInfoConstPtr> & cameraInfoMsgs,
        const std::vector<sensor_msgs::CameraInfoConstPtr> & depthCamInfoMsgs,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        std::vector<cv::Mat> & rgbs,
        std::vector<cv::Mat> & depths,
        std::vector<rtabmap::CameraModel> & cameraModels,
        std::vector<rtabmap::CameraModel> & depthCameraModels,
        tf::TransformListener & listener,
        double waitForTransform);

bool convertStereoMsg(
        const cv_bridge::CvImageConstPtr& leftImageMsg,
        const cv_bridge::CvImageConstPtr& rightImageMsg,
        const sensor_msgs::CameraInfo& leftCamInfoMsg,
        const sensor_msgs::CameraInfo& rightCamInfoMsg,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        cv::Mat & left,
        cv::Mat & right,
        rtabmap::StereoCameraModel & stereoModel,
        tf::TransformListener & listener,
        double waitForTransform,
        bool alreadyRectified);

bool convertScanMsg(
        const sensor_msgs::LaserScan & scan2dMsg,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        rtabmap::LaserScan & scan,
        tf::TransformListener & listener,
        double waitForTransform,
        bool outputInFrameId = false);

bool convertScan3dMsg(
        const sensor_msgs::PointCloud2 & scan3dMsg,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        rtabmap::LaserScan & scan,
        tf::TransformListener & listener,
        double waitForTransform,
        int maxPoints = 0,
        float maxRange = 0.0f);

}

#endif /* MSGCONVERSION_H_ */
