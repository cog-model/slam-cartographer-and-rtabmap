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

#include "rtabmap_ros/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include <ros/ros.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <laser_geometry/laser_geometry.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform)
{
    if(!transform.isNull())
    {
        tf::transformEigenToTF(transform.toEigen3d(), tfTransform);
    }
    else
    {
        tfTransform = tf::Transform();
    }
}

rtabmap::Transform transformFromTF(const tf::Transform & transform)
{
    Eigen::Affine3d eigenTf;
    tf::transformTFToEigen(transform, eigenTf);
    return rtabmap::Transform::fromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg)
{
    if(!transform.isNull())
    {
        tf::transformEigenToMsg(transform.toEigen3d(), msg);

        // make sure the quaternion is normalized
        long double recipNorm = 1.0 / sqrt(msg.rotation.x * msg.rotation.x + msg.rotation.y * msg.rotation.y + msg.rotation.z * msg.rotation.z + msg.rotation.w * msg.rotation.w);
        msg.rotation.x *= recipNorm;
        msg.rotation.y *= recipNorm;
        msg.rotation.z *= recipNorm;
        msg.rotation.w *= recipNorm;
    }
    else
    {
        msg = geometry_msgs::Transform();
    }
}


rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg)
{
    if(msg.rotation.w == 0 &&
        msg.rotation.x == 0 &&
        msg.rotation.y == 0 &&
        msg.rotation.z ==0)
    {
        return rtabmap::Transform();
    }

    Eigen::Affine3d tfTransform;
    tf::transformMsgToEigen(msg, tfTransform);
    return rtabmap::Transform::fromEigen3d(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg)
{
    if(!transform.isNull())
    {
        tf::poseEigenToMsg(transform.toEigen3d(), msg);
    }
    else
    {
        msg = geometry_msgs::Pose();
    }
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg, bool ignoreRotationIfNotSet)
{
    if(msg.orientation.w == 0 &&
        msg.orientation.x == 0 &&
        msg.orientation.y == 0 &&
        msg.orientation.z == 0)
    {
        if(ignoreRotationIfNotSet)
        {
            return rtabmap::Transform(msg.position.x, msg.position.y, msg.position.z, 0, 0, 0);
        }
        return rtabmap::Transform();
    }
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    return rtabmap::Transform::fromEigen3d(tfPose);
}

cv::KeyPoint keypointFromROS(const rtabmap_ros_msgs::KeyPoint & msg)
{
    return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros_msgs::KeyPoint & msg)
{
    msg.angle = kpt.angle;
    msg.class_id = kpt.class_id;
    msg.octave = kpt.octave;
    msg.pt.x = kpt.pt.x;
    msg.pt.y = kpt.pt.y;
    msg.response = kpt.response;
    msg.size = kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros_msgs::KeyPoint> & msg)
{
    std::vector<cv::KeyPoint> v(msg.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        v[i] = keypointFromROS(msg[i]);
    }
    return v;
}

void keypointsFromROS(const std::vector<rtabmap_ros_msgs::KeyPoint> & msg, std::vector<cv::KeyPoint> & kpts, int xShift)
{
    size_t outCurrentIndex = kpts.size();
    kpts.resize(kpts.size()+msg.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        kpts[outCurrentIndex+i] = keypointFromROS(msg[i]);
        kpts[outCurrentIndex+i].pt.x += xShift;
    }
}

void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros_msgs::KeyPoint> & msg)
{
    msg.resize(kpts.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        keypointToROS(kpts[i], msg[i]);
    }
}

rtabmap::EnvSensor envSensorFromROS(const rtabmap_ros_msgs::EnvSensor & msg)
{
    return rtabmap::EnvSensor((rtabmap::EnvSensor::Type)msg.type, msg.value, timestampFromROS(msg.header.stamp));
}

void envSensorToROS(const rtabmap::EnvSensor & sensor, rtabmap_ros_msgs::EnvSensor & msg)
{
    msg.type = sensor.type();
    msg.value = sensor.value();
    msg.header.stamp = ros::Time(sensor.stamp());
}

rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_ros_msgs::EnvSensor> & msg)
{
    rtabmap::EnvSensors v;
    if(!msg.empty())
    {
        for(unsigned int i=0; i<msg.size(); ++i)
        {
            rtabmap::EnvSensor s = envSensorFromROS(msg[i]);
            v.insert(std::make_pair(s.type(), envSensorFromROS(msg[i])));
        }
    }
    return v;
}

void envSensorsToROS(const rtabmap::EnvSensors & sensors, std::vector<rtabmap_ros_msgs::EnvSensor> & msg)
{
    msg.clear();
    if(!sensors.empty())
    {
        msg.resize(sensors.size());
        int i=0;
        for(rtabmap::EnvSensors::const_iterator iter=sensors.begin(); iter!=sensors.end(); ++iter)
        {
            envSensorToROS(iter->second, msg[i++]);
        }
    }
}

cv::Point2f point2fFromROS(const rtabmap_ros_msgs::Point2f & msg)
{
    return cv::Point2f(msg.x, msg.y);
}

void point2fToROS(const cv::Point2f & kpt, rtabmap_ros_msgs::Point2f & msg)
{
    msg.x = kpt.x;
    msg.y = kpt.y;
}

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros_msgs::Point2f> & msg)
{
    std::vector<cv::Point2f> v(msg.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        v[i] = point2fFromROS(msg[i]);
    }
    return v;
}

void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros_msgs::Point2f> & msg)
{
    msg.resize(kpts.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        point2fToROS(kpts[i], msg[i]);
    }
}

cv::Point3f point3fFromROS(const rtabmap_ros_msgs::Point3f & msg)
{
    return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f & pt, rtabmap_ros_msgs::Point3f & msg)
{
    msg.x = pt.x;
    msg.y = pt.y;
    msg.z = pt.z;
}

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros_msgs::Point3f> & msg, const rtabmap::Transform & transform)
{
    bool transformPoints = !transform.isNull() && !transform.isIdentity();
    std::vector<cv::Point3f> v(msg.size());
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        v[i] = point3fFromROS(msg[i]);
        if(transformPoints)
        {
            v[i] = rtabmap::util3d::transformPoint(v[i], transform);
        }
    }
    return v;
}

void points3fFromROS(const std::vector<rtabmap_ros_msgs::Point3f> & msg, std::vector<cv::Point3f> & points3, const rtabmap::Transform & transform)
{
    size_t currentIndex = points3.size();
    points3.resize(points3.size()+msg.size());
    bool transformPoint = !transform.isNull() && !transform.isIdentity();
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        points3[currentIndex+i] = point3fFromROS(msg[i]);
        if(transformPoint)
        {
            points3[currentIndex+i] = rtabmap::util3d::transformPoint(points3[currentIndex+i], transform);
        }
    }
}

void points3fToROS(const std::vector<cv::Point3f> & pts, std::vector<rtabmap_ros_msgs::Point3f> & msg, const rtabmap::Transform & transform)
{
    msg.resize(pts.size());
    bool transformPoints = !transform.isNull() && !transform.isIdentity();
    for(unsigned int i=0; i<msg.size(); ++i)
    {
        if(transformPoints)
        {
            cv::Point3f pt = rtabmap::util3d::transformPoint(pts[i], transform);
            point3fToROS(pt, msg[i]);
        }
        else
        {
            point3fToROS(pts[i], msg[i]);
        }
    }
}

rtabmap::CameraModel cameraModelFromROS(
        const sensor_msgs::CameraInfo & camInfo,
        const rtabmap::Transform & localTransform)
{
    cv:: Mat K;
    UASSERT(camInfo.K.empty() || camInfo.K.size() == 9);
    if(!camInfo.K.empty())
    {
        K = cv::Mat(3, 3, CV_64FC1);
        memcpy(K.data, camInfo.K.elems, 9*sizeof(double));
    }

    cv::Mat D;
    if(camInfo.D.size())
    {
        if(camInfo.D.size()>=4 &&
           (uStrContains(camInfo.distortion_model, "fisheye") ||
            uStrContains(camInfo.distortion_model, "equidistant") ||
            uStrContains(camInfo.distortion_model, "Kannala Brandt4")))
        {
            D = cv::Mat::zeros(1, 6, CV_64FC1);
            D.at<double>(0,0) = camInfo.D[0];
            D.at<double>(0,1) = camInfo.D[1];
            D.at<double>(0,4) = camInfo.D[2];
            D.at<double>(0,5) = camInfo.D[3];
        }
        else if(camInfo.D.size()>8)
        {
            bool zerosAfter8 = true;
            for(size_t i=8; i<camInfo.D.size() && zerosAfter8; ++i)
            {
                if(camInfo.D[i] != 0.0)
                {
                    zerosAfter8 = false;
                }
            }
            static bool warned = false;
            if(!zerosAfter8 && !warned)
            {
                ROS_WARN("Camera info conversion: Distortion model is larger than 8, coefficients after 8 are ignored. This message is only shown once.");
                warned = true;
            }
            D = cv::Mat(1, 8, CV_64FC1);
            memcpy(D.data, camInfo.D.data(), D.cols*sizeof(double));
        }
        else
        {
            D = cv::Mat(1, camInfo.D.size(), CV_64FC1);
            memcpy(D.data, camInfo.D.data(), D.cols*sizeof(double));
        }
    }

    cv:: Mat R;
    UASSERT(camInfo.R.empty() || camInfo.R.size() == 9);
    if(!camInfo.R.empty())
    {
        R = cv::Mat(3, 3, CV_64FC1);
        memcpy(R.data, camInfo.R.elems, 9*sizeof(double));
    }

    cv:: Mat P;
    UASSERT(camInfo.P.empty() || camInfo.P.size() == 12);
    if(!camInfo.P.empty())
    {
        P = cv::Mat(3, 4, CV_64FC1);
        memcpy(P.data, camInfo.P.elems, 12*sizeof(double));
    }

    return rtabmap::CameraModel(
            "ros",
            cv::Size(camInfo.width, camInfo.height),
            K, D, R, P,
            localTransform);
}
void cameraModelToROS(
        const rtabmap::CameraModel & model,
        sensor_msgs::CameraInfo & camInfo)
{
    UASSERT(model.K_raw().empty() || model.K_raw().total() == 9);
    if(model.K_raw().empty())
    {
        memset(camInfo.K.elems, 0.0, 9*sizeof(double));
    }
    else
    {
        memcpy(camInfo.K.elems, model.K_raw().data, 9*sizeof(double));
    }

    if(camInfo.D.size() == 6)
    {
        camInfo.D = std::vector<double>(4);
        camInfo.D[0] = model.D_raw().at<double>(0,0);
        camInfo.D[1] = model.D_raw().at<double>(0,1);
        camInfo.D[2] = model.D_raw().at<double>(0,4);
        camInfo.D[3] = model.D_raw().at<double>(0,5);
        camInfo.distortion_model = "equidistant"; // fisheye
    }
    else
    {
        camInfo.D = std::vector<double>(model.D_raw().cols);
        memcpy(camInfo.D.data(), model.D_raw().data, model.D_raw().cols*sizeof(double));
        if(camInfo.D.size() > 5)
        {
            camInfo.distortion_model = "rational_polynomial";
        }
        else
        {
            camInfo.distortion_model = "plumb_bob";
        }
    }

    UASSERT(model.R().empty() || model.R().total() == 9);
    if(model.R().empty())
    {
        memset(camInfo.R.elems, 0.0, 9*sizeof(double));
    }
    else
    {
        memcpy(camInfo.R.elems, model.R().data, 9*sizeof(double));
    }

    UASSERT(model.P().empty() || model.P().total() == 12);
    if(model.P().empty())
    {
        memset(camInfo.P.elems, 0.0, 12*sizeof(double));
    }
    else
    {
        memcpy(camInfo.P.elems, model.P().data, 12*sizeof(double));
    }

    camInfo.binning_x = 1;
    camInfo.binning_y = 1;
    camInfo.roi.width = model.imageWidth();
    camInfo.roi.height = model.imageHeight();

    camInfo.width = model.imageWidth();
    camInfo.height = model.imageHeight();
}
rtabmap::StereoCameraModel stereoCameraModelFromROS(
        const sensor_msgs::CameraInfo & leftCamInfo,
        const sensor_msgs::CameraInfo & rightCamInfo,
        const rtabmap::Transform & localTransform,
        const rtabmap::Transform & stereoTransform)
{
    return rtabmap::StereoCameraModel(
            "ros",
            cameraModelFromROS(leftCamInfo, localTransform),
            cameraModelFromROS(rightCamInfo, localTransform),
            stereoTransform);
}
rtabmap::StereoCameraModel stereoCameraModelFromROS(
        const sensor_msgs::CameraInfo & leftCamInfo,
        const sensor_msgs::CameraInfo & rightCamInfo,
        const std::string & frameId,
        tf::TransformListener & listener,
        double waitForTransform)
{
    rtabmap::Transform localTransform = getTransform(
            frameId,
            leftCamInfo.header.frame_id,
            leftCamInfo.header.stamp,
            listener,
            waitForTransform);
    if(localTransform.isNull())
    {
        return rtabmap::StereoCameraModel();
    }

    rtabmap::Transform stereoTransform = getTransform(
            leftCamInfo.header.frame_id,
            rightCamInfo.header.frame_id,
            leftCamInfo.header.stamp,
            listener,
            waitForTransform);
    if(stereoTransform.isNull())
    {
        return rtabmap::StereoCameraModel();
    }
    return stereoCameraModelFromROS(leftCamInfo, rightCamInfo, localTransform, stereoTransform);
}

rtabmap::Landmarks landmarksFromROS(
        const std::map<int, std::pair<geometry_msgs::PoseWithCovarianceStamped, float> > & tags,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        tf::TransformListener & listener,
        double waitForTransform,
        double defaultLinVariance,
        double defaultAngVariance)
{
    //tag detections
    rtabmap::Landmarks landmarks;
    for(std::map<int, std::pair<geometry_msgs::PoseWithCovarianceStamped, float> >::const_iterator iter=tags.begin(); iter!=tags.end(); ++iter)
    {
        if(iter->first <=0)
        {
            ROS_ERROR("Invalid landmark received! IDs should be > 0 (it is %d). Ignoring this landmark.", iter->first);
            continue;
        }
        rtabmap::Transform baseToCamera = rtabmap_ros::getTransform(
                frameId,
                iter->second.first.header.frame_id,
                iter->second.first.header.stamp,
                listener,
                waitForTransform);

        if(baseToCamera.isNull())
        {
            ROS_ERROR("Cannot transform tag pose from \"%s\" frame to \"%s\" frame!",
                    iter->second.first.header.frame_id.c_str(), frameId.c_str());
            continue;
        }

        rtabmap::Transform baseToTag = baseToCamera * transformFromPoseMsg(iter->second.first.pose.pose);

        if(!baseToTag.isNull())
        {
            // Correction of the global pose accounting the odometry movement since we received it
            rtabmap::Transform correction = rtabmap_ros::getTransform(
                    frameId,
                    odomFrameId,
                    iter->second.first.header.stamp,
                    odomStamp,
                    listener,
                    waitForTransform);
            if(!correction.isNull())
            {
                baseToTag = correction * baseToTag;
            }
            else
            {
                ROS_WARN("Could not adjust tag pose accordingly to latest odometry pose. "
                        "If odometry is small since it received the tag pose and "
                        "covariance is large, this should not be a problem.");
            }
            cv::Mat covariance = cv::Mat(6,6, CV_64FC1, (void*)iter->second.first.pose.covariance.data()).clone();
            if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
            {
                covariance = cv::Mat::eye(6,6,CV_64FC1);
                covariance(cv::Range(0,3), cv::Range(0,3)) *= defaultLinVariance;
                covariance(cv::Range(3,6), cv::Range(3,6)) *= defaultAngVariance;
            }
            landmarks.insert(std::make_pair(iter->first, rtabmap::Landmark(iter->first, iter->second.second, baseToTag, covariance)));
        }
    }
    return landmarks;
}

rtabmap::Transform getTransform(
        const std::string & fromFrameId,
        const std::string & toFrameId,
        const ros::Time & stamp,
        tf::TransformListener & listener,
        double waitForTransform)
{
    // TF ready?
    rtabmap::Transform transform;
    try
    {
        if(waitForTransform > 0.0 && !stamp.isZero())
        {
            //if(!tfBuffer_.canTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
            std::string errorMsg;
            if(!listener.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(waitForTransform), ros::Duration(0.01), &errorMsg))
            {
                ROS_WARN("Could not get transform from %s to %s after %f seconds (for stamp=%f)! Error=\"%s\".",
                        fromFrameId.c_str(), toFrameId.c_str(), waitForTransform, stamp.toSec(), errorMsg.c_str());
                return transform;
            }
        }

        tf::StampedTransform tmp;
        listener.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
        transform = rtabmap_ros::transformFromTF(tmp);
    }
    catch(tf::TransformException & ex)
    {
        ROS_WARN("(getting transform %s -> %s) %s", fromFrameId.c_str(), toFrameId.c_str(), ex.what());
    }
    return transform;
}

// get moving transform accordingly to a fixed frame. For example get
// transform between moving /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
        const std::string & sourceTargetFrame,
        const std::string & fixedFrame,
        const ros::Time & stampSource,
        const ros::Time & stampTarget,
        tf::TransformListener & listener,
        double waitForTransform)
{
    // TF ready?
    rtabmap::Transform transform;
    try
    {
        ros::Time stamp = stampSource>stampTarget?stampSource:stampTarget;
        if(waitForTransform > 0.0 && !stamp.isZero())
        {
            std::string errorMsg;
            if(!listener.waitForTransform(sourceTargetFrame, fixedFrame, stamp, ros::Duration(waitForTransform), ros::Duration(0.01), &errorMsg))
            {
                ROS_WARN("Could not get transform from %s to %s accordingly to %s after %f seconds (for stamps=%f -> %f)! Error=\"%s\".",
                        sourceTargetFrame.c_str(), sourceTargetFrame.c_str(), fixedFrame.c_str(), waitForTransform, stampSource.toSec(), stampTarget.toSec(), errorMsg.c_str());
                return transform;
            }
        }

        tf::StampedTransform tmp;
        listener.lookupTransform(sourceTargetFrame, stampTarget, sourceTargetFrame, stampSource, fixedFrame, tmp);
        transform = rtabmap_ros::transformFromTF(tmp);
    }
    catch(tf::TransformException & ex)
    {
        ROS_WARN("(getting transform movement of %s according to fixed %s) %s", sourceTargetFrame.c_str(), fixedFrame.c_str(), ex.what());
    }
    return transform;
}

bool convertRGBMsgs(
        const std::vector<sensor_msgs::ImageConstPtr> & imageMsgs,
        const std::vector<sensor_msgs::CameraInfoConstPtr> & cameraInfoMsgs,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        std::vector<cv::Mat> & rgbs,
        std::vector<rtabmap::CameraModel> & cameraModels,
        tf::TransformListener & listener,
        double waitForTransform)
{
    std::vector<cv::Mat> depths;
    std::vector<rtabmap::CameraModel> depthCameraModels;
    bool convertionOk = convertRGBDMsgs(imageMsgs, {}, cameraInfoMsgs, {}, frameId, odomFrameId, odomStamp,
        rgbs, depths, cameraModels, depthCameraModels, listener, waitForTransform);
    return convertionOk;
}

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
        double waitForTransform)
{
    UASSERT(imageMsgs.size() == cameraInfoMsgs.size());
    UASSERT(depthMsgs.size() == depthCamInfoMsgs.size());
    UASSERT(imageMsgs.empty() || depthMsgs.empty() || imageMsgs.size() == depthMsgs.size());

    for(unsigned int i=0; i<imageMsgs.size(); ++i)
    {
        if(!(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0 ||
                imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_RGGB8) == 0))
        {
            ROS_ERROR("Input rgb type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8. Current rgb=%s",
                    imageMsgs[i]->encoding.c_str());
            return false;
        }
        ros::Time stamp;
        stamp = imageMsgs[i]->header.stamp;
        rtabmap::Transform localTransform = rtabmap_ros::getTransform(frameId, imageMsgs[i]->header.frame_id, stamp, listener, waitForTransform);
        if(localTransform.isNull())
        {
            ROS_ERROR("TF of received image %d at time %fs is not set!", i, stamp.toSec());
            return false;
        }
        // sync with odometry stamp
        if(!odomFrameId.empty() && odomStamp != stamp)
        {
            rtabmap::Transform sensorT = getTransform(
                    frameId,
                    odomFrameId,
                    odomStamp,
                    stamp,
                    listener,
                    waitForTransform);
            if(sensorT.isNull())
            {
                ROS_WARN("Could not get odometry value for depth image stamp (%fs). Latest odometry "
                        "stamp is %fs. The depth image pose will not be synchronized with odometry.", stamp.toSec(), odomStamp.toSec());
            }
            else
            {
                //ROS_WARN("RGBD correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(stamp.toSec()-odomStamp.toSec()));
                localTransform = sensorT * localTransform;
            }
        }
        sensor_msgs::ImageConstPtr ptrImage = imageMsgs[i];
        rgbs.push_back(cv_bridge::toCvCopy(ptrImage, "bgr8")->image);
        cameraModels.push_back(rtabmap_ros::cameraModelFromROS(*cameraInfoMsgs[i], localTransform));
    }

    // for(unsigned int i=0; i<depthMsgs.size(); ++i)
    // {
    //     if(!(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
    //            depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
    //            depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
    //     {
    //         ROS_ERROR("Input depth type must be image_depth=32FC1,16UC1,mono16. Current depth=%s",
    //                 depthMsgs[i]->encoding.c_str());
    //         return false;
    //     }
    //     ros::Time stamp;
    //     stamp = depthMsgs[i]->header.stamp;
    //     rtabmap::Transform localTransform = rtabmap_ros::getTransform(frameId, depthMsgs[i]->header.frame_id, stamp, listener, waitForTransform);
    //     if(localTransform.isNull())
    //     {
    //         ROS_ERROR("TF of received image %d at time %fs is not set!", i, stamp.toSec());
    //         return false;
    //     }
    //     // sync with odometry stamp
    //     if(!odomFrameId.empty() && odomStamp != stamp)
    //     {
    //         rtabmap::Transform sensorT = getTransform(
    //                 frameId,
    //                 odomFrameId,
    //                 odomStamp,
    //                 stamp,
    //                 listener,
    //                 waitForTransform);
    //         if(sensorT.isNull())
    //         {
    //             ROS_WARN("Could not get odometry value for depth image stamp (%fs). Latest odometry "
    //                     "stamp is %fs. The depth image pose will not be synchronized with odometry.", stamp.toSec(), odomStamp.toSec());
    //         }
    //         else
    //         {
    //             //ROS_WARN("RGBD correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(stamp.toSec()-odomStamp.toSec()));
    //             localTransform = sensorT * localTransform;
    //         }
    //     }
    //     cv_bridge::CvImageConstPtr ptrDepth = depthMsgs[i];
    //     depths.push_back(ptrDepth->image.clone());
    //     depthCameraModels.push_back(rtabmap_ros::cameraModelFromROS(depthCamInfoMsgs[i], localTransform));
    // }
    return true;
}

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
        bool alreadyRectified)
{
    UASSERT(leftImageMsg.get() && rightImageMsg.get());

    if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 || 
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
        !(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 || 
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
        rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
    {
        ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8");
        ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 Current left=%s and right=%s",
                leftImageMsg->encoding.c_str(),
                rightImageMsg->encoding.c_str());
        return false;
    }

    if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
       leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
    {
        left = leftImageMsg->image;
    }
    else if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
    {
        left = cv_bridge::cvtColor(leftImageMsg, "mono8")->image;
    }
    else
    {
        left = cv_bridge::cvtColor(leftImageMsg, "bgr8")->image;
    }
    if(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
       rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
    {
        right = rightImageMsg->image;
    }
    else
    {
        right = cv_bridge::cvtColor(rightImageMsg, "mono8")->image;
    }

    rtabmap::Transform localTransform = getTransform(frameId, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, listener, waitForTransform);
    if(localTransform.isNull())
    {
        return false;
    }
    // sync with odometry stamp
    if(!odomFrameId.empty() && odomStamp != leftImageMsg->header.stamp)
    {
        rtabmap::Transform sensorT = getTransform(
                frameId,
                odomFrameId,
                odomStamp,
                leftImageMsg->header.stamp,
                listener,
                waitForTransform);
        if(sensorT.isNull())
        {
            ROS_WARN("Could not get odometry value for stereo msg stamp (%fs). Latest odometry "
                    "stamp is %fs. The stereo image pose will not be synchronized with odometry.", leftImageMsg->header.stamp.toSec(), odomStamp.toSec());
        }
        else
        {
            localTransform = sensorT * localTransform;
        }
    }

    rtabmap::Transform stereoTransform;
    if(!alreadyRectified)
    {
        stereoTransform = getTransform(
                rightCamInfoMsg.header.frame_id,
                leftCamInfoMsg.header.frame_id,
                leftCamInfoMsg.header.stamp,
                listener,
                waitForTransform);
        if(stereoTransform.isNull())
        {
            ROS_ERROR("Parameter ??? is false but we cannot get TF between the two cameras!");
            return false;
        }
    }

    stereoModel = rtabmap_ros::stereoCameraModelFromROS(leftCamInfoMsg, rightCamInfoMsg, localTransform, stereoTransform);

    if(stereoModel.baseline() > 10.0)
    {
        static bool shown = false;
        if(!shown)
        {
            ROS_WARN("Detected baseline (%f m) is quite large! Is your "
                     "right camera_info P(0,3) correctly set? Note that "
                     "baseline=-P(0,3)/P(0,0). You may need to calibrate your camera. "
                     "This warning is printed only once.",
                     stereoModel.baseline());
            shown = true;
        }
    }
    else if(stereoModel.baseline() == 0 && alreadyRectified)
    {
        rtabmap::Transform stereoTransform = getTransform(
                leftCamInfoMsg.header.frame_id,
                rightCamInfoMsg.header.frame_id,
                leftCamInfoMsg.header.stamp,
                listener,
                waitForTransform);
        if(stereoTransform.isNull() || stereoTransform.x()<=0)
        {
            ROS_WARN("We cannot estimated the baseline of the rectified images with tf! (%s->%s = %s)",
                    rightCamInfoMsg.header.frame_id.c_str(), leftCamInfoMsg.header.frame_id.c_str(), stereoTransform.prettyPrint().c_str());
        }
        else
        {
            static bool warned = false;
            if(!warned)
            {
                ROS_WARN("Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified. While not "
                        "recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
                        "a valid right camera info if stereo images are already rectified. This message is only printed once...",
                        rightCamInfoMsg.header.frame_id.c_str(), leftCamInfoMsg.header.frame_id.c_str(), stereoTransform.x());
                warned = true;
            }
            stereoModel = rtabmap::StereoCameraModel(
                    stereoModel.left().fx(),
                    stereoModel.left().fy(),
                    stereoModel.left().cx(),
                    stereoModel.left().cy(),
                    stereoTransform.x(),
                    stereoModel.localTransform(),
                    stereoModel.left().imageSize());
        }
    }
    return true;
}

bool convertScanMsg(
        const sensor_msgs::LaserScan & scan2dMsg,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        rtabmap::LaserScan & scan,
        tf::TransformListener & listener,
        double waitForTransform,
        bool outputInFrameId)
{
    // make sure the frame of the laser is updated too
    rtabmap::Transform tmpT = getTransform(
            odomFrameId.empty()?frameId:odomFrameId,
            scan2dMsg.header.frame_id,
            scan2dMsg.header.stamp + ros::Duration().fromSec(scan2dMsg.ranges.size()*scan2dMsg.time_increment),
            listener,
            waitForTransform);
    if(tmpT.isNull())
    {
        return false;
    }

    rtabmap::Transform scanLocalTransform = getTransform(
            frameId,
            scan2dMsg.header.frame_id,
            scan2dMsg.header.stamp,
            listener,
            waitForTransform);
    if(scanLocalTransform.isNull())
    {
        return false;
    }

    //transform in frameId_ frame
    sensor_msgs::PointCloud2 scanOut;
    laser_geometry::LaserProjection projection;
    projection.transformLaserScanToPointCloud(odomFrameId.empty()?frameId:odomFrameId, scan2dMsg, scanOut, listener);

    //transform back in laser frame
    rtabmap::Transform laserToOdom = getTransform(
            scan2dMsg.header.frame_id,
            odomFrameId.empty()?frameId:odomFrameId,
            scan2dMsg.header.stamp,
            listener,
            waitForTransform);
    if(laserToOdom.isNull())
    {
        return false;
    }

    // sync with odometry stamp
    if(!odomFrameId.empty() && odomStamp != scan2dMsg.header.stamp)
    {
        rtabmap::Transform sensorT = getTransform(
                frameId,
                odomFrameId,
                odomStamp,
                scan2dMsg.header.stamp,
                listener,
                waitForTransform);
        if(sensorT.isNull())
        {
            ROS_WARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
                    "stamp is %fs. The laser scan pose will not be synchronized with odometry.", scan2dMsg.header.stamp.toSec(), odomStamp.toSec());
        }
        else
        {
            //ROS_WARN("scan correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(scan2dMsg->header.stamp.toSec()-odomStamp.toSec()));
            scanLocalTransform = sensorT * scanLocalTransform;
        }
    }

    if(outputInFrameId)
    {
        laserToOdom *= scanLocalTransform;
    }

    bool hasIntensity = false;
    for(unsigned int i=0; i<scanOut.fields.size(); ++i)
    {
        if(scanOut.fields[i].name.compare("intensity") == 0)
        {
            if(scanOut.fields[i].datatype == sensor_msgs::PointField::FLOAT32)
            {
                hasIntensity = true;
            }
            else
            {
                static bool warningShown = false;
                if(!warningShown)
                {
                    ROS_WARN("The input scan cloud has an \"intensity\" field "
                            "but the datatype (%d) is not supported. Intensity will be ignored. "
                            "This message is only shown once.", scanOut.fields[i].datatype);
                    warningShown = true;
                }
            }
        }
    }

    rtabmap::LaserScan::Format format;
    cv::Mat data;
    if(hasIntensity)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(scanOut, *pclScan);
        pclScan->is_dense = true;
        data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom).data(); // put back in laser frame
        format = rtabmap::LaserScan::kXYI;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(scanOut, *pclScan);
        pclScan->is_dense = true;
        data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom).data(); // put back in laser frame
        format = rtabmap::LaserScan::kXY;
    }

    rtabmap::Transform zAxis(0,0,1,0,0,0);
    if((scanLocalTransform.rotation()*zAxis).z() < 0)
    {
        cv::Mat flipScan;
        cv::flip(data, flipScan, 1);
        data = flipScan;
    }

    scan = rtabmap::LaserScan(
            data,
            format,
            scan2dMsg.range_min,
            scan2dMsg.range_max,
            scan2dMsg.angle_min,
            scan2dMsg.angle_max,
            scan2dMsg.angle_increment,
            outputInFrameId?rtabmap::Transform::getIdentity():scanLocalTransform);

    return true;
}

bool convertScan3dMsg(
        const sensor_msgs::PointCloud2 & scan3dMsg,
        const std::string & frameId,
        const std::string & odomFrameId,
        const ros::Time & odomStamp,
        rtabmap::LaserScan & scan,
        tf::TransformListener & listener,
        double waitForTransform,
        int maxPoints,
        float maxRange)
{
    if (scan3dMsg.row_step != 0 || scan3dMsg.height != 1) {
        UASSERT_MSG(scan3dMsg.data.size() == scan3dMsg.row_step*scan3dMsg.height,
                uFormat("data=%d row_step=%d height=%d", scan3dMsg.data.size(), scan3dMsg.row_step, scan3dMsg.height).c_str());
    }

    rtabmap::Transform scanLocalTransform = getTransform(frameId, scan3dMsg.header.frame_id, scan3dMsg.header.stamp, listener, waitForTransform);
    if(scanLocalTransform.isNull())
    {
        ROS_ERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", scan3dMsg.header.stamp.toSec());
        return false;
    }

    // sync with odometry stamp
    if(!odomFrameId.empty() && odomStamp != scan3dMsg.header.stamp)
    {
        rtabmap::Transform sensorT = getTransform(
                frameId,
                odomFrameId,
                odomStamp,
                scan3dMsg.header.stamp,
                listener,
                waitForTransform);
        if(sensorT.isNull())
        {
            ROS_WARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
                    "stamp is %fs. The 3d laser scan pose will not be synchronized with odometry.", scan3dMsg.header.stamp.toSec(), odomStamp.toSec());
        }
        else
        {
            scanLocalTransform = sensorT * scanLocalTransform;
        }
    }
    scan = rtabmap::util3d::laserScanFromPointCloud(scan3dMsg);
    scan = rtabmap::LaserScan(scan, maxPoints, maxRange, scanLocalTransform);
    return true;
}

}
