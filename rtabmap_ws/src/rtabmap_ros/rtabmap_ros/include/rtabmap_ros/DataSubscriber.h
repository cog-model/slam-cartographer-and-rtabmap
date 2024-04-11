#pragma once

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <vector>
#include <memory>
#include <functional>

#define DEF_SYNC_POLICIES(POLICY_PREFIX, ...) \
typedef message_filters::sync_policies::ApproximateTime<__VA_ARGS__> POLICY_PREFIX##ApproxPolicy; \
typedef message_filters::sync_policies::ExactTime<__VA_ARGS__> POLICY_PREFIX##ExactPolicy

#define DECL_SYNCHRONIZERS(PREFIX, POLICY_PREFIX) \
typedef message_filters::Synchronizer<POLICY_PREFIX##ApproxPolicy> POLICY_PREFIX##ApproxSync; \
typedef message_filters::Synchronizer<POLICY_PREFIX##ExactPolicy> POLICY_PREFIX##ExactSync; \
std::unique_ptr<POLICY_PREFIX##ApproxSync> PREFIX##ApproxSync_; \
std::unique_ptr<POLICY_PREFIX##ExactSync> PREFIX##ExactSync_

namespace rtabmap_ros {

class DataSubscriber
{
private:
    using DataCallback = std::function<void(
        const nav_msgs::Odometry&,
        const nav_msgs::Odometry&,
        const sensor_msgs::PointCloud2&,
        const std::vector<sensor_msgs::CameraInfoConstPtr>&,
        const std::vector<sensor_msgs::ImageConstPtr>&)>;

    DEF_SYNC_POLICIES(Default,
        nav_msgs::Odometry,
        nav_msgs::Odometry,
        sensor_msgs::PointCloud2);
    DEF_SYNC_POLICIES(WithImage,
        nav_msgs::Odometry,
        nav_msgs::Odometry,
        sensor_msgs::PointCloud2,
        sensor_msgs::CameraInfo,
        sensor_msgs::Image);
    DEF_SYNC_POLICIES(With2Images,
        nav_msgs::Odometry,
        nav_msgs::Odometry,
        sensor_msgs::PointCloud2,
        sensor_msgs::CameraInfo,
        sensor_msgs::Image,
        sensor_msgs::CameraInfo,
        sensor_msgs::Image);

    DECL_SYNCHRONIZERS(default, Default);
    DECL_SYNCHRONIZERS(withImage, WithImage);
    DECL_SYNCHRONIZERS(with2Images, With2Images);

public:
    DataSubscriber() = default;
    ~DataSubscriber();

    void setDataCallback(DataCallback callback);
    void setupCallback(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name);

private:
    void setupDefaultCallback(
        ros::NodeHandle& nh, ros::NodeHandle& pnh,
        int queueSize, bool approxSync);
    void setupWithImageCallback(
        ros::NodeHandle& nh, ros::NodeHandle& pnh,
        int queueSize, bool approxSync);
    void setupWith2ImagesCallback(
        ros::NodeHandle& nh, ros::NodeHandle& pnh,
        int queueSize, bool approxSync);

    void defaultCallback(
        const nav_msgs::OdometryConstPtr& globalOdomMsg,
        const nav_msgs::OdometryConstPtr& localOdomMsg,
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg);
    void withImageCallback(
        const nav_msgs::OdometryConstPtr& globalOdomMsg,
        const nav_msgs::OdometryConstPtr& localOdomMsg,
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg,
        const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
        const sensor_msgs::ImageConstPtr& imageMsg);
    void with2ImagesCallback(
        const nav_msgs::OdometryConstPtr& globalOdomMsg,
        const nav_msgs::OdometryConstPtr& localOdomMsg,
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg,
        const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
        const sensor_msgs::ImageConstPtr& imageMsg1,
        const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
        const sensor_msgs::ImageConstPtr& imageMsg2);

private:
    message_filters::Subscriber<nav_msgs::Odometry> globalOdomSub_;
    message_filters::Subscriber<nav_msgs::Odometry> localOdomSub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub_;
    std::vector<std::unique_ptr<
        message_filters::Subscriber<sensor_msgs::CameraInfo>>> cameraInfoSubs_;
    std::vector<std::unique_ptr<
        image_transport::SubscriberFilter>> imageSubs_;

private:
    bool subscribed_ = false;
    std::string name_;
    DataCallback callback_;
};

}
