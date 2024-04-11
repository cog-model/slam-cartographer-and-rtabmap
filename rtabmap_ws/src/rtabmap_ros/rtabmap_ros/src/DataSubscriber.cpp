#include <rtabmap_ros/DataSubscriber.h>

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap_ros {

DataSubscriber::~DataSubscriber()
{
    defaultApproxSync_.reset();
    defaultExactSync_.reset();
    withImageApproxSync_.reset();
    withImageExactSync_.reset();
    with2ImagesApproxSync_.reset();
    with2ImagesExactSync_.reset();
}

void DataSubscriber::setDataCallback(DataCallback callback)
{
    if (subscribed_)
    {
        ROS_FATAL("Cannot set callback after subscribing.");
    }
    callback_ = std::move(callback);
}

void DataSubscriber::setupCallback(ros::NodeHandle& nh, ros::NodeHandle& pnh,
    const std::string& name)
{
    if (!callback_)
    {
        ROS_FATAL("Cannot subscribe without callback.");
    }

    name_ = name;
    ros::NodeHandle namedNh(nh, name_);
    ros::NodeHandle namedPnh(pnh, name_);

    int queueSize;
    bool approxSync;
    namedPnh.param("queue_size", queueSize, 10);
    namedPnh.param("approx_sync", approxSync, true);

    bool subscribeRgb;
    int numRgb;
    namedPnh.param("subscribe_rgb", subscribeRgb, false);
    namedPnh.param("num_rgb", numRgb, 1);
    if (numRgb < 1 || numRgb > 2)
    {
        ROS_FATAL("Unsupported number of cameras.");
    }

    if (!subscribeRgb)
    {
        setupDefaultCallback(namedNh, namedPnh, queueSize, approxSync);
    }
    else
    {
        if (numRgb == 1)
        {
            setupWithImageCallback(namedNh, namedPnh, queueSize, approxSync);
        }
        if (numRgb == 2)
        {
            setupWith2ImagesCallback(namedNh, namedPnh, queueSize, approxSync);
        }
    }

    subscribed_ = true;
}

void DataSubscriber::setupDefaultCallback(
    ros::NodeHandle& nh, ros::NodeHandle& pnh,
    int queueSize, bool approxSync)
{
    globalOdomSub_.subscribe(nh, "global_odom", queueSize);
    localOdomSub_.subscribe(nh, "local_odom", queueSize);
    pointCloudSub_.subscribe(nh, "point_cloud", queueSize);
    if (approxSync)
    {
        defaultApproxSync_ = std::make_unique<DefaultApproxSync>(
            DefaultApproxPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_);
        defaultApproxSync_->registerCallback(&DataSubscriber::defaultCallback, this);
    }
    else
    {
        defaultExactSync_ = std::make_unique<DefaultExactSync>(
            DefaultExactPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_);
        defaultExactSync_->registerCallback(&DataSubscriber::defaultCallback, this);
    }

    std::string syncStr;
    if (approxSync)
    {
        syncStr = "approx";
    }
    else
    {
        syncStr = "exact";
    }
    ROS_INFO("\nDataSubscriber named '%s' subscribed to (%s sync, queue size %d):\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s",
        name_.c_str(), syncStr.c_str(), queueSize,
        globalOdomSub_.getTopic().c_str(),
        localOdomSub_.getTopic().c_str(),
        pointCloudSub_.getTopic().c_str());
}

void DataSubscriber::setupWithImageCallback(
    ros::NodeHandle& nh, ros::NodeHandle& pnh,
    int queueSize, bool approxSync)
{
    globalOdomSub_.subscribe(nh, "global_odom", queueSize);
    localOdomSub_.subscribe(nh, "local_odom", queueSize);
    pointCloudSub_.subscribe(nh, "point_cloud", queueSize);
    ros::NodeHandle rgbNh(nh, "rgb");
    ros::NodeHandle rgbPnh(pnh, "rgb");
    image_transport::ImageTransport imageTransport(rgbNh);
    image_transport::TransportHints hints("raw", ros::TransportHints(), rgbPnh);
    cameraInfoSubs_.push_back(
        std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            rgbNh, "camera_info", queueSize));
    imageSubs_.push_back(std::make_unique<image_transport::SubscriberFilter>(
        imageTransport, rgbNh.resolveName("image"), queueSize, hints));
    if (approxSync)
    {
        withImageApproxSync_ = std::make_unique<WithImageApproxSync>(
            WithImageApproxPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_,
            *cameraInfoSubs_[0],
            *imageSubs_[0]);
        withImageApproxSync_->registerCallback(&DataSubscriber::withImageCallback, this);
    }
    else
    {
        withImageExactSync_ = std::make_unique<WithImageExactSync>(
            WithImageExactPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_,
            *cameraInfoSubs_[0],
            *imageSubs_[0]);
        withImageExactSync_->registerCallback(&DataSubscriber::withImageCallback, this);
    }

    std::string syncStr;
    if (approxSync)
    {
        syncStr = "approx";
    }
    else
    {
        syncStr = "exact";
    }
    ROS_INFO("\nDataSubscriber named '%s' subscribed to (%s sync, queue size %d):\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s",
        name_.c_str(), syncStr.c_str(), queueSize,
        globalOdomSub_.getTopic().c_str(),
        localOdomSub_.getTopic().c_str(),
        pointCloudSub_.getTopic().c_str(),
        cameraInfoSubs_[0]->getTopic().c_str(),
        imageSubs_[0]->getTopic().c_str());
}

void DataSubscriber::setupWith2ImagesCallback(
    ros::NodeHandle& nh, ros::NodeHandle& pnh,
    int queueSize, bool approxSync)
{
    globalOdomSub_.subscribe(nh, "global_odom", queueSize);
    localOdomSub_.subscribe(nh, "local_odom", queueSize);
    pointCloudSub_.subscribe(nh, "point_cloud", queueSize);
    ros::NodeHandle rgbNh1(nh, "rgb_1");
    ros::NodeHandle rgbPnh1(pnh, "rgb_1");
    image_transport::ImageTransport imageTransport1(rgbNh1);
    image_transport::TransportHints hints1("raw", ros::TransportHints(), rgbPnh1);
    cameraInfoSubs_.push_back(
        std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            rgbNh1, "camera_info", queueSize));
    imageSubs_.push_back(std::make_unique<image_transport::SubscriberFilter>(
        imageTransport1, rgbNh1.resolveName("image"), queueSize, hints1));
    ros::NodeHandle rgbNh2(nh, "rgb_2");
    ros::NodeHandle rgbPnh2(pnh, "rgb_2");
    image_transport::ImageTransport imageTransport2(rgbNh2);
    image_transport::TransportHints hints2("raw", ros::TransportHints(), rgbPnh2);
    cameraInfoSubs_.push_back(
        std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            rgbNh2, "camera_info", queueSize));
    imageSubs_.push_back(std::make_unique<image_transport::SubscriberFilter>(
        imageTransport2, rgbNh2.resolveName("image"), queueSize, hints2));
    if (approxSync)
    {
        with2ImagesApproxSync_ = std::make_unique<With2ImagesApproxSync>(
            With2ImagesApproxPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_,
            *cameraInfoSubs_[0],
            *imageSubs_[0],
            *cameraInfoSubs_[1],
            *imageSubs_[1]);
        with2ImagesApproxSync_->registerCallback(&DataSubscriber::with2ImagesCallback, this);
    }
    else
    {
        with2ImagesExactSync_ = std::make_unique<With2ImagesExactSync>(
            With2ImagesExactPolicy(queueSize),
            globalOdomSub_,
            localOdomSub_,
            pointCloudSub_,
            *cameraInfoSubs_[0],
            *imageSubs_[0],
            *cameraInfoSubs_[1],
            *imageSubs_[1]);
        with2ImagesExactSync_->registerCallback(&DataSubscriber::with2ImagesCallback, this);
    }

    std::string syncStr;
    if (approxSync)
    {
        syncStr = "approx";
    }
    else
    {
        syncStr = "exact";
    }
    ROS_INFO("\nDataSubscriber named '%s' subscribed to (%s sync, queue size %d):\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s\n" \
        "    %s",
        name_.c_str(), syncStr.c_str(), queueSize,
        globalOdomSub_.getTopic().c_str(),
        localOdomSub_.getTopic().c_str(),
        pointCloudSub_.getTopic().c_str(),
        cameraInfoSubs_[0]->getTopic().c_str(),
        imageSubs_[0]->getTopic().c_str(),
        cameraInfoSubs_[1]->getTopic().c_str(),
        imageSubs_[1]->getTopic().c_str());
}

void DataSubscriber::defaultCallback(
    const nav_msgs::OdometryConstPtr& globalOdomMsg,
    const nav_msgs::OdometryConstPtr& localOdomMsg,
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
    callback_(*globalOdomMsg, *localOdomMsg, *pointCloudMsg, {}, {});
}

void DataSubscriber::withImageCallback(
    const nav_msgs::OdometryConstPtr& globalOdomMsg,
    const nav_msgs::OdometryConstPtr& localOdomMsg,
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg,
    const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
    const sensor_msgs::ImageConstPtr& imageMsg)
{
    callback_(*globalOdomMsg, *localOdomMsg, *pointCloudMsg, {cameraInfoMsg}, {imageMsg});
}

void DataSubscriber::with2ImagesCallback(
    const nav_msgs::OdometryConstPtr& globalOdomMsg,
    const nav_msgs::OdometryConstPtr& localOdomMsg,
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg,
    const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
    const sensor_msgs::ImageConstPtr& imageMsg1,
    const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
    const sensor_msgs::ImageConstPtr& imageMsg2)
{
    callback_(*globalOdomMsg, *localOdomMsg, *pointCloudMsg,
        {cameraInfoMsg1, cameraInfoMsg2}, {imageMsg1, imageMsg2});
}

}
