#pragma once

#include <opencv2/opencv.hpp>


namespace kas_utils {

template <typename T>
class DepthToPointCloud
{
public:
    DepthToPointCloud(float fx, float fy, float cx, float cy, int pool_size);

    T convert(const cv::Mat& depth) const;

    void setCameraIntrinsics(float fx, float fy, float cx, float cy)
    {
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
    }
    void setPoolSize(int pool_size)
    {
        pool_size_ = pool_size;
    }

private:
    static T create_point_cloud();
    static void init_point_cloud(T& point_cloud, int points_number);
    static void append_to_point_cloud(T& point_cloud, float x, float y, float z);
    static void post_process_point_cloud(T& point_cloud, int points_number);

    template<typename PT>
    static bool z_is_valid(PT z);

    static float getDepthScale(const cv::Mat& depth);

    template<typename PT>
    int getPointsNumber(const cv::Mat& depth) const;
    template<typename PT>
    int getPointsNumberPoolSize1(const cv::Mat& depth) const;

    template<typename PT>
    void convertImpl(const cv::Mat& depth, float depth_scale,
        T& point_cloud) const;
    template<typename PT>
    void convertImplPoolSize1(const cv::Mat& depth, float depth_scale,
        T& point_cloud) const;

private:
    float fx_, fy_, cx_, cy_;
    int pool_size_;
};

}