#include "kas_utils/depth_to_point_cloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <string>
#include <utility>
#include <type_traits>
#include <stdexcept>


namespace kas_utils {

template<typename T>
DepthToPointCloud<T>::DepthToPointCloud(
        float fx, float fy, float cx, float cy, int pool_size) :
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), pool_size_(pool_size) {}


template<typename T>
inline T DepthToPointCloud<T>::create_point_cloud()
{
    static_assert(
        std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value ||
        std::is_same<T, std::pair<float*, int>>::value);
    if constexpr(std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value)
    {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr();
    }
    if constexpr(std::is_same<T, std::pair<float*, int>>::value)
    {
        return std::make_pair(static_cast<float*>(nullptr), 0);
    }
}


template<typename T>
inline void DepthToPointCloud<T>::init_point_cloud(T& point_cloud,
    int points_number)
{
    static_assert(
        std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value ||
        std::is_same<T, std::pair<float*, int>>::value);
    if constexpr(std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value)
    {
        point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        point_cloud->reserve(points_number);
    }
    if constexpr(std::is_same<T, std::pair<float*, int>>::value)
    {
        point_cloud.first = new float[points_number * 3];
        point_cloud.second = points_number;
    }
}


template<typename T>
inline void DepthToPointCloud<T>::append_to_point_cloud(T& point_cloud,
    float x, float y, float z)
{
    static_assert(
        std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value ||
        std::is_same<T, std::pair<float*, int>>::value);
    if constexpr(std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value)
    {
        point_cloud->emplace_back(x, y, z);
    }
    if constexpr(std::is_same<T, std::pair<float*, int>>::value)
    {
        if (point_cloud.second == 0)
        {
            throw std::runtime_error(
                "DepthToPointCloud: Number of point is larger than expected. "
                "This should not happen.");
        }
        *point_cloud.first++ = x;
        *point_cloud.first++ = y;
        *point_cloud.first++ = z;
        point_cloud.second--;
    }
}


template<typename T>
inline void DepthToPointCloud<T>::post_process_point_cloud(T& point_cloud,
    int points_number)
{
    static_assert(
        std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value ||
        std::is_same<T, std::pair<float*, int>>::value);
    if constexpr(std::is_same<T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value)
    {
        // nothing
    }
    if constexpr(std::is_same<T, std::pair<float*, int>>::value)
    {
        if (point_cloud.second != 0)
        {
            throw std::runtime_error(
                "DepthToPointCloud: Number of point is less than expected. "
                "This should not happen.");
        }
        point_cloud.first -= points_number * 3;
        point_cloud.second = points_number;
    }
}


template<typename T>
template<typename PT>
inline bool DepthToPointCloud<T>::z_is_valid(PT z)
{
    static_assert(
        std::is_same<PT, std::uint16_t>::value ||
        std::is_same<PT, float>::value ||
        std::is_same<PT, double>::value);
    if constexpr(std::is_same<PT, std::uint16_t>::value)
    {
        return (z != static_cast<std::uint16_t>(0));
    }
    if constexpr(std::is_same<PT, float>::value)
    {
        return std::isfinite(z) && (z > 0.f);
    }
    if constexpr(std::is_same<PT, double>::value)
    {
        return std::isfinite(z) && (z > 0.);
    }
}


template<typename T>
float DepthToPointCloud<T>::getDepthScale(const cv::Mat& depth)
{
    if (depth.type() == CV_16UC1)
    {
        return 0.001f;
    }
    else if (depth.type() == CV_32FC1 || depth.type() == CV_64FC1)
    {
        return 1.f;
    }
    else
    {
        return -1.f;
    }
}


template<typename T>
T DepthToPointCloud<T>::convert(const cv::Mat& depth) const
{
    if (depth.dims != 2)
    {
        throw std::runtime_error(
            "DepthToPointCloud: Wrong number of dimentions in input depth image. "
            "Expected 2, got " + std::to_string(depth.dims) + ".");
    }
    T point_cloud = create_point_cloud();
    float depth_scale = getDepthScale(depth);
    if (depth_scale < 0.f)
    {
        throw std::runtime_error(
            "DepthToPointCloud: Unknown format of input depth image. "
            "Expected "
            "CV_16UC1 (" + std::to_string(CV_16UC1) + "), "
            "CV_32FC1 (" + std::to_string(CV_32FC1) + ") or "
            "CV_64FC1 (" + std::to_string(CV_64FC1) + "), "
            "got " + std::to_string(depth.type()) + ".");
    }

    int points_number;
    if (pool_size_ > 1)
    {
        switch (depth.type())
        {
        case CV_16UC1:
            points_number = getPointsNumber<std::uint16_t>(depth);
            break;
        case CV_32FC1:
            points_number = getPointsNumber<float>(depth);
            break;
        case CV_64FC1:
            points_number = getPointsNumber<double>(depth);
            break;
        }
    }
    else
    {
        switch (depth.type())
        {
        case CV_16UC1:
            points_number = getPointsNumberPoolSize1<std::uint16_t>(depth);
            break;
        case CV_32FC1:
            points_number = getPointsNumberPoolSize1<float>(depth);
            break;
        case CV_64FC1:
            points_number = getPointsNumberPoolSize1<double>(depth);
            break;
        }
    }

    init_point_cloud(point_cloud, points_number);
    if (pool_size_ > 1)
    {
        switch (depth.type())
        {
        case CV_16UC1:
            convertImpl<std::uint16_t>(depth, depth_scale, point_cloud);
            break;
        case CV_32FC1:
            convertImpl<float>(depth, depth_scale, point_cloud);
            break;
        case CV_64FC1:
            convertImpl<double>(depth, depth_scale, point_cloud);
            break;
        }
    }
    else
    {
        switch (depth.type())
        {
        case CV_16UC1:
            convertImplPoolSize1<std::uint16_t>(depth, depth_scale, point_cloud);
            break;
        case CV_32FC1:
            convertImplPoolSize1<float>(depth, depth_scale, point_cloud);
            break;
        case CV_64FC1:
            convertImplPoolSize1<double>(depth, depth_scale, point_cloud);
            break;
        }
    }
    post_process_point_cloud(point_cloud, points_number);
    return point_cloud;
}


template<typename T>
template<typename PT>
int DepthToPointCloud<T>::getPointsNumber(const cv::Mat& depth) const
{
    int points_number = 0;
    std::vector<std::uint8_t> has_valid_point(
        (depth.cols - 1) / pool_size_ + 1, static_cast<std::uint8_t>(0));
    for (int j = 0; j < depth.rows; j++)
    {
        for (int i = 0; i < depth.cols; i++)
        {
            const PT& z = depth.at<PT>(j, i);
            if (z_is_valid(z))
            {
                std::uint8_t& has_valid = has_valid_point[i / pool_size_];
                has_valid = static_cast<std::uint8_t>(1);
            }
        }
        if ((j % pool_size_ == pool_size_ - 1) || (j == depth.rows - 1))
        {
            for (std::uint8_t& has_valid : has_valid_point)
            {
                if (has_valid)
                {
                    points_number++;
                }
                has_valid = static_cast<std::uint8_t>(0);
            }
        }
    }
    return points_number;
}


template<typename T>
template<typename PT>
int DepthToPointCloud<T>::getPointsNumberPoolSize1(const cv::Mat& depth) const
{
    int points_number = 0;
    for (int j = 0; j < depth.rows; j++)
    {
        for (int i = 0; i < depth.cols; i++)
        {
            const PT& z = depth.at<PT>(j, i);
            if (z_is_valid(z))
            {
                points_number++;
            }
        }
    }
    return points_number;
}


template<typename T>
template<typename PT>
void DepthToPointCloud<T>::convertImpl(const cv::Mat& depth,
    float depth_scale, T& point_cloud) const
{
    std::vector<PT> min_z_pooled((depth.cols - 1) / pool_size_ + 1, static_cast<PT>(0));
    for (int j = 0; j < depth.rows; j++)
    {
        for (int i = 0; i < depth.cols; i++)
        {
            const PT& z = depth.at<PT>(j, i);
            if (z_is_valid(z))
            {
                PT& min_z = min_z_pooled[i / pool_size_];
                if (min_z == static_cast<PT>(0))
                {
                    min_z = z;
                }
                else
                {
                    min_z = std::min(min_z, z);
                }
            }
        }
        if ((j % pool_size_ == pool_size_ - 1) || (j == depth.rows - 1))
        {
            int real_j;
            if (j != depth.rows - 1)
            {
                real_j = (j - pool_size_ / 2);
            }
            else
            {
                real_j = (depth.rows - 1 - ((depth.rows - 1) % pool_size_ + 1) / 2);
            }
            for (int i = 0; i < min_z_pooled.size(); i++)
            {
                PT& min_z = min_z_pooled[i];
                if (min_z != static_cast<PT>(0))
                {
                    float z = static_cast<float>(min_z) * depth_scale;
                    int real_i;
                    if (i < min_z_pooled.size() - 1)
                    {
                        real_i = (i * pool_size_ + pool_size_ / 2);
                    }
                    else
                    {
                        real_i = (depth.cols - 1 - (depth.cols % pool_size_) / 2);
                    }
                    float x = (real_i - cx_) / fx_ * z;
                    float y = (real_j - cy_) / fy_ * z;
                    append_to_point_cloud(point_cloud, x, y, z);
                }
                min_z = static_cast<PT>(0);
            }
        }
    }
}


template<typename T>
template<typename PT>
void DepthToPointCloud<T>::convertImplPoolSize1(const cv::Mat& depth,
    float depth_scale, T& point_cloud) const
{
    for (int j = 0; j < depth.rows; j++)
    {
        for (int i = 0; i < depth.cols; i++)
        {
            const PT& z_depth = depth.at<PT>(j, i);
            if (z_is_valid(z_depth))
            {
                float z = static_cast<float>(z_depth) * depth_scale;
                float x = (i - cx_) / fx_ * z;
                float y = (j - cy_) / fy_ * z;
                append_to_point_cloud(point_cloud, x, y, z);
            }
        }
    }
}


template class DepthToPointCloud<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
template class DepthToPointCloud<std::pair<float*, int>>;

}