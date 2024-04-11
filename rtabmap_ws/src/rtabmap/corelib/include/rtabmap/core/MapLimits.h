#pragma once

#include <limits>
#include <type_traits>
#include <algorithm>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/proto/MapLimits.pb.h>

namespace rtabmap {

template<typename T>
class MapLimits
{
public:
    MapLimits() :
        minX_(std::numeric_limits<T>::max()),
        minY_(std::numeric_limits<T>::max()),
        maxX_(std::numeric_limits<T>::lowest()),
        maxY_(std::numeric_limits<T>::lowest()) {}
    MapLimits(const T& minX, const T& minY, const T& maxX, const T& maxY)
    {
        set(minX, minY, maxX, maxY);
    }
    void set(const T& minX, const T& minY, const T& maxX, const T& maxY)
    {
        UASSERT(checkLimits(minX, minY, maxX, maxY));
        minX_ = minX;
        minY_ = minY;
        maxX_ = maxX;
        maxY_ = maxY;
    }
    T minX() const
    {
        return minX_;
    }
    T minY() const
    {
        return minY_;
    }
    T maxX() const
    {
        return maxX_;
    }
    T maxY() const
    {
        return maxY_;
    }
    bool operator==(const MapLimits<T>& other) const
    {
        return
            minX_ == other.minX_ &&
            minY_ == other.minY_ &&
            maxX_ == other.maxX_ &&
            maxY_ == other.maxY_;
    }
    bool operator!=(const MapLimits<T>& other) const
    {
        return !operator==(other);
    }
    bool valid() const
    {
        return minX_ != std::numeric_limits<T>::max();
    }
    void update(T x, T y)
    {
        if (minX_ > x)
            minX_ = x;
        if (maxX_ < x)
            maxX_ = x;
        if (minY_ > y)
            minY_ = y;
        if (maxY_ < y)
            maxY_ = y;
    }
    T width() const
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);
        if constexpr (integral)
        {
            return maxX_ - minX_ + static_cast<T>(1);
        }
        if constexpr (floating)
        {
            return maxX_ - minX_;
        }
    }
    T height() const
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);
        if constexpr (integral)
        {
            return maxY_ - minY_ + static_cast<T>(1);
        }
        if constexpr (floating)
        {
            return maxY_ - minY_;
        }
    }
    static MapLimits<T> unite(const MapLimits<T>& a, const MapLimits<T>& b)
    {
        MapLimits<T> res;
        res.minX_ = std::min(a.minX_, b.minX_);
        res.minY_ = std::min(a.minY_, b.minY_);
        res.maxX_ = std::max(a.maxX_, b.maxX_);
        res.maxY_ = std::max(a.maxY_, b.maxY_);
        return res;
    }
    static MapLimits<T> intersect(const MapLimits<T>& a, const MapLimits<T>& b)
    {
        MapLimits<T> res;
        res.minX_ = std::max(a.minX_, b.minX_);
        res.minY_ = std::max(a.minY_, b.minY_);
        res.maxX_ = std::min(a.maxX_, b.maxX_);
        res.maxY_ = std::min(a.maxY_, b.maxY_);
        res.normalize();
        return res;
    }

private:
    static bool checkLimits(const T& minX, const T& minY, const T& maxX, const T& maxY)
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);
        if constexpr (integral)
        {
            return minX <= maxX + static_cast<T>(1) && minY <= maxY + static_cast<T>(1);
        }
        if constexpr (floating)
        {
            return minX <= maxX && minY <= maxY;
        }
    }
    void normalize()
    {
        if (!valid())
        {
            return;
        }
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);
        if constexpr (integral)
        {
            if (minX_ > maxX_ + static_cast<T>(1))
                minX_ = maxX_ + static_cast<T>(1);
            if (minY_ > maxY_ + static_cast<T>(1))
                minY_ = maxY_ + static_cast<T>(1);
        }
        if constexpr (floating)
        {
            if (minX_ > maxX_)
                minX_ = maxX_;
            if (minY_ > maxY_)
                minY_ = maxY_;
        }
    }

private:
    T minX_;
    T minY_;
    T maxX_;
    T maxY_;
};

typedef MapLimits<int> MapLimitsI;
typedef MapLimits<float> MapLimitsF;

proto::MapLimitsI toProto(const MapLimitsI& limits);
MapLimitsI fromProto(const proto::MapLimitsI& proto);

}