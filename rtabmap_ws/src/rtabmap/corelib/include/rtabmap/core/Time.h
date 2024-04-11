#pragma once

#include <rtabmap/utilite/ULogger.h>

#include <limits>
#include <cmath>

#include <rtabmap/proto/Time.pb.h>

namespace rtabmap {

struct Time
{
    Time() : sec(0), nsec(0) {}
    Time(uint32_t otherSec, uint32_t otherNSec) :
        sec(otherSec), nsec(otherNSec) {}
    Time(const Time& other) : sec(other.sec), nsec(other.nsec) {}
    Time(double stamp)
    {
        fromSec(stamp);
    }
    static Time min()
    {
        return Time();
    }
    static Time max()
    {
        return Time(
            std::numeric_limits<uint32_t>::max(),
            static_cast<uint32_t>(1e9 - 1));
    }
    Time& operator=(const Time& other)
    {
        sec = other.sec;
        nsec = other.nsec;
        return *this;
    }
    double toSec() const
    {
        return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    }
    void fromSec(double stamp)
    {
        int64_t sec64 = static_cast<int64_t>(std::floor(stamp));
        UASSERT(sec64 >= 0 && sec64 <= std::numeric_limits<uint32_t>::max());
        sec = static_cast<uint32_t>(sec64);
        nsec = static_cast<uint32_t>(std::round((stamp - sec) * 1e9));
        // avoid rounding errors
        sec += (nsec / 1000000000ul);
        nsec %= 1000000000ul;
    }
    bool operator==(const Time& other) const
    {
        return sec == other.sec && nsec == other.nsec;
    }
    bool operator!=(const Time& other) const
    {
        return sec != other.sec || nsec != other.nsec;
    }
    bool operator<(const Time& other) const
    {
        if (sec == other.sec)
        {
            return nsec < other.nsec;
        }
        return sec < other.sec;
    }
    bool operator<=(const Time& other) const
    {
        if (sec == other.sec)
        {
            return nsec <= other.nsec;
        }
        return sec < other.sec;
    }
    bool operator>(const Time& other) const
    {
        if (sec == other.sec)
        {
            return nsec > other.nsec;
        }
        return sec > other.sec;
    }
    bool operator>=(const Time& other) const
    {
        if (sec == other.sec)
        {
            return nsec >= other.nsec;
        }
        return sec > other.sec;
    }

    uint32_t sec;
    uint32_t nsec;
};

proto::Time toProto(const Time& time);
Time fromProto(const proto::Time& proto);

}
