#include <rtabmap/core/Time.h>

namespace rtabmap {

proto::Time toProto(const Time& time)
{
    proto::Time proto;
    proto.set_sec(time.sec);
    proto.set_nsec(time.nsec);
    return proto;
}

Time fromProto(const proto::Time& proto)
{
    Time time(proto.sec(), proto.nsec());
    return time;
}

}

