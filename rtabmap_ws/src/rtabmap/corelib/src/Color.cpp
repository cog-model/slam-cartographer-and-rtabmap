#include <rtabmap/core/Color.h>

namespace rtabmap {

const Color Color::missingColor;

proto::Color toProto(const Color& color)
{
    proto::Color proto;
    proto.set_data(color.data());
    return proto;
}

Color fromProto(const proto::Color& proto)
{
    Color color;
    color.setData(proto.data());
    return color;
}

}
