#pragma once

#include <cstdint>
#include <rtabmap/proto/Color.pb.h>

namespace rtabmap {

class Color
{
public:
    static const Color missingColor;

    Color() { missing_ = true; }
    Color(const Color& other) { data_ = other.data_; }
    Color(int rgb) { setRgb(rgb); };
    Color& operator=(const Color& other) { data_ = other.data_; return *this; }
    bool operator==(const Color& other) const
    {
        return (missing_ && other.missing_) || data_ == other.data_;
    }
    bool operator!=(const Color& other) const
    {
        return !operator==(other);
    }
    int brightness() const
    {
        if (missing_) return -1;
        return (int)r_ + (int)g_ + (int)b_;
    }
    std::uint8_t& b() { return b_; }
    std::uint8_t b() const { return b_; }
    std::uint8_t& g() { return g_; }
    std::uint8_t g() const { return g_; }
    std::uint8_t& r() { return r_; }
    std::uint8_t r() const { return r_; }
    bool& missing() { return missing_; }
    bool missing() const { return missing_; }
    int rgb() const { return rgb_; }
    void setRgb(int rgb) { rgb_ = rgb; missing_ = false; };
    int data() const { return data_; }
    void setData(int data) { data_ = data; }

private:
    union
    {
        union
        {
            struct
            {
                std::uint8_t b_;
                std::uint8_t g_;
                std::uint8_t r_;
                bool missing_;
            };
            int rgb_;
        };
        int data_;
    };
};

proto::Color toProto(const Color& color);
Color fromProto(const proto::Color& proto);

}