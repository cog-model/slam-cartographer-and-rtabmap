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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>

#include <rtabmap/proto/LaserScan.pb.h>

namespace rtabmap {

class RTABMAP_EXP LaserScan
{
public:
    enum Format{kUnknown=0,
        kXY=1,
        kXYI=2,
        kXYNormal=3,
        kXYINormal=4,
        kXYZ=5,
        kXYZI=6,
        kXYZRGB=7,
        kXYZNormal=8,
        kXYZINormal=9,
        kXYZRGBNormal=10};

    static std::string formatName(const Format & format);
    static int channels(const Format & format);
    static bool isScan2d(const Format & format);
    static bool isScanHasNormals(const Format & format);
    static bool isScanHasRGB(const Format & format);
    static bool isScanHasIntensity(const Format & format);

    static int getScanIntensityOffset(const Format & format);
    static int getScanRGBOffset(const Format & format);
    static int getScanNormalsOffset(const Format & format);

    static LaserScan backwardCompatibility(
            const cv::Mat & oldScanFormat,
            int maxPoints = 0,
            int maxRange = 0,
            const Transform & localTransform = Transform::getIdentity());
    static LaserScan backwardCompatibility(
            const cv::Mat & oldScanFormat,
            float minRange,
            float maxRange,
            float angleMin,
            float angleMax,
            float angleInc,
            const Transform & localTransform = Transform::getIdentity());

public:
    LaserScan();
    LaserScan(const LaserScan & data,
            int maxPoints,
            float maxRange,
            const Transform & localTransform = Transform::getIdentity());
    RTABMAP_DEPRECATED(LaserScan(const LaserScan & data,
            int maxPoints,
            float maxRange,
            Format format,
            const Transform & localTransform = Transform::getIdentity()), "Use version without \"format\" argument.");
    LaserScan(const cv::Mat & data,
            int maxPoints,
            float maxRange,
            Format format,
            const Transform & localTransform = Transform::getIdentity());
    RTABMAP_DEPRECATED(LaserScan(const LaserScan & data,
            Format format,
            float minRange,
            float maxRange,
            float angleMin,
            float angleMax,
            float angleIncrement,
            const Transform & localTransform = Transform::getIdentity()), "Use version without \"format\" argument.");
    LaserScan(const LaserScan & data,
            float minRange,
            float maxRange,
            float angleMin,
            float angleMax,
            float angleIncrement,
            const Transform & localTransform = Transform::getIdentity());
    LaserScan(const cv::Mat & data,
            Format format,
            float minRange,
            float maxRange,
            float angleMin,
            float angleMax,
            float angleIncrement,
            const Transform & localTransform = Transform::getIdentity());

    const cv::Mat & data() const {return data_;}
    Format format() const {return format_;}
    std::string formatName() const {return formatName(format_);}
    int channels() const {return data_.channels();}
    int maxPoints() const {return maxPoints_;}
    float rangeMin() const {return rangeMin_;}
    float rangeMax() const {return rangeMax_;}
    float angleMin() const {return angleMin_;}
    float angleMax() const {return angleMax_;}
    float angleIncrement() const {return angleIncrement_;}
    Transform localTransform() const {return localTransform_;}

    bool empty() const {return data_.empty();}
    bool isEmpty() const {return data_.empty();}
    int size() const {return data_.cols;}
    int dataType() const {return data_.type();}
    bool is2d() const {return isScan2d(format_);}
    bool hasNormals() const {return isScanHasNormals(format_);}
    bool hasRGB() const {return isScanHasRGB(format_);}
    bool hasIntensity() const {return isScanHasIntensity(format_);}
    LaserScan clone() const;

    int getIntensityOffset() const {return getScanIntensityOffset(format_);}
    int getRGBOffset() const {return getScanRGBOffset(format_);}
    int getNormalsOffset() const {return getScanNormalsOffset(format_);}

    float & field(unsigned int pointIndex, unsigned int channelOffset)
    {
        return data_.ptr<float>(0, pointIndex)[channelOffset];
    }
    const float & field(unsigned int pointIndex, unsigned int channelOffset) const
    {
        return data_.ptr<float>(0, pointIndex)[channelOffset];
    }

    void clear() {data_ = cv::Mat();}

    /**
     * Concatenate scan's data, localTransform is ignored.
     */
    LaserScan & operator+=(const LaserScan &);
    /**
     * Concatenate scan's data, localTransform is ignored.
     */
    LaserScan operator+(const LaserScan &);

private:
    void init(const cv::Mat & data,
            Format format,
            float minRange,
            float maxRange,
            float angleMin,
            float angleMax,
            float angleIncrement,
            int maxPoints,
            const Transform & localTransform = Transform::getIdentity());

private:
    cv::Mat data_;
    Format format_;
    int maxPoints_;
    float rangeMin_;
    float rangeMax_;
    float angleMin_;
    float angleMax_;
    float angleIncrement_;
    Transform localTransform_;
};

proto::LaserScan toProto(const LaserScan& laserScan);
LaserScan fromProto(const proto::LaserScan& proto);

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_ */
