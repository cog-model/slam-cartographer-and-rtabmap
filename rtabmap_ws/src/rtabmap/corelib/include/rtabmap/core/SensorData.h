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

#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/proto/SensorData.pb.h>

#include <opencv2/core/core.hpp>

#include <vector>
#include <optional>

namespace rtabmap
{

class SensorData
{
public:
    SensorData() = default;

    template<typename T, typename U>
    void addImage(T&& cameraModel, U&& image)
    {
        cameraModels_.emplace_back(std::forward<T>(cameraModel));
        images_.emplace_back(std::forward<U>(image));
    }
    template<typename T, typename U>
    void setImages(T&& cameraModels, U&& images)
    {
        UASSERT(cameraModels.size() == images.size());
        cameraModels_ = std::forward<T>(cameraModels);
        images_ = std::forward<U>(images);
    }
    int numImages() const { return cameraModels_.size(); }
    void clearImages()
    {
        cameraModels_.clear(); 
        images_.clear();
    }
    const std::vector<CameraModel>& cameraModels() const { return cameraModels_; }
    const std::vector<cv::Mat>& images() const { return images_; }

    template<typename T>
    void setLaserScan(T&& laserScan)
    {
        laserScan_ = std::forward<T>(laserScan);
    }
    bool hasLaserScan() const { return laserScan_.has_value(); }
    void clearLaserScan() { laserScan_.reset(); }
    const LaserScan& laserScan() const { return laserScan_.value(); }

private:
    std::vector<CameraModel> cameraModels_;
    std::vector<cv::Mat> images_;

    std::optional<LaserScan> laserScan_;
};

proto::SensorData toProto(const SensorData& sensorData);
SensorData fromProto(const proto::SensorData& proto);

}
