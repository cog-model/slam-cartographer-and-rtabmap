#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace rtabmap {

std::string compressString(const std::string& uncompressed);
std::string decompressString(const std::string& compressed);

std::string compressMat(const cv::Mat& mat);
cv::Mat decompressMat(const std::string& compressed);

}
