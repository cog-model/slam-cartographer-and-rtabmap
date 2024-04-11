#pragma once

#include <rtabmap/utilite/ULogger.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

namespace rtabmap {

class SemanticDilation
{
public:
    struct Parameters
    {
        int dilationSize = 0;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["DilationSize"])
            {
                parameters.dilationSize = node["DilationSize"].as<int>();
            }
            return parameters;
        }
    };

private:
    struct PixelCoords {
        bool inFrame(int h, int w) const {
            return y >= 0 && x >= 0 && y < h && x < w;
        }
        int y;
        int x;
    };

public:
    SemanticDilation(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    template <typename T, size_t size>
    cv::Mat dilate(const cv::Mat& image, const T (&backgroundColors)[size],
        bool dilateBackground = false) const;

    inline int dilationSize() const
    {
        return dilationSize_;
    }

private:
    void initialize();
    void computeDilationPixels();

private:
    int dilationSize_;
    int dilationSizeSqr_;
    int dilationWidth_;

    std::vector<PixelCoords> dilationPixels_;
    std::vector<int> dilationWidthToPixelsNum_;
};

}
