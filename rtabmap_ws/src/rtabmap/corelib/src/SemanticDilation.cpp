#include <rtabmap/core/SemanticDilation.h>
#include <limits>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

SemanticDilation::SemanticDilation(const Parameters& parameters)
{
    parseParameters(parameters);
}

void SemanticDilation::parseParameters(const Parameters& parameters)
{
    dilationSize_ = parameters.dilationSize;
    initialize();
}

void SemanticDilation::initialize()
{
    UASSERT(dilationSize_ >= 0);
    if (dilationSize_ > 0)
    {
        dilationSizeSqr_ = dilationSize_ * dilationSize_;
        dilationWidth_ = 2 * dilationSize_ + 1;
        computeDilationPixels();
    }
}

// Instantiated below
template <typename T, size_t size>
cv::Mat SemanticDilation::dilate(const cv::Mat& image,
    const T (&backgroundColors)[size], bool dilateBackground /* false */) const
{
    constexpr bool isColor = std::is_same<T, typename cv::Vec3b>::value;
    constexpr bool isGray =
        std::is_same<T, std::uint8_t>::value ||
        std::is_same<T, std::int8_t>::value;
    static_assert(isColor || isGray, "Unknown image type for dilation");
    if constexpr(isColor)
    {
        UASSERT(image.type() == CV_8UC3);
    }
    if constexpr(isGray)
    {
        UASSERT(image.type() == CV_8U || image.type() == CV_8S);
    }
    UASSERT(dilationSize_ > 0);  // we need to make a copy of image
        // even if no dilation is applied, but it consumes time
        // (not much but it does)
    cv::Mat dilated = image.clone();
    for (int y = 0; y < image.rows; y++)
    {
        int currentDilationWidth = dilationWidth_;
        for (int x = 0; x < image.cols; x++)
        {
            currentDilationWidth =
                std::min(currentDilationWidth + 1, dilationWidth_);
            const T& color = image.at<T>(y, x);
            bool colorIsBackground =
                std::find(std::begin(backgroundColors),
                    std::end(backgroundColors), color) !=
                std::end(backgroundColors);
            if (colorIsBackground != dilateBackground)
            {
                continue;
            }
            for (int i = 0; i < dilationWidthToPixelsNum_[currentDilationWidth]; i++)
            {
                PixelCoords pixelCoords = dilationPixels_[i];
                pixelCoords.y += y;
                pixelCoords.x += x;
                if (!pixelCoords.inFrame(dilated.rows, dilated.cols))
                {
                    continue;
                }
                dilated.at<T>(pixelCoords.y, pixelCoords.x) = color;
            }
            currentDilationWidth = 0;
        }
    }
    return dilated;
}

void SemanticDilation::computeDilationPixels()
{
    dilationPixels_.clear();
    dilationWidthToPixelsNum_.clear();
    std::map<int, std::set<int>> rowToCols;
    int numPixels = 0;
    for (int y = -dilationSize_; y <= dilationSize_; y++)
    {
        for (int x = -dilationSize_; x <= dilationSize_; x++)
        {
            if (y * y + x * x <= dilationSizeSqr_)
            {
                rowToCols[y].insert(x);
                numPixels++;
            }
        }
    }
    dilationWidthToPixelsNum_.push_back(0);
    while (numPixels > 0)
    {
        for (auto& rowCols : rowToCols)
        {
            int row = rowCols.first;
            std::set<int>& cols = rowCols.second;
            if (cols.empty())
            {
                continue;
            }
            auto lastColIt = std::prev(cols.end());
            dilationPixels_.push_back(PixelCoords{row, *lastColIt});
            cols.erase(lastColIt);
            numPixels--;
        }
        dilationWidthToPixelsNum_.push_back(dilationPixels_.size());
    }
}

template cv::Mat SemanticDilation::dilate(const cv::Mat&,
    const cv::Vec3b(&)[1], bool) const;
template cv::Mat SemanticDilation::dilate(const cv::Mat&,
    const std::uint8_t(&)[1], bool) const;
template cv::Mat SemanticDilation::dilate(const cv::Mat&,
    const std::int8_t(&)[1], bool) const;

}
