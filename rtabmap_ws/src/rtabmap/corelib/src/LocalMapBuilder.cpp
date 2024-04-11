#include <rtabmap/core/LocalMapBuilder.h>
#include <limits>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

const cv::Vec3b LocalMapBuilder::semanticBackgroundColor(0, 0, 0);

LocalMapBuilder::LocalMapBuilder(const Parameters& parameters)
{
    parseParameters(parameters);
}

void LocalMapBuilder::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    maxVisibleRange_ = parameters.maxVisibleRange;
    minObstacleHeight_ = parameters.minObstacleHeight;
    maxObstacleHeight_ = parameters.maxObstacleHeight;
    minSemanticRange_ = parameters.minSemanticRange;
    maxSemanticRange_ = parameters.maxSemanticRange;
    enableRayTracing_ = parameters.enableRayTracing;
    maxRange2d_ = parameters.maxRange2d;
    sensorBlindRange2d_ = parameters.sensorBlindRange2d;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(minObstacleHeight_ < maxObstacleHeight_);
    UASSERT(minSemanticRange_ >= 0.0f);
    UASSERT(maxSemanticRange_ < 0.0f || minSemanticRange_ < maxSemanticRange_);
    UASSERT(sensorBlindRange2d_ >= 0.0f);

    if (maxVisibleRange_ >= 0.0f)
    {
        maxVisibleRangeSqr_ = maxVisibleRange_ * maxVisibleRange_;
    }
    else
    {
        maxVisibleRangeSqr_ = -1.0f;
    }
    minSemanticRangeSqr_ = minSemanticRange_ * minSemanticRange_;
    if (maxSemanticRange_ >= 0.0f)
    {
        maxSemanticRangeSqr_ = maxSemanticRange_ * maxSemanticRange_;
    }
    else
    {
        maxSemanticRangeSqr_ = -1.0f;
    }
    if (maxRange2d_ >= 0.0f)
    {
        maxRange2dSqr_ = maxRange2d_ * maxRange2d_;
    }
    else
    {
        maxRange2dSqr_ = -1.0f;
    }
    sensorBlindRange2dSqr_ = sensorBlindRange2d_ * sensorBlindRange2d_;

    semanticDilation_ = std::make_unique<SemanticDilation>(
        parameters.semanticDilationParameters);
    if (enableRayTracing_)
    {
        RayTracing::Parameters rayTracingParameters = parameters.rayTracingParameters;
        rayTracingParameters.cellSize = parameters.cellSize;
        rayTracing_ = std::make_unique<RayTracing>(rayTracingParameters);
    }
    else
    {
        rayTracing_.reset();
    }
}

std::shared_ptr<LocalMap> LocalMapBuilder::createLocalMap(
    const SensorData& sensorData, const Time& time,
    const Transform& fromUpdatedPose) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__createLocalMap);
    UASSERT(!sensorData.laserScan().isEmpty() &&
            !sensorData.laserScan().is2d());
    UASSERT(!sensorData.laserScan().localTransform().isNull());
    UASSERT(!fromUpdatedPose.isNull());
    const LaserScan& laserScan = sensorData.laserScan();
    const Transform& transform = laserScan.localTransform();
    Eigen::Matrix3Xf points, obstacles;
    points = convertLaserScan(laserScan);
    if (maxVisibleRangeSqr_ >= 0.0f)
    {
        points = filterMaxVisibleRange(points);
    }
    points = transformPoints(points, transform);
    obstacles = getObstaclePoints(points);

    std::vector<Color> colors;
    if (sensorData.numImages())
    {
        if (semanticDilation_->dilationSize() > 0)
        {
            std::vector<cv::Mat> dilatedImages;
            for (const cv::Mat& image : sensorData.images())
            {
                MEASURE_BLOCK_TIME(LocalMapBuilder__dilate);
                cv::Mat dilatedImage =
                    semanticDilation_->dilate(image, {semanticBackgroundColor});
                dilatedImages.push_back(dilatedImage);
                lastDilatedSemantic_ = dilatedImage;
            }
            colors = getPointsColors(obstacles, dilatedImages,
                sensorData.cameraModels());
        }
        else
        {
            colors = getPointsColors(obstacles, sensorData.images(),
                sensorData.cameraModels());
        }
    }
    else
    {
        colors.resize(obstacles.cols(), Color::missingColor);
    }

    Eigen::Vector2f sensor;
    sensor.x() = transform.translation().x();
    sensor.y() = transform.translation().y();
    LocalMap::ColoredGrid coloredGrid =
        coloredGridFromObstacles(obstacles, colors, sensor);
    if (enableRayTracing_)
    {
        traceRays(coloredGrid, sensor);
    }

    auto localMap = std::make_shared<LocalMap>(
        coloredGrid, maxRange2dSqr_, true /* duplicatePoints */);
    localMap->setSensorBlindRange2dSqr(sensorBlindRange2dSqr_);
    localMap->setToSensor(sensorData.laserScan().localTransform());
    localMap->setFromUpdatedPose(fromUpdatedPose);
    localMap->setTime(time);
    return localMap;
}

Eigen::Matrix3Xf LocalMapBuilder::convertLaserScan(const LaserScan& laserScan) const
{
    Eigen::Matrix3Xf points(3, laserScan.size());
    for (int i = 0; i < laserScan.size(); i++)
    {
        float x = laserScan.field(i, 0);
        float y = laserScan.field(i, 1);
        float z = laserScan.field(i, 2);
        points(0, i) = x;
        points(1, i) = y;
        points(2, i) = z;
    }
    return points;
}

Eigen::Matrix3Xf LocalMapBuilder::filterMaxVisibleRange(const Eigen::Matrix3Xf& points) const
{
    std::vector<int> indices;
    indices.reserve(points.cols());
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        float z = points(2, i);
        if (x * x + y * y + z * z <= maxVisibleRangeSqr_)
        {
            indices.push_back(i);
        }
    }

    Eigen::Matrix3Xf filtered(3, indices.size());
    int i = 0;
    for (int index : indices)
    {
        filtered(0, i) = points(0, index);
        filtered(1, i) = points(1, index);
        filtered(2, i) = points(2, index);
        i++;
    }
    return filtered;
}

Eigen::Matrix3Xf LocalMapBuilder::transformPoints(const Eigen::Matrix3Xf& points,
    const Transform& transform) const
{
    Eigen::Matrix3Xf transformed =
        (transform.toEigen3fRotation() * points).colwise() +
        transform.toEigen3fTranslation();
    return transformed;
}

Eigen::Matrix3Xf LocalMapBuilder::getObstaclePoints(const Eigen::Matrix3Xf& points) const
{
    std::vector<int> obstaclePointsIndices;
    obstaclePointsIndices.reserve(points.cols());
    for (int i = 0; i < points.cols(); i++)
    {
        float z = points(2, i);
        if (minObstacleHeight_ <= z && z <= maxObstacleHeight_)
        {
            obstaclePointsIndices.push_back(i);
        }
    }

    Eigen::Matrix3Xf obstaclePoints(3, obstaclePointsIndices.size());
    int i = 0;
    for (int index : obstaclePointsIndices)
    {
        obstaclePoints(0, i) = points(0, index);
        obstaclePoints(1, i) = points(1, index);
        obstaclePoints(2, i) = points(2, index);
        i++;
    }
    return obstaclePoints;
}

std::vector<Color> LocalMapBuilder::getPointsColors(
    const Eigen::Matrix3Xf& points,
    const std::vector<cv::Mat>& images,
    const std::vector<CameraModel>& cameraModels) const
{
    UASSERT(images.size() == cameraModels.size());
    std::vector<Color> colors;
    colors.resize(points.cols(), Color::missingColor);
    for (int camI = 0; camI < images.size(); camI++)
    {
        const cv::Mat& image = images[camI];
        UASSERT(image.type() == CV_8UC3);
        const CameraModel& cameraModel = cameraModels[camI];
        const Transform& transform = cameraModel.localTransform().inverse();
        const Eigen::Matrix3Xf& transformedPoints =
            (transform.toEigen3fRotation() * points).colwise() +
            transform.toEigen3fTranslation();
        for (int i = 0; i < transformedPoints.cols(); i++)
        {
            float x = transformedPoints(0, i);
            float y = transformedPoints(1, i);
            float z = transformedPoints(2, i);
            float rangeSqr = x * x + y * y + z * z;
            int u, v;
            cameraModel.reproject(x, y, z, u, v);
            if (cameraModel.inFrame(u, v) && z > 0 &&
                rangeSqr >= minSemanticRangeSqr_ &&
                (maxSemanticRangeSqr_ < 0.0f || rangeSqr <= maxSemanticRangeSqr_))
            {
                const cv::Vec3b& pixelColor = image.at<cv::Vec3b>(v, u);
                if (pixelColor == semanticBackgroundColor)
                {
                    continue;
                }
                colors[i].b() = pixelColor[0];
                colors[i].g() = pixelColor[1];
                colors[i].r() = pixelColor[2];
                colors[i].missing() = false;
            }
        }
    }
    return colors;
}

LocalMap::ColoredGrid LocalMapBuilder::coloredGridFromObstacles(
    const Eigen::Matrix3Xf& points,
    const std::vector<Color>& colors,
    const Eigen::Vector2f& sensor) const
{
    MapLimitsF limitsF;
    limitsF.update(sensor.x(), sensor.y());
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        limitsF.update(x, y);
    }
    if (enableRayTracing_ && rayTracing_->traceIntoUnknownSpace())
    {
        float range = rayTracing_->maxTracingRange();
        limitsF.update(sensor.x() - range, sensor.y() - range);
        limitsF.update(sensor.x() + range, sensor.y() + range);
    }

    LocalMap::ColoredGrid coloredGrid;
    coloredGrid.cellSize = cellSize_;
    coloredGrid.limits.set(
        std::floor(limitsF.minX() / cellSize_),
        std::floor(limitsF.minY() / cellSize_),
        std::floor(limitsF.maxX() / cellSize_),
        std::floor(limitsF.maxY() / cellSize_));
    int height = coloredGrid.limits.height();
    int width = coloredGrid.limits.width();
    coloredGrid.grid = cv::Mat(height, width, CV_8S, LocalMap::ColoredGrid::unknownCellValue);
    coloredGrid.colors = cv::Mat(height, width, CV_32S, Color::missingColor.data());
    const int& minX = coloredGrid.limits.minX();
    const int& minY = coloredGrid.limits.minY();
    for (int i = 0; i < points.cols(); i++)
    {
        float xf = points(0, i);
        float yf = points(1, i);
        int y = std::floor(yf / cellSize_) - minY;
        int x = std::floor(xf / cellSize_) - minX;
        coloredGrid.grid.at<std::int8_t>(y, x) = LocalMap::ColoredGrid::occupiedCellValue;
        const Color& pointColor = colors[i];
        if (pointColor != Color::missingColor)
        {
            Color& cellColor =
                reinterpret_cast<Color&>(coloredGrid.colors.at<std::int32_t>(y, x));
            if (pointColor.brightness() > cellColor.brightness())
            {
                cellColor = pointColor;
            }
        }
    }
    return coloredGrid;
}

void LocalMapBuilder::traceRays(LocalMap::ColoredGrid& coloredGrid,
    const Eigen::Vector2f& sensor) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__traceRays);
    RayTracing::Cell origin;
    origin.y = std::floor(sensor.y() / cellSize_) - coloredGrid.limits.minY();
    origin.x = std::floor(sensor.x() / cellSize_) - coloredGrid.limits.minX();
    rayTracing_->traceRays(coloredGrid.grid, origin,
        LocalMap::ColoredGrid::occupiedCellValue, LocalMap::ColoredGrid::emptyCellValue);
}

}
