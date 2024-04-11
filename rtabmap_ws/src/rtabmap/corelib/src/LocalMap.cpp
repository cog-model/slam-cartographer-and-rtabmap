#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Compression.h>

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

LocalMap::LocalMap() :
    numObstacles_(0),
    numEmpty_(0) {}

LocalMap::LocalMap(const Properties& properties) :
    numObstacles_(0),
    numEmpty_(0),
    properties_(properties) {}

LocalMap::LocalMap(const ColoredGrid& coloredGrid,
    float maxRange2dSqr, bool duplicatePoints)
{
    fromColoredGrid(coloredGrid, maxRange2dSqr, duplicatePoints);
}

LocalMap::LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints,
        const Properties& properties) :
    properties_(properties)
{
    fromColoredGrid(coloredGrid, maxRange2dSqr, duplicatePoints);
}

void LocalMap::fromColoredGrid(const ColoredGrid& coloredGrid,
    float maxRange2dSqr, bool duplicatePoints)
{
    UASSERT(coloredGrid.cellSize > 0.0f);
    UASSERT(coloredGrid.limits.valid());
    UASSERT(coloredGrid.grid.type() == CV_8S);
    UASSERT(coloredGrid.colors.type() == CV_32S);
    UASSERT(coloredGrid.grid.rows == coloredGrid.limits.height());
    UASSERT(coloredGrid.grid.cols == coloredGrid.limits.width());
    UASSERT(coloredGrid.colors.rows == coloredGrid.limits.height());
    UASSERT(coloredGrid.colors.cols == coloredGrid.limits.width());

    const float& cellSize = coloredGrid.cellSize;
    const int& minX = coloredGrid.limits.minX();
    const int& minY = coloredGrid.limits.minY();

    std::vector<std::pair<int, int>> occupiedCells;
    std::vector<std::pair<int, int>> emptyCells;
    for (int y = 0; y < coloredGrid.grid.rows; y++)
    {
        for (int x = 0; x < coloredGrid.grid.cols; x++)
        {
            if (maxRange2dSqr >= 0.0f)
            {
                float xf = (x + minX + 0.5f) * cellSize;
                float yf = (y + minY + 0.5f) * cellSize;
                if (xf * xf + yf * yf > maxRange2dSqr)
                {
                    continue;
                }
            }
            std::int8_t value = coloredGrid.grid.at<std::int8_t>(y, x);
            if (value == ColoredGrid::occupiedCellValue)
            {
                occupiedCells.emplace_back(y, x);
            }
            else if (value == ColoredGrid::emptyCellValue)
            {
                emptyCells.emplace_back(y, x);
            }
        }
    }

    int numPoints = occupiedCells.size() + emptyCells.size();
    colors_.clear();
    if (!duplicatePoints)
    {
        numObstacles_ = occupiedCells.size();
        numEmpty_ = emptyCells.size();
        points_.resize(3, numPoints);
        colors_.reserve(numPoints);
    }
    else
    {
        numObstacles_ = occupiedCells.size() * 2;
        numEmpty_ = emptyCells.size() * 2;
        points_.resize(3, numPoints * 2);
        colors_.reserve(numPoints * 2);
    }
    for (int i = 0; i < numPoints; i++)
    {
        int y, x;
        if (i < (int)occupiedCells.size())
        {
            const std::pair<int, int>& occupiedCell = occupiedCells[i];
            y = occupiedCell.first;
            x = occupiedCell.second;
        }
        else
        {
            const std::pair<int, int>& emptyCell = emptyCells[i - occupiedCells.size()];
            y = emptyCell.first;
            x = emptyCell.second;
        }
        const Color& color =
            reinterpret_cast<const Color&>(coloredGrid.colors.at<int>(y, x));
        if (!duplicatePoints)
        {
            float xf = (x + minX + 0.5f) * cellSize;
            float yf = (y + minY + 0.5f) * cellSize;
            points_(0, i) = xf;
            points_(1, i) = yf;
            points_(2, i) = 0.0f;
            colors_.push_back(color);
        }
        else
        {
            float xf1 = (x + minX + 0.25f) * cellSize;
            float yf1 = (y + minY + 0.25f) * cellSize;
            float xf2 = (x + minX + 0.75f) * cellSize;
            float yf2 = (y + minY + 0.75f) * cellSize;
            points_(0, i * 2) = xf1;
            points_(1, i * 2) = yf1;
            points_(2, i * 2) = 0.0f;
            colors_.push_back(color);
            points_(0, i * 2 + 1) = xf2;
            points_(1, i * 2 + 1) = yf2;
            points_(2, i * 2 + 1) = 0.0f;
            colors_.push_back(color);
        }
    }
    pointsDuplicated_ = duplicatePoints;
    cellSize_ = coloredGrid.cellSize;
    limits_ = coloredGrid.limits;
}

LocalMap::ColoredGrid LocalMap::toColoredGrid() const
{
    UASSERT(limits_.valid());
    ColoredGrid coloredGrid;
    coloredGrid.cellSize = cellSize_;
    coloredGrid.limits = limits_;
    coloredGrid.grid = cv::Mat(limits_.height(), limits_.width(), CV_8S,
        ColoredGrid::unknownCellValue);
    coloredGrid.colors = cv::Mat(limits_.height(), limits_.width(), CV_32S,
        Color::missingColor.data());

    const int& minX = limits_.minX();
    const int& minY = limits_.minY();
    int step = pointsDuplicated_ ? 2 : 1;
    for (int i = 0; i < points_.cols(); i += step)
    {
        float xf = points_.coeff(0, i);
        float yf = points_.coeff(1, i);
        int y = std::floor(yf / cellSize_) - minY;
        int x = std::floor(xf / cellSize_) - minX;
        if (isObstacle(i))
        {
            coloredGrid.grid.at<std::int8_t>(y, x) =
                LocalMap::ColoredGrid::occupiedCellValue;
        }
        else
        {
            coloredGrid.grid.at<std::int8_t>(y, x) =
                LocalMap::ColoredGrid::emptyCellValue;
        }
        coloredGrid.colors.at<std::int32_t>(y, x) = colors_[i].data();
    }
    return coloredGrid;
}

proto::LocalMap::ColoredGrid toProto(const LocalMap::ColoredGrid& coloredGrid)
{
    proto::LocalMap::ColoredGrid proto;
    proto.set_cell_size(coloredGrid.cellSize);
    *proto.mutable_limits() = toProto(coloredGrid.limits);
    proto.set_grid_compressed(compressMat(coloredGrid.grid));
    proto.set_colors_compressed(compressMat(coloredGrid.colors));
    return proto;
}

LocalMap::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto)
{
    LocalMap::ColoredGrid coloredGrid;
    coloredGrid.cellSize = proto.cell_size();
    coloredGrid.limits = fromProto(proto.limits());
    coloredGrid.grid = decompressMat(proto.grid_compressed());
    coloredGrid.colors = decompressMat(proto.colors_compressed());
    return coloredGrid;
}

proto::LocalMap toProto(const LocalMap& localMap)
{
    proto::LocalMap proto;
    *proto.mutable_colored_grid() = toProto(localMap.toColoredGrid());
    proto.set_sensor_blind_range_2d_sqr(localMap.sensorBlindRange2dSqr());
    UASSERT(!localMap.toSensor().isNull());
    *proto.mutable_to_sensor() = toProto(localMap.toSensor());
    UASSERT(!localMap.fromUpdatedPose().isNull());
    *proto.mutable_from_updated_pose() = toProto(localMap.fromUpdatedPose());
    *proto.mutable_time() = toProto(localMap.time());
    proto.set_points_duplicated(localMap.pointsDuplicated());
    return proto;
}

std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto)
{
    auto localMap = std::make_shared<LocalMap>(fromProto(proto.colored_grid()),
        -1.0f, proto.points_duplicated());
    localMap->setSensorBlindRange2dSqr(proto.sensor_blind_range_2d_sqr());
    localMap->setToSensor(fromProto(proto.to_sensor()));
    localMap->setFromUpdatedPose(fromProto(proto.from_updated_pose()));
    localMap->setTime(fromProto(proto.time()));
    return localMap;
}

}
