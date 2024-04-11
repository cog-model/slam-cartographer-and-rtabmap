#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

TemporaryOccupancyGridBuilder::TemporaryOccupancyGridBuilder(const Parameters& parameters)
{
    parseParameters(parameters);
}

void TemporaryOccupancyGridBuilder::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    temporaryMissProb_ = parameters.temporaryMissProb;
    temporaryHitProb_ = parameters.temporaryHitProb;
    temporaryOccupancyProbThr_ = parameters.temporaryOccupancyProbThr;
    maxTemporaryLocalMaps_ = parameters.maxTemporaryLocalMaps;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(temporaryMissProb_ > 0.0f && temporaryMissProb_ <= 0.5f);
    UASSERT(temporaryHitProb_ >= 0.5f && temporaryHitProb_ < 1.0f);
    UASSERT(temporaryOccupancyProbThr_ > 0.0f && temporaryOccupancyProbThr_ < 1.0f);
    UASSERT(maxTemporaryLocalMaps_ >= 1);

    updated_ = maxTemporaryLocalMaps_ * 2 + 1;
    precomputeProbabilities();
}

void TemporaryOccupancyGridBuilder::precomputeProbabilities()
{
    float missLogit = logodds(temporaryMissProb_);
    float hitLogit = logodds(temporaryHitProb_);
    precomputedProbabilities_.probabilities.resize(maxTemporaryLocalMaps_ + 1, maxTemporaryLocalMaps_ + 1);
    precomputedProbabilities_.probabilitiesThr.resize(maxTemporaryLocalMaps_ + 1, maxTemporaryLocalMaps_ + 1);
    for (int hits = 0; hits <= maxTemporaryLocalMaps_; hits++)
    {
        for (int misses = 0; misses <= maxTemporaryLocalMaps_; misses++)
        {
            float prob = probability(hits * hitLogit + misses * missLogit);
            precomputedProbabilities_.probabilities.coeffRef(hits, misses) = std::lround(prob * 100.0f);
            if (prob >= temporaryOccupancyProbThr_)
            {
                precomputedProbabilities_.probabilitiesThr.coeffRef(hits, misses) = 100;
            }
            else
            {
                precomputedProbabilities_.probabilitiesThr.coeffRef(hits, misses) = 0;
            }
        }
    }
    precomputedProbabilities_.probabilities.coeffRef(0, 0) = -1;
    precomputedProbabilities_.probabilitiesThr.coeffRef(0, 0) = -1;
}

bool TemporaryOccupancyGridBuilder::addLocalMap(
    const Transform& pose, const std::shared_ptr<const LocalMap>& localMap)
{
    MEASURE_BLOCK_TIME(TemporaryOccupancyGridBuilder__addLocalMap);
    UASSERT(localMap);
    Node node(localMap, pose, cellSize_);
    map_.nodes.emplace_back(std::move(node));
    deployLastNode();
    bool overflowed = false;
    if (map_.nodes.size() > maxTemporaryLocalMaps_)
    {
        overflowed = true;
        removeFirstNode();
    }
    return overflowed;
}

void TemporaryOccupancyGridBuilder::transformMap(const Transform& transform)
{
    MEASURE_BLOCK_TIME(OccupancyGridBuilder__transformMap);
    Transform transform3DoF = transform.to3DoF();

    std::deque<Transform> newPoses;
    for (Node& node : map_.nodes)
    {
        const Transform& pose = node.pose();
        Transform newPose = transform3DoF * pose;
        newPoses.emplace_back(std::move(newPose));
    }

    clear();

    auto nodeIt = map_.nodes.begin();
    auto newPoseIt = newPoses.begin();
    while (newPoseIt != newPoses.end())
    {
        Node& node = *nodeIt;
        const Transform& newPose = *newPoseIt;
        node.transformLocalMap(newPose, cellSize_);
        ++nodeIt;
        ++newPoseIt;
    }
    deployAllNodes();
}

void TemporaryOccupancyGridBuilder::updatePoses(
    const std::deque<Transform>& updatedPoses)
{
    MEASURE_BLOCK_TIME(TemporaryOccupancyGridBuilder__updatePoses);
    UASSERT(map_.nodes.size() == updatedPoses.size());
    std::deque<Transform> newPoses;
    {
        auto nodeIt = map_.nodes.begin();
        auto updatedPoseIt = updatedPoses.begin();
        while (updatedPoseIt != updatedPoses.end())
        {
            const Transform& fromUpdatedPose = nodeIt->localMap()->fromUpdatedPose();
            const Transform& updatedPose = *updatedPoseIt;
            newPoses.emplace_back(updatedPose * fromUpdatedPose);
            ++nodeIt;
            ++updatedPoseIt;
        }
    }

    clear();

    auto nodeIt = map_.nodes.begin();
    auto newPoseIt = newPoses.begin();
    while (newPoseIt != newPoses.end())
    {
        Node& node = *nodeIt;
        const Transform& newPose = *newPoseIt;
        node.transformLocalMap(newPose, cellSize_);
        ++nodeIt;
        ++newPoseIt;
    }
    deployAllNodes();
}

void TemporaryOccupancyGridBuilder::createOrResizeMap(const MapLimitsI& newMapLimits)
{
    UASSERT(newMapLimits.valid());
    if(!map_.mapLimits.valid())
    {
        map_.mapLimits = newMapLimits;
        int height = newMapLimits.height();
        int width = newMapLimits.width();
        map_.hitCounter = CounterType::Constant(height, width, 0);
        map_.missCounter = CounterType::Constant(height, width, 0);
        map_.colors = ColorsType::Constant(height, width, Color::missingColor.data());
    }
    else if(map_.mapLimits != newMapLimits)
    {
        int dstStartY = std::max(map_.mapLimits.minY() - newMapLimits.minY(), 0);
        int dstStartX = std::max(map_.mapLimits.minX() - newMapLimits.minX(), 0);
        int srcStartY = std::max(newMapLimits.minY() - map_.mapLimits.minY(), 0);
        int srcStartX = std::max(newMapLimits.minX() - map_.mapLimits.minX(), 0);
        MapLimitsI intersection = MapLimitsI::intersect(map_.mapLimits, newMapLimits);
        int copyHeight = intersection.height();
        int copyWidth = intersection.width();
        UASSERT(copyHeight > 0 && copyWidth > 0);

        int height = newMapLimits.height();
        int width = newMapLimits.width();
        CounterType newHitCounter = CounterType::Constant(height, width, 0);
        CounterType newMissCounter = CounterType::Constant(height, width, 0);
        ColorsType newColors =
            ColorsType::Constant(height, width, Color::missingColor.data());

        newHitCounter.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            map_.hitCounter.block(srcStartY, srcStartX, copyHeight, copyWidth);
        newMissCounter.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            map_.missCounter.block(srcStartY, srcStartX, copyHeight, copyWidth);
        newColors.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            map_.colors.block(srcStartY, srcStartX, copyHeight, copyWidth);

        map_.mapLimits = newMapLimits;
        map_.hitCounter = std::move(newHitCounter);
        map_.missCounter = std::move(newMissCounter);
        map_.colors = std::move(newColors);
    }
}

void TemporaryOccupancyGridBuilder::deployLastNode()
{
    UASSERT(map_.nodes.size());
    const Node& lastNode = map_.nodes.back();
    MapLimitsI newMapLimits = MapLimitsI::unite(
        map_.mapLimits,
        lastNode.transformedLocalMap().mapLimits());
    if (map_.mapLimits != newMapLimits)
    {
        createOrResizeMap(newMapLimits);
    }
    deployTransformedLocalMap(*lastNode.localMap(), lastNode.transformedLocalMap());
}

void TemporaryOccupancyGridBuilder::deployAllNodes()
{
    MapLimitsI newMapLimits = map_.mapLimits;
    for (const Node& node : map_.nodes)
    {
        newMapLimits = MapLimitsI::unite(
            newMapLimits,
            node.transformedLocalMap().mapLimits());
    }
    if (map_.mapLimits != newMapLimits)
    {
        createOrResizeMap(newMapLimits);
    }
    for (const Node& node : map_.nodes)
    {
        deployTransformedLocalMap(*node.localMap(), node.transformedLocalMap());
    }
}

void TemporaryOccupancyGridBuilder::removeFirstNode()
{
    UASSERT(map_.nodes.size());
    const Node& firstNode = map_.nodes.front();
    removeTransformedLocalMap(*firstNode.localMap(), firstNode.transformedLocalMap());
    map_.nodes.pop_front();
    MapLimitsI newMapLimits;
    for (const Node& node : map_.nodes)
    {
        newMapLimits = MapLimitsI::unite(
            newMapLimits,
            node.transformedLocalMap().mapLimits());
    }
    createOrResizeMap(newMapLimits);
}

void TemporaryOccupancyGridBuilder::deployTransformedLocalMap(
    const LocalMap& localMap, const TransformedLocalMap& transformedLocalMap)
{
    const Eigen::Matrix2Xi& transformedPoints = transformedLocalMap.points();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int y = transformedPoints.coeff(1, i) - map_.mapLimits.minY();
        int x = transformedPoints.coeff(0, i) - map_.mapLimits.minX();
        UASSERT(y >= 0 && x >= 0 && y < map_.missCounter.rows() && x < map_.missCounter.cols());

        if (map_.hitCounter.coeffRef(y, x) >= updated_)
        {
            continue;
        }
        bool occupied = localMap.isObstacle(i);
        if (occupied)
        {
            map_.hitCounter.coeffRef(y, x) += 1;
        }
        else
        {
            map_.missCounter.coeffRef(y, x) += 1;
        }
        map_.hitCounter.coeffRef(y, x) += updated_;

        const Color& color = localMap.colors()[i];
        map_.colors.coeffRef(y, x) = color.data();
    }
    for (int y = 0; y < map_.hitCounter.rows(); y++)
    {
        for (int x = 0; x < map_.hitCounter.cols(); x++)
        {
            if (map_.hitCounter.coeffRef(y, x) >= updated_)
            {
                map_.hitCounter.coeffRef(y, x) -= updated_;
            }
        }
    }
}

void TemporaryOccupancyGridBuilder::removeTransformedLocalMap(
    const LocalMap& localMap, const TransformedLocalMap& transformedLocalMap)
{
    const Eigen::Matrix2Xi& transformedPoints = transformedLocalMap.points();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int y = transformedPoints.coeff(1, i) - map_.mapLimits.minY();
        int x = transformedPoints.coeff(0, i) - map_.mapLimits.minX();
        UASSERT(y >= 0 && x >= 0 && y < map_.missCounter.rows() && x < map_.missCounter.cols());

        if (map_.hitCounter.coeffRef(y, x) >= updated_)
        {
            continue;
        }
        bool occupied = localMap.isObstacle(i);
        if (occupied)
        {
            map_.hitCounter.coeffRef(y, x) -= 1;
        }
        else
        {
            map_.missCounter.coeffRef(y, x) -= 1;
        }
        map_.hitCounter.coeffRef(y, x) += updated_;
    }
    for (int y = 0; y < map_.hitCounter.rows(); y++)
    {
        for (int x = 0; x < map_.hitCounter.cols(); x++)
        {
            if (map_.hitCounter.coeffRef(y, x) >= updated_)
            {
                map_.hitCounter.coeffRef(y, x) -= updated_;
            }
            UASSERT(map_.hitCounter.coeffRef(y, x) >= 0);
            UASSERT(map_.missCounter.coeffRef(y, x) >= 0);
        }
    }
}

OccupancyGrid TemporaryOccupancyGridBuilder::getOccupancyGrid(
    MapLimitsI roi /* MapLimitsI() */) const
{
    if (!map_.mapLimits.valid())
    {
        return OccupancyGrid();
    }
    if (!roi.valid())
    {
        roi = map_.mapLimits;
    }
    OccupancyGrid occupancyGrid;
    occupancyGrid.limits = roi;
    occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
    MapLimitsI intersection = MapLimitsI::intersect(map_.mapLimits, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return occupancyGrid;
    }
    int dstStartY = std::max(map_.mapLimits.minY() - roi.minY(), 0);
    int dstStartX = std::max(map_.mapLimits.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - map_.mapLimits.minY(), 0);
    int srcStartX = std::max(roi.minX() - map_.mapLimits.minX(), 0);
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            int hits = map_.hitCounter.coeff(y + srcStartY, x + srcStartX);
            int misses = map_.missCounter.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                precomputedProbabilities_.probabilitiesThr.coeff(hits, misses);
        }
    }
    return occupancyGrid;
}

OccupancyGrid TemporaryOccupancyGridBuilder::getProbOccupancyGrid(
    MapLimitsI roi /* MapLimitsI() */) const
{
    if (!map_.mapLimits.valid())
    {
        return OccupancyGrid();
    }
    if (!roi.valid())
    {
        roi = map_.mapLimits;
    }
    OccupancyGrid occupancyGrid;
    occupancyGrid.limits = roi;
    occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
    MapLimitsI intersection = MapLimitsI::intersect(map_.mapLimits, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return occupancyGrid;
    }
    int dstStartY = std::max(map_.mapLimits.minY() - roi.minY(), 0);
    int dstStartX = std::max(map_.mapLimits.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - map_.mapLimits.minY(), 0);
    int srcStartX = std::max(roi.minX() - map_.mapLimits.minX(), 0);
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            int hits = map_.hitCounter.coeff(y + srcStartY, x + srcStartX);
            int misses = map_.missCounter.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                precomputedProbabilities_.probabilities.coeff(hits, misses);
        }
    }
    return occupancyGrid;
}

ColorGrid TemporaryOccupancyGridBuilder::getColorGrid(
    MapLimitsI roi /* MapLimitsI() */) const
{
    if (!map_.mapLimits.valid())
    {
        return ColorGrid();
    }
    if (!roi.valid())
    {
        roi = map_.mapLimits;
    }
    ColorGrid colorGrid;
    colorGrid.limits = roi;
    colorGrid.grid = ColorGrid::GridType::Constant(roi.height(), roi.width(),
        Color::missingColor.data());
    MapLimitsI intersection = MapLimitsI::intersect(map_.mapLimits, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return colorGrid;
    }
    int dstStartY = std::max(map_.mapLimits.minY() - roi.minY(), 0);
    int dstStartX = std::max(map_.mapLimits.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - map_.mapLimits.minY(), 0);
    int srcStartX = std::max(roi.minX() - map_.mapLimits.minX(), 0);
    colorGrid.grid.block(dstStartY, dstStartX, height, width) =
        map_.colors.block(srcStartY, srcStartX, height, width);
    return colorGrid;
}

void TemporaryOccupancyGridBuilder::clear()
{
    for (Node& node : map_.nodes)
    {
        node.removePose();
    }
    map_.mapLimits = MapLimitsI();
    map_.hitCounter = CounterType();
    map_.missCounter = CounterType();
    map_.colors = ColorsType();
}

void TemporaryOccupancyGridBuilder::reset()
{
    map_.nodes.clear();
    map_.mapLimits = MapLimitsI();
    map_.hitCounter = CounterType();
    map_.missCounter = CounterType();
    map_.colors = ColorsType();
}

}
