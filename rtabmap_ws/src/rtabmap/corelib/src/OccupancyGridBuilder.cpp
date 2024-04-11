#include <rtabmap/core/OccupancyGridBuilder.h>

#include <stdexcept>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

OccupancyGridBuilder::OccupancyGridBuilder(const Parameters& parameters)
{
    parseParameters(parameters);
}

void OccupancyGridBuilder::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    missProb_ = parameters.missProb;
    hitProb_ = parameters.hitProb;
    minClampingProb_ = parameters.minClampingProb;
    maxClampingProb_ = parameters.maxClampingProb;
    occupancyProbThr_ = parameters.occupancyProbThr;
    temporarilyOccupiedCellColorRgb_ = parameters.temporarilyOccupiedCellColorRgb;
    showTemporarilyOccupiedCells_ = parameters.showTemporarilyOccupiedCells;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(missProb_ > 0.0f && missProb_ <= 0.5f);
    UASSERT(hitProb_ >= 0.5f && hitProb_ < 1.0f);
    UASSERT(minClampingProb_ > 0.0f && minClampingProb_ < 1.0f);
    UASSERT(maxClampingProb_ > 0.0f && maxClampingProb_ < 1.0f);
    UASSERT(minClampingProb_ < maxClampingProb_);
    UASSERT(occupancyProbThr_ > 0.0f && occupancyProbThr_ < 1.0f);

    occupancyThr_ = probabilityToVlaue(occupancyProbThr_);
    if (temporarilyOccupiedCellColorRgb_ >= 0)
    {
        temporarilyOccupiedCellColor_.setRgb(temporarilyOccupiedCellColorRgb_);
    }
    precomputeUpdateValues();
}

void OccupancyGridBuilder::precomputeUpdateValues()
{
    float missLogit = logodds(missProb_);
    float hitLogit = logodds(hitProb_);
    float minClampingLogit = logodds(minClampingProb_);
    float maxClampingLogit = logodds(maxClampingProb_);

    updateValues_.missUpdates.clear();
    updateValues_.hitUpdates.clear();
    updateValues_.probabilities.clear();
    updateValues_.probabilitiesThr.clear();
    for (int value = 0; value <= PrecomputedUpdateValues::splitNum + 1; value++)
    {
        float prob = valueToProbability(value);
        float missUpdatedLogit, hitUpdatedLogit;
        if (value == 1)
        {
            // prob = 0.0
            missUpdatedLogit = minClampingLogit;
            hitUpdatedLogit = minClampingLogit;
        }
        else if (value == PrecomputedUpdateValues::splitNum + 1)
        {
            // prob = 1.0
            missUpdatedLogit = maxClampingLogit;
            hitUpdatedLogit = maxClampingLogit;
        }
        else
        {
            float logit = logodds(prob);
            missUpdatedLogit = logit + missLogit;
            hitUpdatedLogit = logit + hitLogit;
            missUpdatedLogit =
                std::clamp(missUpdatedLogit, minClampingLogit, maxClampingLogit);
            hitUpdatedLogit =
                std::clamp(hitUpdatedLogit, minClampingLogit, maxClampingLogit);
        }
        updateValues_.missUpdates.push_back(
            probabilityToVlaue(probability(missUpdatedLogit)));
        updateValues_.hitUpdates.push_back(
            probabilityToVlaue(probability(hitUpdatedLogit)));

        if (value == 0)
        {
            // unknown
            updateValues_.probabilitiesThr.push_back(-1);
            updateValues_.probabilities.push_back(-1);
        }
        else
        {
            updateValues_.probabilities.push_back(std::lround(prob * 100.0f));
            if (prob >= occupancyProbThr_)
            {
                updateValues_.probabilitiesThr.push_back(100);
            }
            else
            {
                updateValues_.probabilitiesThr.push_back(0);
            }
        }
    }
    for (int& updatedValue : updateValues_.missUpdates)
    {
        updatedValue += PrecomputedUpdateValues::updated;
    }
    for (int& updatedValue : updateValues_.hitUpdates)
    {
        updatedValue += PrecomputedUpdateValues::updated;
    }
}

int OccupancyGridBuilder::addLocalMap(
    const std::shared_ptr<const LocalMap>& localMap)
{
    int nodeId;
    if (map_.nodes.empty())
    {
        nodeId = 0;
    }
    else
    {
        nodeId = map_.nodes.rbegin()->first + 1;
    }
    map_.nodes.emplace(nodeId, Node(localMap));
    return nodeId;
}

int OccupancyGridBuilder::addLocalMap(const Transform& pose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    MEASURE_BLOCK_TIME(OccupancyGridBuilder__addLocalMap__withPose);
    int nodeId;
    if (map_.nodes.empty())
    {
        nodeId = 0;
    }
    else
    {
        nodeId = map_.nodes.rbegin()->first + 1;
    }
    Node node(localMap, pose, cellSize_);
    auto newNodeIt = map_.nodes.emplace(nodeId, std::move(node)).first;
    Node& newNode = newNodeIt->second;
    deployNode(newNode);
    newNode.removeTransformedLocalMap();
    return nodeId;
}

void OccupancyGridBuilder::removeNodes(const std::vector<int>& nodeIdsToRemove)
{
    for (int nodeIdToRemove : nodeIdsToRemove)
    {
        auto it = map_.nodes.find(nodeIdToRemove);
        UASSERT(it != map_.nodes.end());
        if (it == std::prev(map_.nodes.end()))
        {
            throw std::runtime_error("Cannot remove last node.");
        }
        map_.nodes.erase(it);
    }
}

void OccupancyGridBuilder::transformMap(const Transform& transform)
{
    MEASURE_BLOCK_TIME(OccupancyGridBuilder__transformMap);
    Transform transform3DoF = transform.to3DoF();
    Eigen::Matrix2f rotation2D =
        transform3DoF.toEigen3fRotation().block(0, 0, 2, 2);
    Eigen::Vector2f translation2D =
        transform3DoF.toEigen3fTranslation().block(0, 0, 2, 1);

    int numPoints = 0;
    for (int y = 0; y < map_.map.rows(); y++)
    {
        for (int x = 0; x < map_.map.cols(); x++)
        {
            std::int8_t cellState =
                updateValues_.probabilitiesThr[map_.map.coeff(y, x)];
            if (cellState == (std::int8_t)0)
            {
                // empty
                numPoints += 2;
            }
            else if (cellState == (std::int8_t)100)
            {
                // occupied
                numPoints++;
            }
        }
    }

    Eigen::Matrix2Xf points;
    std::vector<int> values;
    std::vector<int> colors;
    points.resize(2, numPoints);
    values.reserve(numPoints);
    colors.reserve(numPoints);
    int i = 0;
    for (int y = 0; y < map_.map.rows(); y++)
    {
        for (int x = 0; x < map_.map.cols(); x++)
        {
            int value = map_.map.coeff(y, x);
            std::int8_t cellState = updateValues_.probabilitiesThr[value];
            if (cellState == (std::int8_t)0)
            {
                // empty
                int color = map_.colors.coeff(y, x);
                points.coeffRef(0, i) = (x + map_.mapLimits.minX() + 0.25f) * cellSize_;
                points.coeffRef(1, i) = (y + map_.mapLimits.minY() + 0.25f) * cellSize_;
                values.push_back(value);
                colors.push_back(color);
                i++;
                points.coeffRef(0, i) = (x + map_.mapLimits.minX() + 0.75f) * cellSize_;
                points.coeffRef(1, i) = (y + map_.mapLimits.minY() + 0.75f) * cellSize_;
                values.push_back(value);
                colors.push_back(color);
                i++;
            }
            else if (cellState == (std::int8_t)100)
            {
                // occupied
                int color = map_.colors.coeff(y, x);
                points.coeffRef(0, i) = (x + map_.mapLimits.minX() + 0.5f) * cellSize_;
                points.coeffRef(1, i) = (y + map_.mapLimits.minY() + 0.5f) * cellSize_;
                values.push_back(value);
                colors.push_back(color);
                i++;
            }
        }
    }

    points = (rotation2D * points).colwise() + translation2D;
    for (auto& [nodeId, node] : map_.nodes)
    {
        if (node.hasPose())
        {
            const Transform& pose = node.pose();
            Transform newPose = transform3DoF * pose;
            node.setPose(std::move(newPose));
        }
    }

    MapLimitsF newMapLimitsF;
    for (int i = 0; i < numPoints; i++)
    {
        float x = points.coeff(0, i);
        float y = points.coeff(1, i);
        newMapLimitsF.update(x, y);
    }
    MapLimitsI newMapLimits(
        std::floor(newMapLimitsF.minX() / cellSize_),
        std::floor(newMapLimitsF.minY() / cellSize_),
        std::floor(newMapLimitsF.maxX() / cellSize_),
        std::floor(newMapLimitsF.maxY() / cellSize_));

    clear();
    map_.mapLimits = newMapLimits;
    int width = newMapLimits.width();
    int height = newMapLimits.height();
    map_.map = MapType::Constant(height, width, PrecomputedUpdateValues::unknown);
    map_.colors = ColorsType::Constant(height, width, Color::missingColor.data());
    for (int i = 0; i < numPoints; i++)
    {
        float xf = points.coeff(0, i);
        float yf = points.coeff(1, i);
        int value = values[i];
        int color = colors[i];

        int y = std::floor(yf / cellSize_) - newMapLimits.minY();
        int x = std::floor(xf / cellSize_) - newMapLimits.minX();
        if (value > map_.map.coeff(y, x))
        {
            map_.map.coeffRef(y, x) = value;
            if (color != Color::missingColor.data())
            {
                map_.colors.coeffRef(y, x) = color;
            }
        }
    }
}

void OccupancyGridBuilder::updateCachedMap()
{
    if(!map_.mapLimits.valid())
    {
        return;
    }

    auto nodeIt = map_.nodes.begin();
    if (cachedMap_.mapLimits.valid())
    {
        nodeIt = map_.nodes.upper_bound(cachedMap_.poses.rbegin()->first);
    }
    while (nodeIt != map_.nodes.end())
    {
        int nodeId = nodeIt->first;
        const Node& node = nodeIt->second;
        if (node.hasPose())
        {
            cachedMap_.poses.emplace(nodeId, node.pose());
        }
        ++nodeIt;
    }
    cachedMap_.mapLimits = map_.mapLimits;
    cachedMap_.map = map_.map;
    cachedMap_.colors = map_.colors;
    cachedMap_.temporarilyOccupiedCells = map_.temporarilyOccupiedCells;
}

bool OccupancyGridBuilder::cachedMapCanBeUsed(
    const std::map<int, Transform>& newPoses)
{
    if (!cachedMap_.mapLimits.valid())
    {
        return false;
    }
    auto newPoseIt = newPoses.begin();
    auto cachedPoseIt = cachedMap_.poses.begin();
    while (cachedPoseIt != cachedMap_.poses.end())
    {
        if (newPoseIt == newPoses.end())
        {
            return false;
        }
        if (*cachedPoseIt != *newPoseIt)
        {
            return false;
        }
        ++newPoseIt;
        ++cachedPoseIt;
    }
    return true;
}

void OccupancyGridBuilder::useCachedMap()
{
    clear();
    auto cachedPoseIt = cachedMap_.poses.begin();
    auto nodeIt = map_.nodes.begin();
    while (cachedPoseIt != cachedMap_.poses.end())
    {
        UASSERT(nodeIt != map_.nodes.end());
        while (cachedPoseIt->first != nodeIt->first)
        {
            ++nodeIt;
            UASSERT(nodeIt != map_.nodes.end());
        }
        const Transform& cachedPose = cachedPoseIt->second;
        Node& node = nodeIt->second;
        node.setPose(cachedPose);
        ++cachedPoseIt;
        ++nodeIt;
    }
    map_.mapLimits = cachedMap_.mapLimits;
    map_.map = cachedMap_.map;
    map_.colors = cachedMap_.colors;
    map_.temporarilyOccupiedCells = cachedMap_.temporarilyOccupiedCells;
}

bool OccupancyGridBuilder::tryToUseCachedMap(const std::map<int, Transform>& newPoses)
{
    MEASURE_TIME_FROM_HERE(OccupancyGridBuilder__tryToUseCachedMap__fail);
    MEASURE_TIME_FROM_HERE(OccupancyGridBuilder__tryToUseCachedMap__success);

    if (!cachedMapCanBeUsed(newPoses))
    {
        STOP_TIME_MEASUREMENT(OccupancyGridBuilder__tryToUseCachedMap__fail);
        return false;
    }

    useCachedMap();
    STOP_TIME_MEASUREMENT(OccupancyGridBuilder__tryToUseCachedMap__success);
    return true;
}

void OccupancyGridBuilder::updatePoses(
    const std::map<int, Transform>& updatedPoses,
    int lastNodeIdToIncludeInCachedMap /* -1 */)
{
    MEASURE_BLOCK_TIME(OccupancyGridBuilder__updatePoses);
    std::map<int, Transform> newPoses;
    {
        auto nodeIt = map_.nodes.begin();
        auto updatedPoseIt = updatedPoses.begin();
        while (updatedPoseIt != updatedPoses.end())
        {
            UASSERT(nodeIt != map_.nodes.end());
            while (nodeIt->first != updatedPoseIt->first)
            {
                ++nodeIt;
                UASSERT(nodeIt != map_.nodes.end());
            }
            int nodeId = nodeIt->first;
            const Transform& fromUpdatedPose = nodeIt->second.localMap()->fromUpdatedPose();
            const Transform& updatedPose = updatedPoseIt->second;
            newPoses[nodeId] = updatedPose * fromUpdatedPose;
            ++nodeIt;
            ++updatedPoseIt;
        }
    }

    clear();
    bool usedCachedMap = tryToUseCachedMap(newPoses);

    int lastNodeIdFromCache = -1;
    if (usedCachedMap)
    {
        lastNodeIdFromCache = cachedMap_.poses.rbegin()->first;
    }
    std::vector<std::reference_wrapper<Node>> nodesToDeploy;
    auto nodeIt = map_.nodes.upper_bound(lastNodeIdFromCache);
    auto newPoseIt = newPoses.upper_bound(lastNodeIdFromCache);
    while (newPoseIt != newPoses.end())
    {
        UASSERT(nodeIt != map_.nodes.end());
        while (nodeIt->first != newPoseIt->first)
        {
            ++nodeIt;
            UASSERT(nodeIt != map_.nodes.end());
        }
        int nodeId = nodeIt->first;
        Node& node = nodeIt->second;
        node.transformLocalMap(newPoseIt->second, cellSize_);
        nodesToDeploy.emplace_back(std::ref(node));

        if (nodeId == lastNodeIdToIncludeInCachedMap)
        {
            deployNodes(nodesToDeploy);
            for (Node& node : nodesToDeploy)
            {
                node.removeTransformedLocalMap();
            }
            if (!usedCachedMap)
            {
                clearCachedMap();
            }
            updateCachedMap();
            nodesToDeploy.clear();
        }

        ++newPoseIt;
        ++nodeIt;
    }
    if (nodesToDeploy.size())
    {
        deployNodes(nodesToDeploy);
        for (Node& node : nodesToDeploy)
        {
            node.removeTransformedLocalMap();
        }
    }
}

void OccupancyGridBuilder::createOrResizeMap(const MapLimitsI& newMapLimits)
{
    UASSERT(newMapLimits.valid());
    if(!map_.mapLimits.valid())
    {
        map_.mapLimits = newMapLimits;
        int height = newMapLimits.height();
        int width = newMapLimits.width();
        map_.map = MapType::Constant(height, width, PrecomputedUpdateValues::unknown);
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
        MapType newMap = MapType::Constant(height, width, PrecomputedUpdateValues::unknown);
        ColorsType newColors =
            ColorsType::Constant(height, width, Color::missingColor.data());

        newMap.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            map_.map.block(srcStartY, srcStartX, copyHeight, copyWidth);
        newColors.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            map_.colors.block(srcStartY, srcStartX, copyHeight, copyWidth);

        map_.mapLimits = newMapLimits;
        map_.map = std::move(newMap);
        map_.colors = std::move(newColors);
    }
}

void OccupancyGridBuilder::deployNode(const Node& node)
{
    UASSERT(node.hasTransformedLocalMap());
    MapLimitsI newMapLimits = MapLimitsI::unite(
        map_.mapLimits,
        node.transformedLocalMap().mapLimits());
    if (map_.mapLimits != newMapLimits)
    {
        createOrResizeMap(newMapLimits);
    }
    deployTransformedLocalMap(*node.localMap(), node.transformedLocalMap());
}

void OccupancyGridBuilder::deployNodes(
    const std::vector<std::reference_wrapper<Node>>& nodes)
{
    MapLimitsI newMapLimits = map_.mapLimits;
    for (const Node& node : nodes)
    {
        UASSERT(node.hasTransformedLocalMap());
        newMapLimits = MapLimitsI::unite(
            newMapLimits,
            node.transformedLocalMap().mapLimits());
    }
    if (map_.mapLimits != newMapLimits)
    {
        createOrResizeMap(newMapLimits);
    }
    for (const Node& node : nodes)
    {
        deployTransformedLocalMap(*node.localMap(), node.transformedLocalMap());
    }
}

void OccupancyGridBuilder::deployTransformedLocalMap(const LocalMap& localMap,
    const TransformedLocalMap& transformedLocalMap)
{
    map_.temporarilyOccupiedCells.clear();
    const Eigen::Matrix2Xi& transformedPoints = transformedLocalMap.points();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int y = transformedPoints.coeff(1, i) - map_.mapLimits.minY();
        int x = transformedPoints.coeff(0, i) - map_.mapLimits.minX();
        UASSERT(y >= 0 && x >= 0 && y < map_.map.rows() && x < map_.map.cols());

        int& value = map_.map.coeffRef(y, x);
        if (value >= PrecomputedUpdateValues::updated)
        {
            continue;
        }
        bool occupied = localMap.isObstacle(i);
        if (occupied)
        {
            if (temporarilyOccupiedCellColor_ != Color::missingColor)
            {
                const Color& color = localMap.colors()[i];
                if (color == temporarilyOccupiedCellColor_)
                {
                    map_.temporarilyOccupiedCells.emplace_back(y, x);
                    continue;
                }
            }
            value = updateValues_.hitUpdates[value];
        }
        else
        {
            if (localMap.sensorBlindRange2dSqr() > 0.0f &&
                value >= occupancyThr_)
            {
                float localX = localMap.points().coeff(0, i);
                float localY = localMap.points().coeff(1, i);
                float sensorX = localX - localMap.toSensor().translation().x();
                float sensorY = localY - localMap.toSensor().translation().y();
                if (sensorX * sensorX + sensorY * sensorY <=
                    localMap.sensorBlindRange2dSqr())
                {
                    continue;
                }
            }
            value = updateValues_.missUpdates[value];
        }

        const Color& color = localMap.colors()[i];
        if (!color.missing())
        {
            map_.colors.coeffRef(y, x) = color.rgb();
        }
    }
    for (int y = 0; y < map_.map.rows(); y++)
    {
        for (int x = 0; x < map_.map.cols(); x++)
        {
            if (map_.map.coeffRef(y, x) >= PrecomputedUpdateValues::updated)
            {
                map_.map.coeffRef(y, x) -= PrecomputedUpdateValues::updated;
            }
        }
    }
}

OccupancyGrid OccupancyGridBuilder::getOccupancyGrid(
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
            int value = map_.map.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                updateValues_.probabilitiesThr[value];
        }
    }
    if (showTemporarilyOccupiedCells_)
    {
        for (const auto& coords : map_.temporarilyOccupiedCells)
        {
            int y = coords.first;
            int x = coords.second;
            y -= srcStartY;
            x -= srcStartX;
            if (y >= 0 && x >= 0 && y < height && x < width)
            {
                occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
            }
        }
    }
    return occupancyGrid;
}

OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid(
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
            int value = map_.map.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                updateValues_.probabilities[value];
        }
    }
    if (showTemporarilyOccupiedCells_)
    {
        for (const auto& coords : map_.temporarilyOccupiedCells)
        {
            int y = coords.first;
            int x = coords.second;
            y -= srcStartY;
            x -= srcStartX;
            if (y >= 0 && x >= 0 && y < height && x < width)
            {
                occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
            }
        }
    }
    return occupancyGrid;
}

ColorGrid OccupancyGridBuilder::getColorGrid(
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
    if (showTemporarilyOccupiedCells_)
    {
        for (const auto& coords : map_.temporarilyOccupiedCells)
        {
            int y = coords.first;
            int x = coords.second;
            y -= srcStartY;
            x -= srcStartX;
            if (y >= 0 && x >= 0 && y < height && x < width)
            {
                colorGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                    temporarilyOccupiedCellColor_.rgb();
            }
        }
    }
    return colorGrid;
}

void OccupancyGridBuilder::clear()
{
    for (auto& [nodeId, node] : map_.nodes)
    {
        node.removePose();
    }
    map_.mapLimits = MapLimitsI();
    map_.map = MapType();
    map_.colors = ColorsType();
    map_.temporarilyOccupiedCells.clear();
}

void OccupancyGridBuilder::clearCachedMap()
{
    cachedMap_.poses.clear();
    cachedMap_.mapLimits = MapLimitsI();
    cachedMap_.map = MapType();
    cachedMap_.colors = ColorsType();
    cachedMap_.temporarilyOccupiedCells.clear();
}

void OccupancyGridBuilder::reset()
{
    map_.nodes.clear();
    clear();
    clearCachedMap();
}

}
