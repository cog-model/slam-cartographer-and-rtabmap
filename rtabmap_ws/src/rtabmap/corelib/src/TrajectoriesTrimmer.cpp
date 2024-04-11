#include <rtabmap/core/TrajectoriesTrimmer.h>
#include <rtabmap/utilite/ULogger.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

float NodesSimilarityEstimation::getSimilarity(
    const Transform& oldPose, const LocalMap& oldLocalMap,
    const Transform& newPose, const LocalMap::ColoredGrid& newGrid)
{
    MEASURE_BLOCK_TIME(NodesSimilarityEstimation__getSimilarity);
    const Transform& relativePose = newPose.inverse() * oldPose;
    Eigen::Matrix3Xf oldPoints =
        (relativePose.toEigen3fRotation() * oldLocalMap.points()).colwise() +
        relativePose.toEigen3fTranslation();

    int hits = 0;
    for (int i = 0; i < oldPoints.cols(); i++)
    {
        float y = oldPoints.coeff(1, i);
        float x = oldPoints.coeff(0, i);
        int yi = std::floor(y / newGrid.cellSize) - newGrid.limits.minY();
        int xi = std::floor(x / newGrid.cellSize) - newGrid.limits.minX();
        if (yi < 0 || xi < 0 || yi >= newGrid.limits.height() || xi >= newGrid.limits.width())
        {
            continue;
        }
        if (newGrid.grid.at<std::int8_t>(yi, xi) == LocalMap::ColoredGrid::unknownCellValue)
        {
            continue;
        }
        hits++;
    }

    float similarity = 1.0f * hits / oldPoints.cols();
    return similarity;
}

TrajectoriesTrimmer::TrajectoriesTrimmer(const Parameters& parameters)
{
    parseParameters(parameters);
}

void TrajectoriesTrimmer::parseParameters(const Parameters& parameters)
{
    maxTimeErrorForClosestLocalMapSearch_ =
        parameters.maxTimeErrorForClosestLocalMapSearch;
    skipLastN_ = parameters.skipLastN;
    maxDistance_ = parameters.maxDistance;
    minSimilarity_ = parameters.minSimilarity;
    UASSERT(skipLastN_ >= 0);
    UASSERT(maxDistance_ >= 0.0f);
    UASSERT(minSimilarity_ >= 0.0f && minSimilarity_ <= 1.0f);

    maxDistanceSqr_ = maxDistance_ * maxDistance_;
}

void TrajectoriesTrimmer::addLocalMap(const std::shared_ptr<const LocalMap>& localMap)
{
    // UASSERT(localMaps_.empty() || (*localMaps_.rbegin())->time() < localMap->time());
    localMaps_.push_back(localMap);
}

const LocalMap* TrajectoriesTrimmer::findNextClosestLocalMap(
    std::vector<std::shared_ptr<const LocalMap>>::const_iterator& it,
    const Time& time, double maxError) const
{
    while (it != localMaps_.end() && (*it)->time() < time)
    {
        ++it;
    }

    const LocalMap* upper = nullptr;
    double upperDiff = std::numeric_limits<double>::max();
    const LocalMap* lower = nullptr;
    double lowerDiff = std::numeric_limits<double>::max();

    if (it != localMaps_.end())
    {
        upper = it->get();
        upperDiff = (*it)->time().toSec() - time.toSec();
        UASSERT(upperDiff >= 0.0);
    }
    if (it != localMaps_.begin())
    {
        auto prevIt = std::prev(it);
        lower = prevIt->get();
        lowerDiff = time.toSec() - (*prevIt)->time().toSec();
        UASSERT(lowerDiff > 0.0);
    }

    if (std::min(upperDiff, lowerDiff) <= maxError)
    {
        if (upperDiff < lowerDiff)
        {
            return upper;
        }
        else
        {
            return lower;
        }
    }
    return nullptr;
}

std::map<Time, const LocalMap*> TrajectoriesTrimmer::findClosestLocalMaps(
    const Trajectories& trajectories, double maxError) const
{
    std::map<Time, const LocalMap*> closestLocalMaps;
    auto localMapIt = localMaps_.begin();
    for (auto it = trajectories.poses_begin(); it != trajectories.poses_end(); ++it)
    {
        const LocalMap* closestLocalMap = findNextClosestLocalMap(
            localMapIt, it->time, maxError);
        closestLocalMaps.insert(closestLocalMaps.end(),
            std::make_pair(it->time, closestLocalMap));
    }
    return closestLocalMaps;
}

std::set<Time> TrajectoriesTrimmer::trimTrajectories(
    const Trajectories& trajectories)
{
    MEASURE_TIME_FROM_HERE(TrajectoriesTrimmer__trimTrajectories);

    auto trajectoryIt = trajectories.findCurrentOrNextTrajectory(prevLastTime_);
    if (trajectoryIt == trajectories.end())
    {
        return {};
    }
    Trajectory::const_iterator poseIt;
    if (trajectoryIt->containsTime(prevLastTime_))
    {
        poseIt = trajectoryIt->getBounds(prevLastTime_).second;
    }
    else
    {
        poseIt = trajectoryIt->begin();
    }

    std::map<Time, const LocalMap*> closestLocalMaps =
        findClosestLocalMaps(trajectories, maxTimeErrorForClosestLocalMapSearch_);
    auto newIt = trajectories.poses_iterator(trajectoryIt, poseIt);
    auto newLocalMapIt = closestLocalMaps.find(newIt->time);
    UASSERT(newLocalMapIt != closestLocalMaps.end());

    int skipN = skipLastN_;
    auto lastOldIt = newIt;
    auto lastOldLocalMapIt = newLocalMapIt;
    while (skipN > 0 && lastOldIt != trajectories.poses_begin())
    {
        --lastOldIt;
        UASSERT(lastOldLocalMapIt != closestLocalMaps.begin());
        --lastOldLocalMapIt;
        skipN--;
    }
    while (skipN > 0 && newIt != trajectories.poses_end())
    {
        ++newIt;
        UASSERT(newLocalMapIt != closestLocalMaps.end());
        ++newLocalMapIt;
        skipN--;
    }
    if (newIt != trajectories.poses_end() && lastOldIt == trajectories.poses_begin())
    {
        ++newIt;
        UASSERT(newLocalMapIt != closestLocalMaps.end());
        ++newLocalMapIt;
        ++lastOldIt;
        UASSERT(lastOldLocalMapIt != closestLocalMaps.end());
        ++lastOldLocalMapIt;
    }

    std::set<Time> posesToTrim;
    for (; newIt != trajectories.poses_end();
        ++newIt, ++newLocalMapIt, ++lastOldIt, ++lastOldLocalMapIt)
    {
        UASSERT(newLocalMapIt != closestLocalMaps.end());
        if (newLocalMapIt->second == nullptr)
        {
            continue;
        }
        const Transform& newPose = newIt->pose;
        const LocalMap::ColoredGrid& newGrid = newLocalMapIt->second->toColoredGrid();

        auto oldIt = trajectories.poses_begin();
        auto oldLocalMapIt = closestLocalMaps.begin();
        for (; oldIt != lastOldIt; ++oldIt, ++oldLocalMapIt)
        {
            UASSERT(oldLocalMapIt != lastOldLocalMapIt);
            if (oldLocalMapIt->second == nullptr)
            {
                continue;
            }
            if (posesToTrim.count(oldIt->time))
            {
                continue;
            }

            const Transform& oldPose = oldIt->pose;
            float distanceSqr = oldPose.getDistanceSquared(newPose);
            if (distanceSqr > maxDistanceSqr_)
            {
                continue;
            }
            float similarity = similarityEstimator_.getSimilarity(
                oldPose, *oldLocalMapIt->second,
                newPose, newGrid);
            if (similarity >= minSimilarity_)
            {
                posesToTrim.insert(oldIt->time);
                newLocalMapIt->second = nullptr;
            }
        }
        UASSERT(oldLocalMapIt == lastOldLocalMapIt);
    }
    UASSERT(newLocalMapIt == closestLocalMaps.end());

    prevLastTime_ = (--trajectories.poses_end())->time;

    STOP_TIME_MEASUREMENT(TrajectoriesTrimmer__trimTrajectories);
    return posesToTrim;
}

Trajectories TrajectoriesTrimmer::getTrimmedTrajectories(
    const Trajectories& trajectories, const std::set<Time>& posesToTrim)
{
    Trajectories trimmedTrajectories;
    auto poseToTrimIt = posesToTrim.begin();
    for (const Trajectory& trajectory : trajectories)
    {
        Trajectory trimmedTrajectory;
        for (const TimedPose& timedPose : trajectory)
        {
            while (poseToTrimIt != posesToTrim.end() && *poseToTrimIt < timedPose.time)
            {
                ++poseToTrimIt;
            }
            if (poseToTrimIt == posesToTrim.end() || *poseToTrimIt != timedPose.time)
            {
                trimmedTrajectory.addPose(timedPose.time, timedPose.pose);
            }
        }
        if (trimmedTrajectory.size())
        {
            trimmedTrajectories.addTrajectory(trimmedTrajectory);
        }
    }
    return trimmedTrajectories;
}

}