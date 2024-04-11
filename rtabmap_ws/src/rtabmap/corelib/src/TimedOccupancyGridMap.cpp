#include <rtabmap/core/TimedOccupancyGridMap.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap/proto/OccupancyGridMap.pb.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

TimedOccupancyGridMap::TimedOccupancyGridMap(const Parameters& parameters)
{
    parseParameters(parameters);
}

void TimedOccupancyGridMap::parseParameters(const Parameters& parameters)
{
    maxInterpolationTimeError_ = parameters.maxInterpolationTimeError;
    guaranteedInterpolationTimeWindow_ = parameters.guaranteedInterpolationTimeWindow;
    enableTrajectoriesTrimmer_ = parameters.enableTrajectoriesTrimmer;

    occupancyGridMap_ = std::make_unique<OccupancyGridMap>(
        parameters.occupancyGridMapParameters);
    if (enableTrajectoriesTrimmer_)
    {
        trajectoriesTrimmer_ = std::make_unique<TrajectoriesTrimmer>(
            parameters.trajectoriesTrimmerParameters);
    }
    else
    {
        trajectoriesTrimmer_.reset();
    }
}

std::shared_ptr<LocalMap> TimedOccupancyGridMap::createLocalMap(
    const SensorData& sensorData,
    const Time& time, const Transform& fromUpdatedPose) const
{
    return occupancyGridMap_->createLocalMap(sensorData, time, fromUpdatedPose);
}

int TimedOccupancyGridMap::addLocalMap(const std::shared_ptr<const LocalMap>& localMap)
{
    int nodeId = occupancyGridMap_->addLocalMap(localMap);
    return nodeId;
}

int TimedOccupancyGridMap::addLocalMap(const Transform& pose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    // UASSERT(localMap->time() > lastPoseTime_);
    int nodeId = occupancyGridMap_->addLocalMap(pose, localMap);

    const Transform& toUpdatedPose = localMap->fromUpdatedPose().inverse();
    currentTrajectory_.addPose(localMap->time(), pose * toUpdatedPose);
    lastPoseTime_ = localMap->time();

    if (trajectoriesTrimmer_)
    {
        trajectoriesTrimmer_->addLocalMap(
            occupancyGridMap_->localMapsWithoutDilation().rbegin()->second);
    }

    return nodeId;
}

bool TimedOccupancyGridMap::addTemporaryLocalMap(const Transform& pose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    UASSERT(localMap->time() > lastTemporaryPoseTime_);
    bool overflowed = occupancyGridMap_->addTemporaryLocalMap(pose, localMap);

    const Transform& toUpdatedPose = localMap->fromUpdatedPose().inverse();
    currentTrajectory_.addPose(localMap->time(), pose * toUpdatedPose);
    lastTemporaryPoseTime_ = localMap->time();

    return overflowed;
}

int TimedOccupancyGridMap::addSensorData(const SensorData& sensorData,
    const Time& time, const Transform& fromUpdatedPose)
{
    std::shared_ptr<LocalMap> localMap =
        createLocalMap(sensorData, time, fromUpdatedPose);
    int nodeId = addLocalMap(localMap);
    return nodeId;
}

int TimedOccupancyGridMap::addSensorData(const SensorData& sensorData,
    const Time& time, const Transform& pose,
    const Transform& fromUpdatedPose)
{
    std::shared_ptr<LocalMap> localMap =
        createLocalMap(sensorData, time, fromUpdatedPose);
    int nodeId = addLocalMap(pose, localMap);
    return nodeId;
}

bool TimedOccupancyGridMap::addTemporarySensorData(const SensorData& sensorData,
    const Time& time, const Transform& pose,
    const Transform& fromUpdatedPose)
{
    std::shared_ptr<LocalMap> localMap =
        createLocalMap(sensorData, time, fromUpdatedPose);
    bool overflowed = addTemporaryLocalMap(pose, localMap);
    return overflowed;
}

void TimedOccupancyGridMap::transformMap(const Transform& transform)
{
    Transform transform3DoF = transform.to3DoF();
    Trajectory transformedCurrentTrajectory;
    for (const auto& timedPose : currentTrajectory_)
    {
        const Transform& pose = timedPose.pose;
        Transform transformedPose = transform3DoF * pose;
        transformedCurrentTrajectory.addPose(timedPose.time, transformedPose);
    }
    currentTrajectory_ = std::move(transformedCurrentTrajectory);

    occupancyGridMap_->transformMap(transform);
}

void TimedOccupancyGridMap::updatePoses(const Trajectories& trajectories)
{
    const Trajectory* activeTrajectoryPtr = nullptr;
    if (currentTrajectory_.size())
    {
        auto activeTrajectoryIt = trajectories.findCurrentOrPreviousTrajectory(
            currentTrajectory_.maxTime());
        if (activeTrajectoryIt != trajectories.end())
        {
            activeTrajectoryPtr = &(*activeTrajectoryIt);
        }
    }
    std::optional<Transform> extrapolationShift = std::nullopt;
    if (activeTrajectoryPtr)
    {
        Time commonTime = std::min(
            currentTrajectory_.maxTime(), activeTrajectoryPtr->maxTime());
        std::optional<Transform> currentPose = currentTrajectory_.interpolate(commonTime);
        std::optional<Transform> updatedPose = activeTrajectoryPtr->interpolate(commonTime);
        if (currentPose.has_value() && updatedPose.has_value())
        {
            extrapolationShift = (*updatedPose) * currentPose->inverse();
        }
    }

    if (!extrapolationShift.has_value())
    {
        currentTrajectory_.clear();
        occupancyGridMap_->resetTemporary();
    }

    Time minUpdatedCurrentTrajectoryTime = Time::max();
    if (currentTrajectory_.size())
    {
        minUpdatedCurrentTrajectoryTime =
            Time(currentTrajectory_.maxTime().toSec() - guaranteedInterpolationTimeWindow_);
        if (activeTrajectoryPtr)
        {
            minUpdatedCurrentTrajectoryTime = std::min(
                minUpdatedCurrentTrajectoryTime,
                activeTrajectoryPtr->maxTime());
        }
        const auto& temporaryNodesRef = temporaryNodes(0);
        if (temporaryNodesRef.size())
        {
            minUpdatedCurrentTrajectoryTime = std::min(
                minUpdatedCurrentTrajectoryTime,
                temporaryNodes(0).begin()->localMap()->time());
        }
    }
    currentTrajectory_.trim(minUpdatedCurrentTrajectoryTime);

    Trajectory updatedCurrentTrajectory;
    int lastNodeIdToIncludeInCachedMap = -1;
    std::map<int, Transform> updatedPoses;
    {
        const auto& nodesRef = nodes(0);
        for (const auto& [nodeId, node] : nodesRef)
        {
            const Time& time = node.localMap()->time();
            std::optional<Transform> pose = getPose(
                trajectories, time, activeTrajectoryPtr, extrapolationShift);
            if (pose.has_value())
            {
                auto nodeTrajectoryIt = trajectories.findCurrentTrajectory(time);
                if (activeTrajectoryPtr && nodeTrajectoryIt != trajectories.end() &&
                    *nodeTrajectoryIt < *activeTrajectoryPtr)
                {
                    lastNodeIdToIncludeInCachedMap = nodeId;
                }
                if (currentTrajectory_.size() &&
                    currentTrajectory_.containsTime(time))
                {
                    updatedCurrentTrajectory.addPose(time, *pose);
                }
                updatedPoses[nodeId] = *pose;
            }
        }
    }

    std::deque<Transform> updatedTemporaryPoses;
    {
        const auto& nodesRef = temporaryNodes(0);
        for (const Node& node : nodesRef)
        {
            const Time& time = node.localMap()->time();
            std::optional<Transform> pose = getPose(
                trajectories, time, activeTrajectoryPtr, extrapolationShift);
            UASSERT(pose.has_value());
            updatedCurrentTrajectory.addPose(time, *pose);
            updatedTemporaryPoses.push_back(*pose);
        }
    }

    currentTrajectory_ = std::move(updatedCurrentTrajectory);
    occupancyGridMap_->updatePoses(updatedPoses, updatedTemporaryPoses,
        lastNodeIdToIncludeInCachedMap);
}

std::optional<Transform> TimedOccupancyGridMap::getPose(
    const Trajectories& trajectories, const Time& time,
    const Trajectory* activeTrajectoryPtr /* nullptr */,
    const std::optional<Transform>& extrapolationShift /* std::nullopt */)
{
    if (trajectories.empty())
    {
        return std::nullopt;
    }
    for (const Trajectory& trajectory : trajectories)
    {
        std::optional<Transform> pose = trajectory.interpolate(time, maxInterpolationTimeError_);
        if (pose.has_value())
        {
            return pose;
        }
    }
    if (currentTrajectory_.size() &&
        currentTrajectory_.containsTime(time) &&
        activeTrajectoryPtr && extrapolationShift.has_value())
    {
        auto trajectoryIt = trajectories.findCurrentOrPreviousTrajectory(time);
        if (&(*trajectoryIt) == activeTrajectoryPtr)
        {
            std::optional<Transform> currentPose = currentTrajectory_.getPose(time);
            UASSERT(currentPose.has_value() || !currentTrajectory_.containsTime(time));
            if (currentPose.has_value())
            {
                Transform pose = *extrapolationShift * (*currentPose);
                return pose;
            }
        }
    }
    return std::nullopt;
}

void TimedOccupancyGridMap::reset()
{
    occupancyGridMap_->resetAll();
    currentTrajectory_.clear();
    lastPoseTime_ = Time();
    lastTemporaryPoseTime_ = Time();
}

void TimedOccupancyGridMap::save(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__save);
    MapSerialization writer(file);
    proto::OccupancyGridMap::MetaData metaData;
    metaData.set_cell_size(cellSize());
    writer.write(metaData);

    const auto& localMapsRef = localMapsWithoutDilation();
    auto localMapIt = localMapsRef.begin();
    const auto& nodesRef = nodes(0);
    auto nodeIt = nodesRef.begin();
    while (localMapIt != localMapsRef.end())
    {
        UASSERT(nodeIt != nodesRef.end());
        UASSERT(localMapIt->first == nodeIt->first);
        int nodeId = nodeIt->first;
        const Node& node = nodeIt->second;
        proto::OccupancyGridMap::Node proto;
        proto.set_node_id(nodeId);
        if (node.hasPose())
        {
            *proto.mutable_pose() = rtabmap::toProto(node.pose());
        }
        *proto.mutable_local_map() = rtabmap::toProto(*(localMapIt->second));
        writer.write(proto);
        ++localMapIt;
        ++nodeIt;
    }
    UASSERT(nodeIt == nodesRef.end());
    writer.close();
}

void TimedOccupancyGridMap::load(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__load);
    reset();
    MapDeserialization reader(file);
    UASSERT(cellSize() == reader.metaData().cell_size());
    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.readNode())
    {
        std::shared_ptr<LocalMap> localMap =
            rtabmap::fromProto(proto->local_map());
        if (proto->has_pose())
        {
            Transform pose = rtabmap::fromProto(proto->pose());
            addLocalMap(pose, localMap);
        }
        else
        {
            addLocalMap(localMap);
        }
    }
    reader.close();
}

}
