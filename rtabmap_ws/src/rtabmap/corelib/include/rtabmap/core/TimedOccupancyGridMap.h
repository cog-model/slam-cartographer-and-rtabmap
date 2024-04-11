#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/ObjectTracking.h>
#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/core/TrajectoriesTrimmer.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <set>
#include <map>
#include <optional>
#include <utility>
#include <memory>

namespace rtabmap {

class TimedOccupancyGridMap
{
public:
    struct Parameters
    {
        float maxInterpolationTimeError = 0.06f;
        float guaranteedInterpolationTimeWindow = 1.0f;
        bool enableTrajectoriesTrimmer = false;

        TrajectoriesTrimmer::Parameters trajectoriesTrimmerParameters;
        OccupancyGridMap::Parameters occupancyGridMapParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["MaxInterpolationTimeError"])
            {
                parameters.maxInterpolationTimeError =
                    node["MaxInterpolationTimeError"].as<float>();
            }
            if (node["GuaranteedInterpolationTimeWindow"])
            {
                parameters.guaranteedInterpolationTimeWindow =
                    node["GuaranteedInterpolationTimeWindow"].as<float>();
            }
            if (node["EnableTrajectoriesTrimmer"])
            {
                parameters.enableTrajectoriesTrimmer =
                    node["EnableTrajectoriesTrimmer"].as<bool>();
            }
            if (node["TrajectoriesTrimmer"])
            {
                parameters.trajectoriesTrimmerParameters =
                    TrajectoriesTrimmer::Parameters::createParameters(
                        node["TrajectoriesTrimmer"]);
            }
            if (node["OccupancyGridMap"])
            {
                parameters.occupancyGridMapParameters =
                    OccupancyGridMap::Parameters::createParameters(
                        node["OccupancyGridMap"]);
            }
            return parameters;
        }
    };

public:
    TimedOccupancyGridMap(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    int addLocalMap(const std::shared_ptr<const LocalMap>& localMap);
    int addLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);
    bool addTemporaryLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);

    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& fromUpdatedPose);
    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);
    bool addTemporarySensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);

    void removeNodes(const std::vector<int>& nodeIdsToRemove)
        { occupancyGridMap_->removeNodes(nodeIdsToRemove); }

    void transformMap(const Transform& transform);
    bool trajectoriesTrimmerEnabled() const { return trajectoriesTrimmer_ != nullptr; }
    std::set<Time> trimTrajectories(const Trajectories& trajectories)
        { UASSERT(trajectoriesTrimmer_); return trajectoriesTrimmer_->trimTrajectories(trajectories); }
    void updatePoses(const Trajectories& trajectories);

    OccupancyGrid getOccupancyGrid(int index) const
        { return occupancyGridMap_->getOccupancyGrid(index); }
    OccupancyGrid getProbOccupancyGrid(int index) const
        { return occupancyGridMap_->getProbOccupancyGrid(index); }
    ColorGrid getColorGrid(int index) const
        { return occupancyGridMap_->getColorGrid(index); }

    float cellSize() const { return occupancyGridMap_->cellSize(); }
    std::pair<float, float> getGridOrigin(int index) const
        { return occupancyGridMap_->getGridOrigin(index); }
    int maxTemporaryLocalMaps(int index) const
        { return occupancyGridMap_->maxTemporaryLocalMaps(index); }
    const std::map<int, Node>& nodes(int index) const { return occupancyGridMap_->nodes(index); }
    const std::deque<Node>& temporaryNodes(int index) const
        { return occupancyGridMap_->temporaryNodes(index); }
    const std::map<int, const std::shared_ptr<const LocalMap>>& localMapsWithoutDilation() const
        { return occupancyGridMap_->localMapsWithoutDilation(); }
    const cv::Mat& lastDilatedSemantic() const
        { return occupancyGridMap_->lastDilatedSemantic(); }
    int numBuilders() const { return occupancyGridMap_->numBuilders(); };
    bool objectTrackingEnabled() const
        { return occupancyGridMap_->objectTrackingEnabled(); }
    const std::vector<ObjectTracking::TrackedObject>& trackedObjects() const
        { return occupancyGridMap_->trackedObjects(); }
    const std::list<ObjectTracking::MOT16TrackedObject>& mot16TrackedObjectsCache() const
        { return occupancyGridMap_->mot16TrackedObjectsCache(); }

    void reset();

    void save(const std::string& file);
    void load(const std::string& file);

private:
    std::optional<Transform> getPose(
        const Trajectories& trajectories, const Time& time,
        const Trajectory* activeTrajectoryPtr = nullptr,
        const std::optional<Transform>& extrapolationShift = std::nullopt);

private:
    double maxInterpolationTimeError_;
    double guaranteedInterpolationTimeWindow_;
    bool enableTrajectoriesTrimmer_;

    std::unique_ptr<OccupancyGridMap> occupancyGridMap_;
    std::unique_ptr<TrajectoriesTrimmer> trajectoriesTrimmer_;

    Trajectory currentTrajectory_;

    Time lastPoseTime_;
    Time lastTemporaryPoseTime_;
};

}
