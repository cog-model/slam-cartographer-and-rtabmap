#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <yaml-cpp/yaml.h>

#include <deque>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class TemporaryOccupancyGridBuilder
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float temporaryMissProb = 0.4f;
        float temporaryHitProb = 0.7f;
        float temporaryOccupancyProbThr = 0.5f;
        int maxTemporaryLocalMaps = 1;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["TemporaryMissProb"])
            {
                parameters.temporaryMissProb = node["TemporaryMissProb"].as<float>();
            }
            if (node["TemporaryHitProb"])
            {
                parameters.temporaryHitProb = node["TemporaryHitProb"].as<float>();
            }
            if (node["TemporaryOccupancyProbThr"])
            {
                parameters.temporaryOccupancyProbThr =
                    node["TemporaryOccupancyProbThr"].as<float>();
            }
            if (node["MaxTemporaryLocalMaps"])
            {
                parameters.maxTemporaryLocalMaps =
                    node["MaxTemporaryLocalMaps"].as<int>();
            }
            return parameters;
        }
    };

private:
    using CounterType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    struct ColoredGridMap
    {
        std::deque<Node> nodes;
        MapLimitsI mapLimits;
        CounterType hitCounter;
        CounterType missCounter;
        ColorsType colors;
    };

    struct PrecomputedProbabilities
    {
        Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilities;
        Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilitiesThr;
    };

public:
    TemporaryOccupancyGridBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    bool addLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);

    void transformMap(const Transform& transform);

    void updatePoses(const std::deque<Transform>& updatedPoses);

    OccupancyGrid getOccupancyGrid(MapLimitsI roi = MapLimitsI()) const;
    OccupancyGrid getProbOccupancyGrid(MapLimitsI roi = MapLimitsI()) const;
    ColorGrid getColorGrid(MapLimitsI roi = MapLimitsI()) const;

    int maxTemporaryLocalMaps() const { return maxTemporaryLocalMaps_; }
    const std::deque<Node>& nodes() const { return map_.nodes; }
    const MapLimitsI& mapLimits() const { return map_.mapLimits; }

    void reset();

private:
    void precomputeProbabilities();

    void createOrResizeMap(const MapLimitsI& newMapLimits);

    void deployLastNode();
    void deployAllNodes();
    void removeFirstNode();

    void deployTransformedLocalMap(const LocalMap& localMap,
        const TransformedLocalMap& transformedLocalMap);
    void removeTransformedLocalMap(const LocalMap& localMap,
        const TransformedLocalMap& transformedLocalMap);

    void clear();

private:
    float cellSize_;
    float temporaryMissProb_;
    float temporaryHitProb_;
    float temporaryOccupancyProbThr_;
    int updated_;
    int maxTemporaryLocalMaps_;

    ColoredGridMap map_;

    PrecomputedProbabilities precomputedProbabilities_;
};

}
