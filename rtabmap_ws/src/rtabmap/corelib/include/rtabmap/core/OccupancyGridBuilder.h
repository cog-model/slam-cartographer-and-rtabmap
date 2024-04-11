#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <yaml-cpp/yaml.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <functional>
#include <Eigen/Core>

namespace rtabmap {

class OccupancyGridBuilder
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float missProb = 0.4f;
        float hitProb = 0.7f;
        float minClampingProb = 0.1192f;
        float maxClampingProb = 0.971f;
        float occupancyProbThr = 0.5f;
        int temporarilyOccupiedCellColorRgb = -1;
        bool showTemporarilyOccupiedCells = true;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["MissProb"])
            {
                parameters.missProb = node["MissProb"].as<float>();
            }
            if (node["HitProb"])
            {
                parameters.hitProb = node["HitProb"].as<float>();
            }
            if (node["MinClampingProb"])
            {
                parameters.minClampingProb = node["MinClampingProb"].as<float>();
            }
            if (node["MaxClampingProb"])
            {
                parameters.maxClampingProb = node["MaxClampingProb"].as<float>();
            }
            if (node["OccupancyProbThr"])
            {
                parameters.occupancyProbThr = node["OccupancyProbThr"].as<float>();
            }
            if (node["TemporarilyOccupiedCellColorRgb"])
            {
                parameters.temporarilyOccupiedCellColorRgb =
                    node["TemporarilyOccupiedCellColorRgb"].as<int>();
            }
            if (node["ShowTemporarilyOccupiedCells"])
            {
                parameters.showTemporarilyOccupiedCells =
                    node["ShowTemporarilyOccupiedCells"].as<bool>();
            }
            return parameters;
        }
    };

private:
    using MapType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using Cell = std::pair<int, int>;
    using Cells = std::vector<Cell>;

    struct ColoredGridMap
    {
        std::map<int, Node> nodes;
        MapLimitsI mapLimits;
        MapType map;
        ColorsType colors;
        Cells temporarilyOccupiedCells;
    };

    struct CachedColoredGridMap
    {
        std::map<int, Transform> poses;
        MapLimitsI mapLimits;
        MapType map;
        ColorsType colors;
        Cells temporarilyOccupiedCells;
    };

    struct PrecomputedUpdateValues
    {
        // value 0 = unknown
        // value 1 = prob 0.0
        // value (splitNum + 1) = prob 1.0
        static constexpr int splitNum = 1000;
        static constexpr int updated = splitNum * 2;
        static constexpr int unknown = 0;
        std::vector<int> missUpdates;
        std::vector<int> hitUpdates;
        std::vector<std::int8_t> probabilities;
        std::vector<std::int8_t> probabilitiesThr;
    };

public:
    OccupancyGridBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    int addLocalMap(const std::shared_ptr<const LocalMap>& localMap);
    int addLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);

    void removeNodes(const std::vector<int>& nodeIdsToRemove);

    void transformMap(const Transform& transform);

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        int lastNodeIdToIncludeInCachedMap = -1);

    OccupancyGrid getOccupancyGrid(MapLimitsI roi = MapLimitsI()) const;
    OccupancyGrid getProbOccupancyGrid(MapLimitsI roi = MapLimitsI()) const;
    ColorGrid getColorGrid(MapLimitsI roi = MapLimitsI()) const;

    const std::map<int, Node>& nodes() const { return map_.nodes; }
    const MapLimitsI& mapLimits() const { return map_.mapLimits; }

    void reset();

private:
    void precomputeUpdateValues();

    void updateCachedMap();
    bool cachedMapCanBeUsed(const std::map<int, Transform>& newPoses);
    void useCachedMap();
    bool tryToUseCachedMap(const std::map<int, Transform>& newPoses);

    void createOrResizeMap(const MapLimitsI& newMapLimits);

    void deployNode(const Node& node);
    void deployNodes(const std::vector<std::reference_wrapper<Node>>& nodes);

    void deployTransformedLocalMap(const LocalMap& localMap,
        const TransformedLocalMap& transformedLocalMap);

    void clear();
    void clearCachedMap();

    float valueToProbability(int value)
    {
        if (value == 0)
        {
            return 0.5f;
        }
        return 1.0f * (value - 1) / PrecomputedUpdateValues::splitNum;
    }

    int probabilityToVlaue(float probability)
    {
        return std::lround(probability * PrecomputedUpdateValues::splitNum) + 1;
    }

private:
    float cellSize_;
    float missProb_;
    float hitProb_;
    float minClampingProb_;
    float maxClampingProb_;
    float occupancyProbThr_;
    int occupancyThr_;
    int temporarilyOccupiedCellColorRgb_;
    Color temporarilyOccupiedCellColor_;
    bool showTemporarilyOccupiedCells_;

    ColoredGridMap map_;
    CachedColoredGridMap cachedMap_;

    PrecomputedUpdateValues updateValues_;
};

}
