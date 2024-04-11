#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/MapLimits.h>

#include <optional>
#include <memory>
#include <Eigen/Core>

namespace rtabmap {

class TransformedLocalMap
{
public:
    TransformedLocalMap(const LocalMap& localMap, const Transform& pose, float cellSize)
    {
        set(localMap, pose, cellSize);
    }

    void set(const LocalMap& localMap, const Transform& pose, float cellSize);

    const MapLimitsI& mapLimits() const { return mapLimits_; }
    const Eigen::Matrix2Xi& points() const { return points_; }

private:
    MapLimitsI mapLimits_;
    Eigen::Matrix2Xi points_;
};

class Node
{
public:
    template <typename T>
    Node(const std::shared_ptr<T>& localMap) :
        localMap_(localMap),
        pose_(std::nullopt),
        transformedLocalMap_(std::nullopt) {}
    template <typename T, typename U>
    Node(const std::shared_ptr<T>& localMap, U&& pose) :
        localMap_(localMap),
        pose_(std::forward<U>(pose)),
        transformedLocalMap_(std::nullopt) {}
    template <typename T, typename U>
    Node(const std::shared_ptr<T>& localMap, U&& pose, float cellSize) :
        localMap_(localMap),
        pose_(std::forward<U>(pose)),
        transformedLocalMap_(std::in_place, *localMap, pose, cellSize) {}

    const std::shared_ptr<const LocalMap>& localMap() const { return localMap_; }

    bool hasPose() const { return pose_.has_value(); }
    void setPose(const Transform& pose) { pose_ = pose; transformedLocalMap_.reset(); }
    const Transform& pose() const { return *pose_; }
    void removePose() { pose_.reset(); transformedLocalMap_.reset(); }

    bool hasTransformedLocalMap() const { return transformedLocalMap_.has_value(); }
    void transformLocalMap(const Transform& pose, float cellSize)
    {
        pose_ = pose;
        if (transformedLocalMap_.has_value())
        {
            transformedLocalMap_->set(*localMap_, *pose_, cellSize);
        }
        else
        {
            transformedLocalMap_ = TransformedLocalMap(*localMap_, *pose_, cellSize);
        }
    }
    const TransformedLocalMap& transformedLocalMap() const { return *transformedLocalMap_; }
    void removeTransformedLocalMap() { transformedLocalMap_.reset(); }

private:
    const std::shared_ptr<const LocalMap> localMap_;
    std::optional<Transform> pose_;
    std::optional<TransformedLocalMap> transformedLocalMap_;
};

}