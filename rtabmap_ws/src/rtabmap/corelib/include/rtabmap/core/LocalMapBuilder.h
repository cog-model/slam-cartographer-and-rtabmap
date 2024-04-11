#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/RayTracing.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Grid.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

class LocalMapBuilder
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float maxVisibleRange = -1.0f;  // inf
        float minObstacleHeight = 0.2f;
        float maxObstacleHeight = 1.5f;
        float minSemanticRange = 0.0f;
        float maxSemanticRange = -1.0f;  // inf
        bool enableRayTracing = false;
        float maxRange2d = 10.0f;  // (-1) - inf
        float sensorBlindRange2d = 0.0f;

        SemanticDilation::Parameters semanticDilationParameters;
        RayTracing::Parameters rayTracingParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["MaxVisibleRange"])
            {
                parameters.maxVisibleRange = node["MaxVisibleRange"].as<float>();
            }
            if (node["MinObstacleHeight"])
            {
                parameters.minObstacleHeight = node["MinObstacleHeight"].as<float>();
            }
            if (node["MaxObstacleHeight"])
            {
                parameters.maxObstacleHeight = node["MaxObstacleHeight"].as<float>();
            }
            if (node["MinSemanticRange"])
            {
                parameters.minSemanticRange = node["MinSemanticRange"].as<float>();
            }
            if (node["MaxSemanticRange"])
            {
                parameters.maxSemanticRange = node["MaxSemanticRange"].as<float>();
            }
            if (node["EnableRayTracing"])
            {
                parameters.enableRayTracing = node["EnableRayTracing"].as<bool>();
            }
            if (node["MaxRange2d"])
            {
                parameters.maxRange2d = node["MaxRange2d"].as<float>();
            }
            if (node["SensorBlindRange2d"])
            {
                parameters.sensorBlindRange2d = node["SensorBlindRange2d"].as<float>();
            }
            if (node["SemanticDilation"])
            {
                parameters.semanticDilationParameters =
                    SemanticDilation::Parameters::createParameters(
                        node["SemanticDilation"]);
            }
            if (node["RayTracing"])
            {
                parameters.rayTracingParameters =
                    RayTracing::Parameters::createParameters(
                        node["RayTracing"]);
            }
            return parameters;
        }
    };

    static const cv::Vec3b semanticBackgroundColor;  // (0, 0, 0)

public:
    LocalMapBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    const cv::Mat& lastDilatedSemantic() const { return lastDilatedSemantic_; }

private:
    Eigen::Matrix3Xf convertLaserScan(const LaserScan& laserScan) const;
    Eigen::Matrix3Xf filterMaxVisibleRange(const Eigen::Matrix3Xf& points) const;
    Eigen::Matrix3Xf transformPoints(const Eigen::Matrix3Xf& points,
        const Transform& transform) const;
    Eigen::Matrix3Xf getObstaclePoints(const Eigen::Matrix3Xf& points) const;

    std::vector<Color> getPointsColors(const Eigen::Matrix3Xf& points,
        const std::vector<cv::Mat>& images,
        const std::vector<rtabmap::CameraModel>& cameraModels) const;

    LocalMap::ColoredGrid coloredGridFromObstacles(const Eigen::Matrix3Xf& points,
        const std::vector<Color>& colors,
        const Eigen::Vector2f& sensor) const;
    void traceRays(LocalMap::ColoredGrid& coloredGrid,
        const Eigen::Vector2f& sensor) const;

private:
    float cellSize_;
    float maxVisibleRange_;
    float maxVisibleRangeSqr_;
    float minObstacleHeight_;
    float maxObstacleHeight_;
    float minSemanticRange_;
    float minSemanticRangeSqr_;
    float maxSemanticRange_;
    float maxSemanticRangeSqr_;
    bool enableRayTracing_;
    float maxRange2d_;
    float maxRange2dSqr_;
    float sensorBlindRange2d_;
    float sensorBlindRange2dSqr_;

    std::unique_ptr<SemanticDilation> semanticDilation_;
    std::unique_ptr<RayTracing> rayTracing_;

    mutable cv::Mat lastDilatedSemantic_;
};

}
