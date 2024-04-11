#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/LocalMap.h>

#include <yaml-cpp/yaml.h>

#include <memory>

namespace rtabmap {

class ObstacleDilation
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float dilationSize = 0.0f;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["DilationSize"])
            {
                parameters.dilationSize = node["DilationSize"].as<float>();
            }
            return parameters;
        }
    };

public:
    ObstacleDilation(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> dilate(const LocalMap& localMap) const;

    float dilationSize() { return dilationSizeF_; }

private:
    float cellSize_;
    float dilationSizeF_;
    int dilationSize_;

    std::unique_ptr<SemanticDilation> semanticDilation_;
};

}
