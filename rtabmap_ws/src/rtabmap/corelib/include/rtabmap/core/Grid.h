#pragma once

#include <rtabmap/core/MapLimits.h>

#include <cmath>
#include <Eigen/Core>

namespace rtabmap {

struct OccupancyGrid
{
    using GridType = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MapLimitsI limits;
    GridType grid;
};

struct ColorGrid
{
    using GridType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MapLimitsI limits;
    GridType grid;
};

inline float logodds(float probability)
{
    return std::log(probability / (1.0f - probability));
}

inline float probability(float logodds)
{
    return 1.0f - (1.0f / (1.0f + std::exp(logodds)));
}

}