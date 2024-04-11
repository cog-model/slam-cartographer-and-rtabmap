#include <rtabmap/core/Node.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

void TransformedLocalMap::set(const LocalMap& localMap, const Transform& pose,
    float cellSize)
{
    points_.resize(2, localMap.points().cols());
    Eigen::Matrix3Xf transformedPoints =
        (pose.toEigen3fRotation() * localMap.points()).colwise() +
        pose.toEigen3fTranslation();
    mapLimits_ = MapLimitsI();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int x = std::floor(transformedPoints(0, i) / cellSize);
        int y = std::floor(transformedPoints(1, i) / cellSize);
        points_.coeffRef(0, i) = x;
        points_.coeffRef(1, i) = y;
        mapLimits_.update(x, y);
    }
    // UASSERT(mapLimits_.valid());
}

}
