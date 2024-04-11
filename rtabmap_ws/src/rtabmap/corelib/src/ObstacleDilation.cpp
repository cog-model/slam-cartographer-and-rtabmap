#include <rtabmap/core/ObstacleDilation.h>
#include <limits>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

ObstacleDilation::ObstacleDilation(const Parameters& parameters)
{
    parseParameters(parameters);
}

void ObstacleDilation::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    dilationSizeF_ = parameters.dilationSize;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(dilationSizeF_ >= 0.0f);

    dilationSize_ = std::ceil(dilationSizeF_ / cellSize_);
    SemanticDilation::Parameters semanticDilationParameters;
    semanticDilationParameters.dilationSize = dilationSize_;
    semanticDilation_ = std::make_unique<SemanticDilation>(semanticDilationParameters);
}

std::shared_ptr<LocalMap> ObstacleDilation::dilate(
    const LocalMap& localMap) const
{
    UASSERT(dilationSize_ > 0);
    LocalMap::ColoredGrid coloredGrid = localMap.toColoredGrid();
    coloredGrid.grid = semanticDilation_->dilate(coloredGrid.grid,
        {LocalMap::ColoredGrid::occupiedCellValue} /* backgroundColors */,
        true /* dilateBackground */);

    auto dilatedLocalMap = std::make_shared<LocalMap>(
        coloredGrid, 0.0f, localMap.pointsDuplicated(),
        localMap.properties());
    return dilatedLocalMap;
}

}
