#include <rtabmap/core/DoorTracking.h>
#include <rtabmap/utilite/ULogger.h>
#include <kas_utils/time_measurer.h>

namespace rtabmap {

DoorTracking::DoorTracking(int smallRadius, int largeRadius)
{
    initialize(smallRadius, largeRadius);
}

void DoorTracking::initialize(int smallRadius, int largeRadius)
{
    UASSERT(initialized_ == false);
    smallRadius_ = smallRadius;
    largeRadius_ = largeRadius;
    smallRadiusSqr_ = smallRadius_ * smallRadius_;
    doubleSmallRadius_ = smallRadius_ * 2;
    doubleSmallRadiusSqr_ = doubleSmallRadius_ * doubleSmallRadius_;
    largeRadiusSqr_ = largeRadius_ * largeRadius_;
    doubleLargeRadius_ = largeRadius_ * 2;
    doubleLargeRadiusSqr_ = doubleLargeRadius_ * doubleLargeRadius_;
    precomputeCellToCheckForOccupation();
    initialized_ = true;
}

inline int DoorTracking::cellsDistanceSqr(const Cell& a, const Cell& b)
{
    int yDistance = a.first - b.first;
    int xDistance = a.second - b.second;
    int distanceSqr = yDistance * yDistance + xDistance * xDistance;
    return distanceSqr;
}

void DoorTracking::precomputeCellToCheckForOccupation()
{
    for (int y = -doubleLargeRadius_; y <= doubleLargeRadius_; y++)
    {
        for (int x = -doubleLargeRadius_; x <= doubleLargeRadius_; x++)
        {
            if (y * y + x * x <= doubleLargeRadiusSqr_)
            {
                cellToCheckForOccupation_.emplace_back(y, x);
            }
        }
    }
}

std::vector<DoorTracking::Cell>
DoorTracking::getOccupiedCells(const cv::Mat& occupancyGrid, const Cell& doorCenterEstimation)
{
    std::vector<Cell> occupiedCells;
    int ey = doorCenterEstimation.first;
    int ex = doorCenterEstimation.second;
    for (const auto& cell : cellToCheckForOccupation_)
    {
        int cy = cell.first + ey;
        int cx = cell.second + ex;
        if (cy < 0 || cx < 0 || cy >= occupancyGrid.rows || cx >= occupancyGrid.cols)
        {
            continue;
        }
        if (occupancyGrid.at<std::uint8_t>(cy, cx) == occupiedCellValue)
        {
            occupiedCells.emplace_back(cy, cx);
        }
    }
    return occupiedCells;
}

std::vector<DoorTracking::Segment> DoorTracking::segmentation(const std::vector<Cell>& cells)
{
    std::vector<Segment> segments;
    std::vector<int> notUsedCellIndices;
    notUsedCellIndices.reserve(cells.size());
    for (int i = 0; i < cells.size(); i++)
    {
        notUsedCellIndices.push_back(i);
    }
    while (notUsedCellIndices.size())
    {
        Segment& segment = segments.emplace_back();
        segment.reserve(cells.size());
        std::deque<int> nextCellIndices;
        int newCellI = notUsedCellIndices.back();
        notUsedCellIndices.pop_back();
        nextCellIndices.push_back(newCellI);
        segment.push_back(cells[newCellI]);
        while (nextCellIndices.size())
        {
            int nextCellI = nextCellIndices.front();
            nextCellIndices.pop_front();
            std::vector<int> newCellIndices;
            const Cell& nextCell = cells[nextCellI];
            for (auto it = notUsedCellIndices.end() - 1; it >= notUsedCellIndices.begin();)
            {
                int notUsedCellI = *it;
                const Cell& notUsedCell = cells[notUsedCellI];
                int distanceSqr = cellsDistanceSqr(nextCell, notUsedCell);
                if (distanceSqr <= doubleSmallRadiusSqr_)
                {
                    auto removeIt = it;
                    --it;
                    notUsedCellIndices.erase(removeIt);
                    nextCellIndices.push_back(notUsedCellI);
                    segment.push_back(notUsedCell);
                }
                else
                {
                    --it;
                }
            }
        }
    }
    return segments;
}

std::pair<DoorTracking::Cell, DoorTracking::Cell>
DoorTracking::findClosestCellsInSegments(const Segment& segment1, const Segment& segment2)
{
    int minDistanceSqr = std::numeric_limits<int>::max();
    Cell closestCell1;
    Cell closestCell2;
    for (int i = 0; i < segment1.size(); i++)
    {
        for (int j = 0; j < segment2.size(); j++)
        {
            const Cell& cell1 = segment1[i];
            const Cell& cell2 = segment2[j];
            int distanceSqr = cellsDistanceSqr(cell1, cell2);
            if (distanceSqr < minDistanceSqr)
            {
                closestCell1 = cell1;
                closestCell2 = cell2;
                minDistanceSqr = distanceSqr;
            }
        }
    }
    return std::make_pair(closestCell1, closestCell2);
}

std::pair<DoorTracking::Cell, DoorTracking::Cell>
DoorTracking::trackDoor(const cv::Mat& occupancyGrid, const Cell& doorCenterEstimation)
{
    MEASURE_BLOCK_TIME(DoorTracking__trackDoor);
    UASSERT(initialized_ == true);
    UASSERT(occupancyGrid.type() == CV_8U);
    const std::vector<Cell>& occupiedCells = getOccupiedCells(occupancyGrid, doorCenterEstimation);
    const std::vector<Segment>& segments = segmentation(occupiedCells);
    int minEstimationErrorSqr = std::numeric_limits<int>::max();
    std::pair<Cell, Cell> doorCorners(Cell(-1, -1), Cell(-1, -1));
    for (int i = 0; i < (int)segments.size() - 1; i++)
    {
        for (int j = i + 1; j < (int)segments.size(); j++)
        {
            Cell doorCorner1, doorCorner2;
            std::tie(doorCorner1, doorCorner2) = findClosestCellsInSegments(segments[i], segments[j]);
            Cell doorCenter;
            doorCenter.first = (doorCorner1.first + doorCorner2.first) / 2;
            doorCenter.second = (doorCorner1.second + doorCorner2.second) / 2;
            int estimationErrorSqr = cellsDistanceSqr(doorCenter, doorCenterEstimation);
            if (estimationErrorSqr < minEstimationErrorSqr)
            {
                doorCorners = std::make_pair(doorCorner1, doorCorner2);
                minEstimationErrorSqr = estimationErrorSqr;
            }
        }
    }
    return doorCorners;
}

}
