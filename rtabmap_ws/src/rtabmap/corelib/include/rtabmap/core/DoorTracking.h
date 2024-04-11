#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace rtabmap {

class DoorTracking
{
private:
    static constexpr std::uint8_t occupiedCellValue = 100;

public:
    using Cell = std::pair<int, int>;  // (y, x)
    using Segment = std::vector<Cell>;

    DoorTracking() {};
    DoorTracking(int smallRadius, int largeRadius);
    ~DoorTracking() {};

    void initialize(int smallRadius, int largeRadius);
    std::pair<Cell, Cell> trackDoor(const cv::Mat& occupancyGrid, const Cell& doorCenterEstimation);

private:
    inline int cellsDistanceSqr(const Cell& a, const Cell& b);

    void precomputeCellToCheckForOccupation();

    std::vector<Cell> getOccupiedCells(const cv::Mat& occupancyGrid, const Cell& doorCenterEstimation);
    std::vector<Segment> segmentation(const std::vector<Cell>& cells);
    std::pair<Cell, Cell> findClosestCellsInSegments(const Segment& segment1, const Segment& segment2);

private:
    bool initialized_ = false;

    int smallRadius_;
    int smallRadiusSqr_;
    int doubleSmallRadius_;
    int doubleSmallRadiusSqr_;
    int largeRadius_;
    int largeRadiusSqr_;
    int doubleLargeRadius_;
    int doubleLargeRadiusSqr_;

    std::vector<Cell> cellToCheckForOccupation_;
};

}
