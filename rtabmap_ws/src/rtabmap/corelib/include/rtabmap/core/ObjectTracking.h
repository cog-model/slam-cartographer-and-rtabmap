#pragma once

#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/LocalMap.h>

#include <vector>
#include <utility>
#include <functional>

namespace rtabmap {

class ObjectTracking
{
public:
    struct Cell
    {
        Cell(int otherY, int otherX) :
            y(otherY),
            x(otherX) {}

        static int distanceSqr(const Cell& a, const Cell& b)
        {
            int dy = a.y - b.y;
            int dx = a.x - b.x;
            return dy * dy + dx * dx;
        }

        Cell operator+(const Cell& other)
        {
            return Cell(y + other.y, x + other.x);
        }

        int y;
        int x;
    };

    struct Point
    {
        Point() : x(0.0f), y(0.0f) {}
        Point(float otherX, float otherY) :
            x(otherX),
            y(otherY) {}

        static float distanceSqr(const Point& a, const Point& b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        float x;
        float y;
    };

    struct Velocity
    {
        Velocity() : vx(0.0f), vy(0.0f) {}

        float vx;
        float vy;
    };

    struct TrackedObject
    {
        std::int32_t id = -1;
        std::uint32_t trackedTimes = 0;
        Point position;
        Velocity velocity;
        std::vector<Cell> object;
        Color color;
    };

    struct MOT16TrackedObject
    {
        std::int32_t frameId = -1;
        std::int32_t id = -1;
        Point center;
        float width = 0.0;
        float height = 0.0;

        std::string toMOT16Entry() const
        {
            std::string mot16Entry =
                std::to_string(frameId + 1) + ", " +
                std::to_string(id + 1) + ", " +
                std::to_string(center.x - width / 2) + ", " +
                std::to_string(center.y - height / 2) + ", " +
                std::to_string(width) + ", " +
                std::to_string(height) + ", " +
                "0.9, -1, -1, -1";
            return mot16Entry;
        }
    };

private:

    struct Score
    {
        bool operator<(const Score& other) { return score < other.score; }

        float score;
        int oldIndex;
        int newIndex;
    };

public:
    ObjectTracking(float cellSize);

    void track(const LocalMap& localMap, const Transform& pose);

    const std::vector<TrackedObject>& trackedObjects() const
        { return trackedObjects_; }
    const std::list<MOT16TrackedObject>& mot16TrackedObjectsCache() const
        { return mot16TrackedObjectsCache_; }

private:
    std::vector<TrackedObject> detect(
        const LocalMap& localMap, const Transform& pose) const;
    TrackedObject segment(cv::Mat& colorGrid, const Cell& startCell,
        const MapLimitsI& mapLimits, const Transform& pose) const;
    std::vector<TrackedObject> assign(
        const std::vector<TrackedObject>& trackedObjects, float dt) const;
    void update(std::vector<TrackedObject>& trackedObjects,
        const std::vector<TrackedObject>& assignedTrackedObjects,
        float dt);

    void cacheTrackedObjectsInMOT16Format(
        const std::vector<TrackedObject>& trackedObjects,
        const Transform& pose);

private:
    float cellSize_;
    std::vector<TrackedObject> trackedObjects_;

    std::int32_t frameId_ = 0;

    Time prevTime_;
    std::int32_t nextTrackedId_ = 0;

    std::list<MOT16TrackedObject> mot16TrackedObjectsCache_;

    std::vector<Cell> neighborCells_;
};

}
