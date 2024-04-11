#pragma once

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>

#include <rtabmap/proto/Trajectory.pb.h>

#include <set>
#include <utility>
#include <optional>
#include <limits>

namespace rtabmap {

struct TimedPose
{
    struct CompareId
    {
        using is_transparent = void;
        bool operator()(const TimedPose& a, const TimedPose& b) const
        {
            return a.time < b.time;
        }
        bool operator()(const TimedPose& a, const Time& time) const
        {
            return a.time < time;
        }
        bool operator()(const Time& time, const TimedPose& a) const
        {
            return time < a.time;
        }
    };

    template<typename T, typename U>
    TimedPose(T&& otherTime, U&& otherPose) :
        time(std::forward<T>(otherTime)),
        pose(std::forward<U>(otherPose)) {}
    Time time;
    Transform pose;
};


class Trajectory
{
public:
    struct CompareId
    {
        using is_transparent = void;
        bool operator()(const Trajectory& a, const Trajectory& b) const
        {
            return a < b;
        }
        bool operator()(const Trajectory& a, const Time& time) const
        {
            return a.maxTime() < time;
        }
        bool operator()(const Time& time, const Trajectory& a) const
        {
            return time < a.minTime();
        }
    };

    using TimedPosesSet = std::set<TimedPose, TimedPose::CompareId>;
    using const_iterator = TimedPosesSet::const_iterator;
    using Bounds = std::pair<
        TimedPosesSet::const_iterator, TimedPosesSet::const_iterator>;

public:
    void addPose(const Time& time, const Transform& pose)
    {
        const_iterator it;
        bool emplaced;
        std::tie(it, emplaced) = trajectory_.emplace(time, pose);
        UASSERT(emplaced || it->pose == pose);
    }
    const Time& minTime() const
    {
        UASSERT(trajectory_.size());
        return trajectory_.begin()->time;
    }
    const Time& maxTime() const
    {
        UASSERT(trajectory_.size());
        return trajectory_.rbegin()->time;
    }
    bool containsTime(const Time& time) const
    {
        UASSERT(trajectory_.size());
        return minTime() <= time && time <= maxTime();
    }
    bool hasTime(const Time& time) const
    {
        UASSERT(trajectory_.size());
        return trajectory_.count(time);
    }
    std::optional<Transform> getPose(const Time& time)
    {
        auto it = trajectory_.find(time);
        if (it == trajectory_.end())
        {
            return std::nullopt;
        }
        return it->pose;
    }
    Bounds getBounds(const Time& time) const;
    std::optional<Transform> interpolate(const Time& time,
        double maxInterpolationTimeError = std::numeric_limits<double>::max()) const;
    void trim(const Time& time);
    size_t size() const
    {
        return trajectory_.size();
    }
    bool empty() const
    {
        return trajectory_.empty();
    }
    void clear()
    {
        trajectory_.clear();
    }
    bool operator<(const Trajectory& other) const
    {
        return maxTime() < other.minTime();
    }
    const_iterator begin() const
    {
        return trajectory_.cbegin();
    }
    const_iterator end() const
    {
        return trajectory_.cend();
    }

private:
    TimedPosesSet trajectory_;
};


class Trajectories
{
public:
    using TrajectoriesSet = std::set<Trajectory, Trajectory::CompareId>;
    using const_iterator = TrajectoriesSet::const_iterator;
    class const_pose_iterator
    {
    public:
        static const_pose_iterator beginOf(const Trajectories& trajectories);
        static const_pose_iterator endOf(const Trajectories& trajectories);
        static const_pose_iterator fromPose(const Trajectories& trajectories,
            const const_iterator& trajectoryIt, const Trajectory::const_iterator& poseIt);

        const_pose_iterator(const Trajectories& trajectories) :
            trajectoriesEnd_(trajectories.end()) {}

        const_pose_iterator& operator++();
        const_pose_iterator operator++(int);

        const_pose_iterator& operator--();
        const_pose_iterator operator--(int);

        const TimedPose& operator*() { return *poseIt_; }
        const TimedPose* operator->() { return &(*poseIt_); }
        const Trajectory& trajectory() { return *trajectoryIt_; }

        bool operator==(const const_pose_iterator& other);
        bool operator!=(const const_pose_iterator& other) { return !operator==(other); }

    private:
        const_iterator trajectoryIt_;
        Trajectory::const_iterator poseIt_;
        const const_iterator trajectoriesEnd_;
    };

public:
    template<typename T>
    void addTrajectory(T&& trajectory)
    {
        UASSERT(trajectory.size());
        bool emplaced = trajectories_.emplace(std::forward<T>(trajectory)).second;
        UASSERT(emplaced);
    }
    const_iterator findCurrentTrajectory(const Time& time) const
    {
        return trajectories_.find(time);
    }
    const_iterator findPreviousTrajectory(const Time& time) const
    {
        auto it = trajectories_.lower_bound(time);
        if (it == trajectories_.begin())
        {
            return trajectories_.end();
        }
        return std::prev(it);
    }
    const_iterator findCurrentOrPreviousTrajectory(const Time& time) const
    {
        auto it = trajectories_.upper_bound(time);
        if (it == trajectories_.begin())
        {
            return trajectories_.end();
        }
        return std::prev(it);
    }
    const_iterator findCurrentOrNextTrajectory(const Time& time) const
    {
        auto it = trajectories_.lower_bound(time);
        return it;
    }
    size_t size() const
    {
        return trajectories_.size();
    }
    bool empty() const
    {
        return trajectories_.empty();
    }
    const_iterator begin() const
    {
        return trajectories_.cbegin();
    }
    const_iterator end() const
    {
        return trajectories_.cend();
    }
    const_pose_iterator poses_begin() const
    {
        return const_pose_iterator::beginOf(*this);
    }
    const_pose_iterator poses_end() const
    {
        return const_pose_iterator::endOf(*this);
    }
    const_pose_iterator poses_iterator(
        const const_iterator& trajectoryIt,
        const Trajectory::const_iterator& poseIt) const
    {
        return const_pose_iterator::fromPose(*this, trajectoryIt, poseIt);
    }

private:
    TrajectoriesSet trajectories_;
};

proto::TimedPose toProto(const TimedPose& timedPose);
TimedPose fromProto(const proto::TimedPose& proto);

proto::Trajectory toProto(const Trajectory& trajectory);
Trajectory fromProto(const proto::Trajectory& proto);

proto::Trajectories toProto(const Trajectories& trajectories);
Trajectories fromProto(const proto::Trajectories& proto);

}
