#ifndef OFFLINE_ROS_COMPAT_H
#define OFFLINE_ROS_COMPAT_H

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace ros {

class Duration {
public:
    Duration() = default;
    explicit Duration(double sec) : sec_(sec) {}
    double toSec() const { return sec_; }

private:
    double sec_{0.0};
};

class Time {
public:
    Time() = default;
    explicit Time(double sec) : sec_(sec) {}

    static Time now()
    {
        using clock = std::chrono::steady_clock;
        const auto now = clock::now().time_since_epoch();
        return Time(std::chrono::duration<double>(now).count());
    }

    Time fromSec(double sec) const { return Time(sec); }
    double toSec() const { return sec_; }

private:
    double sec_{0.0};
};

inline Duration operator-(const Time& lhs, const Time& rhs)
{
    return Duration(lhs.toSec() - rhs.toSec());
}

class Publisher {
public:
    template <typename T>
    void publish(const T&) const {}
};

}  // namespace ros

namespace std_msgs {

struct Header {
    std::uint32_t seq{0};
    ros::Time stamp;
    std::string frame_id;
};

}  // namespace std_msgs

namespace geometry_msgs {

struct Point {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseWithCovariance {
    Pose pose;
    std::array<double, 36> covariance{};
};

struct PoseWithCovarianceStamped {
    std_msgs::Header header;
    PoseWithCovariance pose;
};

}  // namespace geometry_msgs

#endif
