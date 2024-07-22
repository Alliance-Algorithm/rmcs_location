#pragma once

namespace utility {

// @brief convert between ros2 translation and eigen
template <typename Ros2Translation>
concept ros2_translation = requires(Ros2Translation t) {
    t.x;
    t.y;
    t.z;
};
template <typename EigenTranslation>
concept eigen_translation = requires(EigenTranslation t) {
    t.x();
    t.y();
    t.z();
};
inline void set_translation(ros2_translation auto& dst, const eigen_translation auto& src)
{
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

inline void set_translation(eigen_translation auto& dst, const ros2_translation auto& src)
{
    dst.x() = src.x;
    dst.y() = src.y;
    dst.z() = src.z;
}

// @brief convert between ros2 quaternion and eigen
template <typename RosQuaternion>
concept ros2_quaternion = requires(RosQuaternion q) {
    q.w;
    q.x;
    q.y;
    q.z;
};
template <typename EigenQuaternion>
concept eigen_quaternion = requires(EigenQuaternion q) {
    q.w();
    q.x();
    q.y();
    q.z();
};
inline void set_quaternion(ros2_quaternion auto& dst, const eigen_quaternion auto& src)
{
    dst.w = src.w();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}
inline void set_quaternion(eigen_quaternion auto& dst, const ros2_quaternion auto& src)
{
    dst.w() = src.w;
    dst.x() = src.x;
    dst.y() = src.y;
    dst.z() = src.z;
}
}