/***************************************************************************************************************************
 * math_utils.h
 *
 * Author: Qyp
 *
 * Update Time: 2019.3.16
 *
 * Introduction:  math utils functions 数学工具函数
 *
 *               1、转换 ref to https://github.com/PX4/Matrix/blob/56b069956da141da244926ed7000e89b2ba6c731/matrix/Euler.hpp
 ***************************************************************************************************************************/
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#define ratio_RAD2DEG      180.0 / M_PI
#define ratio_DEG2RAD      M_PI / 180.0
#define EARTH_RADIUS_METER 6371000  //地球半径

#include <Eigen/Eigen>
#include <cmath>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace std;
namespace math_utils
{
// 四元数转欧拉角
inline Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond& q)
{
    // YPR - ZYX
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
inline Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d& rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

//旋转矩阵转欧拉角
inline void rotation_to_euler(const Eigen::Matrix3d& dcm, Eigen::Vector3d& euler_angle)
{
    double phi_val   = atan2(dcm(2, 1), dcm(2, 2));
    double theta_val = asin(-dcm(2, 0));
    double psi_val   = atan2(dcm(1, 0), dcm(0, 0));
    double pi        = M_PI;

    if (fabs(theta_val - pi / 2.0) < 1.0e-3)
    {
        phi_val = 0.0;
        psi_val = atan2(dcm(1, 2), dcm(0, 2));
    }
    else if (fabs(theta_val + pi / 2.0) < 1.0e-3)
    {
        phi_val = 0.0;
        psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }

    euler_angle(0) = phi_val;
    euler_angle(1) = theta_val;
    euler_angle(2) = psi_val;
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d& R)
{
    Eigen::Vector4d quat;
    double          tr = R.trace();
    if (tr > 0.0)
    {
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0)  = 0.25 * S;
        quat(1)  = (R(2, 1) - R(1, 2)) / S;
        quat(2)  = (R(0, 2) - R(2, 0)) / S;
        quat(3)  = (R(1, 0) - R(0, 1)) / S;
    }
    else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
    {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0)  = (R(2, 1) - R(1, 2)) / S;
        quat(1)  = 0.25 * S;
        quat(2)  = (R(0, 1) + R(1, 0)) / S;
        quat(3)  = (R(0, 2) + R(2, 0)) / S;
    }
    else if (R(1, 1) > R(2, 2))
    {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0)  = (R(0, 2) - R(2, 0)) / S;
        quat(1)  = (R(0, 1) + R(1, 0)) / S;
        quat(2)  = 0.25 * S;
        quat(3)  = (R(1, 2) + R(2, 1)) / S;
    }
    else
    {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0)  = (R(1, 0) - R(0, 1)) / S;
        quat(1)  = (R(0, 2) + R(2, 0)) / S;
        quat(2)  = (R(1, 2) + R(2, 1)) / S;
        quat(3)  = 0.25 * S;
    }
    return quat;
}

inline Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d& m)
{
    Eigen::Vector3d v;
    // TODO: Sanity checks if m is skew symmetric
    v << m(7), m(2), m(3);
    return v;
}

inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d& q, const Eigen::Vector4d& p)
{
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d& q)
{
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3), 2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

// constrain_function
inline float constrain_function(float data, float Max)
{
    if (abs(data) > Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

// constrain_function2
inline float constrain_function2(float data, float Min, float Max)
{
    if (data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }
    else
    {
        return data;
    }
}

// sign_function
inline float sign_function(float data)
{
    if (data > 0)
    {
        return 1.0;
    }
    else if (data < 0)
    {
        return -1.0;
    }
    else if (data == 0)
    {
        return 0.0;
    }
}

// min function
inline float min(float data1, float data2)
{
    if (data1 >= data2)
    {
        return data2;
    }
    else
    {
        return data1;
    }
}

// determine if the target point has been reached
template <typename T> inline bool reachGoal(const T& a, const T& b, double dis)
{
    return std::abs(a - b) <= dis;
}

template <> inline bool reachGoal<Eigen::Vector3d>(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double dis)
{
    return (a - b).norm() <= dis;
}

inline Eigen::Vector3d GPS_2_xyz(const sensor_msgs::msg::NavSatFix& GPS_target, const sensor_msgs::msg::NavSatFix& GPS_ref)
{
    // gps_pos[0][1][2] 经度、维度、海拔
    double lon_rad     = ratio_DEG2RAD * GPS_target.longitude;  // GPS数据角度单位为角度
    double lat_rad     = ratio_DEG2RAD * GPS_target.latitude;   // 角度 -> 弧度    A/57.295
    double ref_lon_rad = ratio_DEG2RAD * GPS_ref.longitude;
    double ref_lat_rad = ratio_DEG2RAD * GPS_ref.latitude;
    double sin_lat     = sin(lat_rad);  //程序中三角运算使用的是弧度
    double cos_lat     = cos(lat_rad);
    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon;

    if (arg > 1.0)
    {
        arg = 1.0;
    }
    else if (arg < -1.0)
    {
        arg = -1.0;  //限幅
    }

    double c = acos(arg);
    double k = (fabs(c) > 0) ? (c / sin(c)) : 1.0;  // c为正数
    // GPS->NED->ENU(x,y对调实现NED->ENU的转换)
    double y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * EARTH_RADIUS_METER;
    double x = k * cos_lat * sin(lon_rad - ref_lon_rad) * EARTH_RADIUS_METER;
    //计算z(z没有转成NED的，可直接使用)
    double z = GPS_target.altitude - GPS_ref.altitude;

    Eigen::Vector3d result(x, y, z);

    return result;
}
}  // namespace math_utils

#endif
