#ifndef KONTIKIV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
#define KONTIKIV2_UNIFORM_R3_SPLINE_TRAJECTORY_H

#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "trajectory.h"
#include "spline_base.h"
#include "../trajectory_estimator.h"

namespace kontiki {
namespace trajectories {

namespace internal {

template<typename T>
struct R3SplineControlPointInfo : public ControlPointInfo<Eigen::Matrix<T, 3, 1>, 3> {
  // No validation or parameterization required
};

template<typename T>
class UniformR3SplineSegmentView : public SplineSegmentView<T, R3SplineControlPointInfo<T>> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using Base = SplineSegmentView<T, R3SplineControlPointInfo<T>>;
 public:
  // Import constructor
  using Base::SplineSegmentView;
  // 由曲线获取t时刻的位置，速度，加速度
  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);
    // std::cout << "t=" << t << " i0=" << i0 << " u=" << u << std::endl;

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    Vector4 Up, Uv, Ua; // 插值函数的u矩阵
    Vector4 Bp, Bv, Ba; //u^T*M矩阵，第i个元素代表u^T和M矩阵的第i列相乘的结果
    T u2, u3;
    Vector3 &p = result->position;
    Vector3 &v = result->velocity;
    Vector3 &a = result->acceleration;
    T dt_inv = T(1)/this->dt();

    if (result->needs.Position() || result->needs.Velocity())
      u2 = ceres::pow(u, 2);
    if (flags & EvalPosition)
      u3 = ceres::pow(u, 3);

    if (result->needs.Position()) {
      Up = Vector4(T(1), u, u2, u3); 
      Bp = Up.transpose()*M.cast<T>();
      p.setZero();
    }

    if (result->needs.Velocity()) {
      Uv = dt_inv*Vector4(T(0), T(1), T(2)*u, T(3)*u2); // (u^T)'/t
      Bv = Uv.transpose()*M.cast<T>();
      v.setZero();
    }

    if (result->needs.Acceleration()) {
      Ua = ceres::pow(dt_inv, 2)*Vector4(T(0), T(0), T(2), T(6)*u); // (u^T)''/(t^2)
      Ba = Ua.transpose()*M.cast<T>();
      a.setZero();
    }

    // 4个控制点
    for (int i = i0; i < i0 + 4; ++i) {
      Vector3Map cp = this->ControlPoint(i); // 每个控制点位置

      if (flags & EvalPosition)
        p += Bp(i - i0)*cp; // 对位置加权

      if (flags & EvalVelocity)
        v += Bv(i - i0)*cp; // 对速度加权

      if (flags & EvalAcceleration)
        a += Ba(i - i0)*cp;

    }

    // This trajectory is not concerned with orientations, so just return identity/zero if requested
    if (result->needs.Orientation())
      result->orientation.setIdentity();
    if (result->needs.AngularVelocity())
      result->angular_velocity.setZero();

    return result;
  }
};

} // namespace internal


class UniformR3SplineTrajectory : public internal::SplineEntity<internal::UniformR3SplineSegmentView> {
 public:
  static constexpr const char* CLASS_ID = "UniformR3SplineTrajectory";
  using internal::SplineEntity<internal::UniformR3SplineSegmentView>::SplineEntity;
};

} // namespace trajectories
} // namespace kontiki

#endif //KONTIKIV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
