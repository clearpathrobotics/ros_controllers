/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Clearpath Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#ifndef DIRECT_KINEMATICS_INTEGRATE_FUNCTOR_H_
#define DIRECT_KINEMATICS_INTEGRATE_FUNCTOR_H_

#include <diff_drive_controller/integrate_function.h>

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>

namespace diff_drive_controller
{

  /// Integration with direct kinematics:
  template <typename Functor>
  struct DirectKinematicsIntegrateFunctor
  {
    DirectKinematicsIntegrateFunctor()
      : functor_()
      , wheel_separation_(0.0)
      , left_wheel_radius_(0.0)
      , right_wheel_radius_(0.0)
    {}

    /**
     * \brief Integrates the pose (x, y, yaw) given the wheel velocities
     * (v_l, v_r)
     * \param[in, out] x   Pose x   component
     * \param[in, out] y   Pose y   component
     * \param[in, out] yaw Pose yaw component
     * \param[in] v_l Left  wheel velocity [rad]
     *                (angular  displacement, i.e. rad/s * dt) computed by encoders
     * \param[in] v_r Right wheel velocity [rad]
     *                 (angular  displacement, i.e. rad/s * dt) computed by encoders
     */
    template <typename T>
    // @todo rename v_l, v_r to dp_l, dp_r (here and in other files)
    void operator()(T& x, T& y, T& yaw, const T& v_l, const T& v_r) const
    {
      BOOST_STATIC_ASSERT_MSG(
          !boost::is_pod<T>::value || boost::is_floating_point<T>::value,
          "The pose components must be specified as float values.");

      /// Compute direct kinematics, i.e. obtain linear and angular velocity
      /// from wheel velocities:
      const T vl = v_l * T(left_wheel_radius_);
      const T vr = v_r * T(right_wheel_radius_);

      const T v = (vr + vl) * T(0.5);
      const T w = (vr - vl) / T(wheel_separation_);

      /// Integrate:
      functor_(x, y, yaw, v, w);
    }

    /**
     * \brief Integrates the pose (x, y, yaw) given the wheel velocities
     * (v_l, v_r)
     * \param[in, out] x   Pose x   component
     * \param[in, out] y   Pose y   component
     * \param[in, out] yaw Pose yaw component
     * \param[in] v_l Left  wheel velocity [rad]
     *                (angular  displacement, i.e. rad/s * dt) computed by encoders
     * \param[in] v_r Right wheel velocity [rad]
     *                (angular  displacement, i.e. rad/s * dt) computed by encoders
     * \param[out] J_pose Jacobian wrt the pose (x, y, yaw)
     * \param[out] J_meas Jacobian wrt the meas(urement) wheel velocities
     *                    (v_l, v_r)
     */
    void operator()(double& x, double& y, double& yaw,
        const double& v_l, const double& v_r,
        IntegrateFunction::PoseJacobian& J_pose,
        IntegrateFunction::MeasJacobian& J_meas) const
    {
      /// Compute direct kinematics, i.e. obtain linear and angular velocity
      /// from wheel velocities:
      const double vl = v_l * left_wheel_radius_;
      const double vr = v_r * right_wheel_radius_;

      const double b_inv = 1.0 / wheel_separation_;

      const double v = (vr + vl) * 0.5;
      const double w = (vr - vl) * b_inv;

      /// Jacobian of direct kinematics:
      Eigen::Matrix2d J_dk;
      J_dk << 0.5 * left_wheel_radius_, 0.5 * right_wheel_radius_,
              -left_wheel_radius_ * b_inv, right_wheel_radius_ * b_inv;

      /// Integrate:
      functor_(x, y, yaw, v, w, J_pose, J_meas);

      /// Jacobian wrt wheel velocities (v_l, v_r), applying the chain rule:
      J_meas *= J_dk;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param[in] wheel_separation   Seperation between
     *                               left and right wheels [m]
     * \param[in] left_wheel_radius  Left  wheel radius [m]
     * \param[in] right_wheel_radius Right wheel radius [m]
     */
    void setWheelParams(const double wheel_separation,
        const double left_wheel_radius, const double right_wheel_radius)
    {
      wheel_separation_   = wheel_separation;
      left_wheel_radius_  = left_wheel_radius;
      right_wheel_radius_ = right_wheel_radius;
    }

  private:
    /// Integrate functor:
    Functor functor_;

    /// Wheel parameters:
    double wheel_separation_;
    double left_wheel_radius_;
    double right_wheel_radius_;
  };

} // namespace diff_drive_controller

#endif /* DIRECT_KINEMATICS_INTEGRATE_FUNCTOR_H_ */
