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

#ifndef ANALYTIC_INTEGRATE_FUNCTION_H_
#define ANALYTIC_INTEGRATE_FUNCTION_H_

#include <diff_drive_controller/integrate_function.h>

namespace diff_drive_controller
{

  /**
   * \brief Integrate function, which computes jacobians analytically
   */
  template <template <typename> class IntegrateFunctor, typename Functor>
  class AnalyticIntegrateFunction : public IntegrateFunction
  {
    public:
      /// Pose and measurement jacobian types:
      typedef Eigen::Matrix3d             PoseJacobian;
      typedef Eigen::Matrix<double, 3, 2> MeasJacobian;

      /**
       * \brief Constructor
       */
      AnalyticIntegrateFunction()
        : functor_()
      {}

      /**
       * \brief Integrates w/o computing the jacobians
       * \param[in, out] x Pose x component
       * \param[in, out] y Pose y component
       * \param[in, out] Y Pose yaw component
       * \param[in] v_l Left  wheel velocity (displacement)
       * \param[in] v_r Right wheel velocity (displacement)
       */
      virtual void operator()(double& x, double& y, double& Y,
          const double& v_l, const double& v_r) const
      {
        functor_(x, y, Y, v_l, v_r);
      }

      /**
       * \brief Integrates and computes the jacobians
       * \param[in, out] x Pose x component
       * \param[in, out] y Pose y component
       * \param[in, out] Y Pose yaw component
       * \param[in] v_l Left  wheel velocity (displacement)
       * \param[in] v_r Right wheel velocity (displacement)
       * \param[out] J_pose Jacobian (3x3) wrt the pose (x, y, Y)
       * \param[out] J_meas Jacobian (3x2) wrt the measurement (v_l, v_r)
       */
      virtual void operator()(double& x, double& y, double& Y,
          const double& v_l, const double& v_r,
          PoseJacobian& J_pose, MeasJacobian& J_meas) const
      {
        /// Integrate and compute the partial derivatives:
        functor_(x, y, Y, v_l, v_r, J_pose, J_meas);
      }

      /**
       * \brief Sets the wheel parameters: radius and separation
       * \param[in] wheel_separation   Seperation between
       *                               left and right wheels [m]
       * \param[in] left_wheel_radius  Left  wheel radius [m]
       * \param[in] right_wheel_radius Right wheel radius [m]
       */
      virtual void setWheelParams(const double wheel_separation,
          const double left_wheel_radius, const double right_wheel_radius)
      {
        functor_.setWheelParams(wheel_separation,
            left_wheel_radius, right_wheel_radius);
      }

    private:
      /// Integrate functor:
      IntegrateFunctor<Functor> functor_;
  };

}  // namespace diff_drive_controller

#endif /* ANALYTIC_INTEGRATE_FUNCTION_H_ */
