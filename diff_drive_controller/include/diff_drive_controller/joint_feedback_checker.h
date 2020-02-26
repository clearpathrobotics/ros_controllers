/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Clearpath Robotics, Inc.
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

#ifndef DIFF_DRIVE_CONTROLLER_JOINT_FEEDBACK_CHECKER_H
#define DIFF_DRIVE_CONTROLLER_JOINT_FEEDBACK_CHECKER_H

#include <cmath>

namespace diff_drive_controller
{

  /**
   * \brief Joint feedback checker, that checks if the error has converged
   * after the settling time, using a error band.
   *
   * This check can be used to apply the following logic for each joint with
   * actual and actual estimated state (wrt the desired state):
   * 1. Both converge, which implies they're similar => both are SANE
   * 2. The actual OR the actual estimated does NOT converge:
   *    1. The actual           converge => SANE, actual estimated INSANE
   *    2. The actual estimated converge => SANE, actual           INSANE
   *    3. NONE converge => both are INSANE
   *       1. They're different => use the one closer to the desired
   *       2. They're similar   => do nothing (they are equally bad)
   */
  class JointFeedbackChecker
  {
  public:
    /**
     * \brief Constructor.
     * \param[in] settling_time  Settling time [s].
     * \param[in] error_band     Error band [rad/s].
     * \param[in] error_estimate Error (discrepancy) wrt estimate [rad/s].
     * \param[in] max_change     Maximum change [rad/s].
     */
    JointFeedbackChecker(
        const double settling_time = 1.0,
        const double error_band = 1e-1,
        const double error_estimate = 1e-2,
        const double max_change = 10.0)
      : settling_time_(settling_time)
      , error_band_(error_band)
      , error_estimate_(error_estimate)
      , max_change_(max_change)
    {
    }

    /**
     * \brief Settling time getter.
     * \return Settling time [s].
     */
    double getSettlingTime() const
    {
      return settling_time_;
    }

    /**
     * \brief Error band getter.
     * \return Error band [rad/s].
     */
    double getErrorBand() const
    {
      return error_band_;
    }

    /**
     * \brief Error estimate getter.
     * \return Error estimate [rad/s].
     */
    double getErrorEstimate() const
    {
      return error_estimate_;
    }

    /**
     * \brief Maximum change getter.
     * \return Maximum change [rad/s].
     */
    double getMaxChange() const
    {
      return max_change_;
    }

    /**
     * \brief Settling time setter.
     * \param[in] settling_time Settling time [s].
     */
    void setSettlingTime(const double settling_time)
    {
      settling_time_ = settling_time;
    }

    /**
     * \brief Error band setter.
     * \param[in] error_band Error band [rad/s].
     */
    void setErrorBand(const double error_band)
    {
      error_band_ = error_band;
    }

    /**
     * \brief Error estimate setter.
     * \param[in] error_estimate Error estimate [rad/s].
     */
    void setErrorEstimate(const double error_estimate)
    {
      error_estimate_ = error_estimate;
    }

    /**
     * \brief Maximum change setter.
     * \param[in] max_change Maximum change [rad/s].
     */
    void setMaxChange(const double max_change)
    {
      max_change_ = max_change;
    }

    /**
     * \brief Check if the settling time has been met since the last time the
     * desired state was changed.
     * \param[in] dt Elapsed time since the last time the desired state was
     *               changed.
     * \return True, if the settling time has been met.
     */
    bool isSettled(const double dt) const
    {
      return dt > settling_time_;
    }

    /**
     * \brief Check if the actual state has converged to the desired one
     * \param[in] error Error between the desired and actual state, i.e.
     *                  error = desired - actual.
     * \param[in] dt      Elapsed time since the last time the desired state was
     *                    changed.
     * \return True, if the actual state has converged to the desired one within
     *         the error band after the settling time.
     *         If the settling time has NOT been met yet, also returns True.
     */
    bool hasConverged(const double error, const double dt) const
    {
      // Instead of:
      //
      //   if (isSettled(dt))
      //   {
      //     return std::abs(error) < error_band_;
      //   }
      //
      //   return true;
      //
      // we use the following simplified expression:
      return !isSettled(dt) || std::abs(error) < error_band_;
    }

    /**
     * \brief Check that the actual state doesn't jump wrt the previous one,
     * according to the maximum change.
     * \param[in] actual   Actual state.
     * \param[in] previous Previous actual state.
     * \param[in] dt       Elapsed time between previous and current actual
     *                     states.
     * \return True if the absolute difference between the actual and previous
     *         actual state (normalized by dt) is smaller than the maximum
     *         change.
     */
    bool isContinuous(
        const double actual, const double previous, const double dt) const
    {
      return std::abs(actual - previous) < max_change_ * dt;
    }

    /**
     * \brief Check that the actual state and the estimated one are NOT
     * discrepant, i.e. they're similar, so the absolute difference between them
     * is smaller than the error estimate threshold.
     * \param[in] actual           Actual state.
     * \param[in] actual_estimated Actual estimated state, e.g. velocity
     *                             estimated from position (by differentiation).
     * \return True if the absolute difference between the actual and the actual
     * estimated state is smaller than the error estimate threshold.
     */
    bool isDiscrepant(const double actual, const double actual_estimated) const
    {
      return std::abs(actual - actual_estimated) > error_estimate_;
    }

  private:
    // Settling time [s]:
    double settling_time_;

    // Error band [rad/s]:
    double error_band_;

    // Error estimate [rad/s]:
    double error_estimate_;

    // Maximum change [rad/s]:
    double max_change_;
  };

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER_JOINT_FEEDBACK_CHECKER_H
