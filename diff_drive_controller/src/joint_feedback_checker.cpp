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

#include <diff_drive_controller/joint_feedback_checker.h>

#include <cmath>

namespace diff_drive_controller
{

  JointFeedbackChecker::JointFeedbackChecker(
    const double settling_time,
    const double error_band,
    const double error_estimate,
    const double max_change
  )
    : settling_time_(settling_time)
    , error_band_(error_band)
    , error_estimate_(error_estimate)
    , max_change_(max_change)
  {
  }

  bool JointFeedbackChecker::isSettled(const double dt) const
  {
    return dt > settling_time_;
  }

  bool JointFeedbackChecker::hasConverged(
      const double error, const double dt) const
  {
    if (isSettled(dt))
    {
      return std::abs(error) < error_band_;
    }

    return true;
  }

  bool JointFeedbackChecker::isContinuous(
      const double actual,
      const double previous,
      const double dt) const
  {
    return std::abs(actual - previous) < max_change_ * dt;
  }

  bool JointFeedbackChecker::isDiscrepant(
      const double actual,
      const double actual_estimated) const
  {
    return std::abs(actual - actual_estimated) > error_estimate_;
  }

}  // namespace diff_drive_controller
