///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, Clearpath Robotics Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

/// \author Enrique Fernandez

#include "gtest_common.h"

#include <diff_drive_controller/joint_feedback_checker.h>

#include <gtest/gtest.h>

const double SETTLING_TIME = 1.0;
const double ERROR_BAND = 1e-1;
const double ERROR_ESTIMATE = 1e-2;
const double MAX_CHANGE = 10.0;

TEST(JointFeedbackCheckerTest, testBeforeSettledInsideErrorBand)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt    = 0.5 * SETTLING_TIME;
  const double error = 0.5 * ERROR_BAND;

  // Check settled and convergence:
  EXPECT_FALSE(checker.isSettled(dt));
  EXPECT_TRUE(checker.hasConverged(error, dt));
}

TEST(JointFeedbackCheckerTest, testBeforeSettledOutsideErrorBand)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt    = 0.5 * SETTLING_TIME;
  const double error = 1.5 * ERROR_BAND;

  // Check settled and convergence:
  EXPECT_FALSE(checker.isSettled(dt));
  EXPECT_TRUE(checker.hasConverged(error, dt));
}

TEST(JointFeedbackCheckerTest, testAfterSettledInsideErrorBand)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt    = 1.5 * SETTLING_TIME;
  const double error = 0.5 * ERROR_BAND;

  // Check settled and convergence:
  EXPECT_TRUE(checker.isSettled(dt));
  EXPECT_TRUE(checker.hasConverged(error, dt));
}

TEST(JointFeedbackCheckerTest, testAfterSettledOutsideErrorBand)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt    = 1.5 * SETTLING_TIME;
  const double error = 1.5 * ERROR_BAND;

  // Check settled and convergence:
  EXPECT_TRUE(checker.isSettled(dt));
  EXPECT_FALSE(checker.hasConverged(error, dt));
}

TEST(JointFeedbackCheckerTest, testContinous)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt = 0.01;
  const double actual = 1.0;
  const double previous = actual + (0.5 * MAX_CHANGE) * dt;

  // Check continuity:
  EXPECT_TRUE(checker.isContinuous(actual, previous, dt));
}

TEST(JointFeedbackCheckerTest, testNotContinous)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double dt = 0.01;
  const double actual = 1.0;
  const double previous = actual + (1.5 * MAX_CHANGE) * dt;

  // Check continuity:
  EXPECT_FALSE(checker.isContinuous(actual, previous, dt));
}

TEST(JointFeedbackCheckerTest, testEstimatedNotDiscrepant)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double actual = 1.0;
  const double actual_estimated = actual + 0.5 * ERROR_ESTIMATE;

  // Check discrepancy:
  EXPECT_FALSE(checker.isDiscrepant(actual, actual_estimated));
}

TEST(JointFeedbackCheckerTest, testEstimatedDiscrepant)
{
  // Create joint feedback checker:
  diff_drive_controller::JointFeedbackChecker checker(
      SETTLING_TIME, ERROR_BAND, ERROR_ESTIMATE, MAX_CHANGE);

  // Simulate dt and error:
  const double actual = 1.0;
  const double actual_estimated = actual + 1.5 * ERROR_ESTIMATE;

  // Check discrepancy:
  EXPECT_TRUE(checker.isDiscrepant(actual, actual_estimated));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
