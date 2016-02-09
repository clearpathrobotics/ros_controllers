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

#include <diff_drive_controller/covariance.h>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

const double CONDITION_NUMBER_DESIRED = 10.0;

const double MATRIX_HIGH_DIAGONAL_ELEMENT = 1e6;

TEST(CovarianceTest, testChangeConditionNumber)
{
  // Create positive semi-definite matrix:
  Eigen::Matrix3d M = Eigen::Matrix3d::Random();
  M = 0.5 * (M + M.transpose());

  // Make condition number high:
  M(0, 0) = MATRIX_HIGH_DIAGONAL_ELEMENT;

  // Compute condition number:
  using namespace diff_drive_controller;

  const double condition_number = conditionNumber(M);

  EXPECT_LT(CONDITION_NUMBER_DESIRED, condition_number);

  // Change condition number:
  changeConditionNumber(M, CONDITION_NUMBER_DESIRED);

  // Compute and check the new condition number is equal to the desired one:
  const double condition_number_changed = conditionNumber(M);

  EXPECT_LT(condition_number_changed, condition_number);
  EXPECT_NEAR(CONDITION_NUMBER_DESIRED, condition_number_changed, 1e-14);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
