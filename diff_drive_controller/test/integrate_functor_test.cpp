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

#include <diff_drive_controller/autodiff_integrate_function.h>
#include <diff_drive_controller/analytic_integrate_function.h>

#include <diff_drive_controller/direct_kinematics_integrate_functor.h>

#include <diff_drive_controller/euler_integrate_functor.h>
#include <diff_drive_controller/runge_kutta_2_integrate_functor.h>
#include <diff_drive_controller/exact_integrate_functor.h>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

const double WHEEL_SEPARATION   = 0.2;  // [m]
const double LEFT_WHEEL_RADIUS  = 0.1;  // [m]
const double RIGHT_WHEEL_RADIUS = LEFT_WHEEL_RADIUS;  // [m]

/**
 * Test an integrate functor, which can be any of the following:
 * EulerIntegrateFunctor
 * RungeKutta2IntegrateFunctor
 * ExactIntegrateFunctor
 */
template <class Functor>
void testIntegrateFunctor(double& x, double& y, double& yaw,
    const double dp_l, const double dp_r)
{
  // Declare auto-diff and analytic integrator types:
  using namespace diff_drive_controller;

  typedef AutoDiffIntegrateFunction<DirectKinematicsIntegrateFunctor, Functor>
          AutoDiffIntegrator;
  typedef AnalyticIntegrateFunction<DirectKinematicsIntegrateFunctor, Functor>
          AnalyticIntegrator;

  // Create auto-diff and analytic integrators:
  AutoDiffIntegrator integrate_auto_diff;
  AnalyticIntegrator integrate_analytic;

  integrate_auto_diff.setWheelParams(WHEEL_SEPARATION,
          LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS);
  integrate_analytic.setWheelParams(WHEEL_SEPARATION,
          LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS);

  // Integrate with auto-diff and analytic integrators:
  const double x0   = x;
  const double y0   = y;
  const double yaw0 = yaw;

  IntegrateFunction::PoseJacobian J_pose_auto_diff;
  IntegrateFunction::MeasJacobian J_meas_auto_diff;

  integrate_auto_diff(x, y, yaw, dp_l, dp_r,
      J_pose_auto_diff, J_meas_auto_diff);

  const double x_auto_diff   = x;
  const double y_auto_diff   = y;
  const double yaw_auto_diff = yaw;

  x   = x0;
  y   = y0;
  yaw = yaw0;

  IntegrateFunction::PoseJacobian J_pose_analytic;
  IntegrateFunction::MeasJacobian J_meas_analytic;

  integrate_analytic(x, y, yaw, dp_l, dp_r,
      J_pose_analytic, J_meas_analytic);

  const double x_analytic   = x;
  const double y_analytic   = y;
  const double yaw_analytic = yaw;

  // Check the state is the same:
  EXPECT_EQ(x_auto_diff, x_analytic);
  EXPECT_EQ(y_auto_diff, y_analytic);
  EXPECT_EQ(yaw_auto_diff, yaw_analytic);

  // Check the Jacobians are the same:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  EXPECT_TRUE(((J_pose_auto_diff - J_pose_analytic).array().abs() < std::numeric_limits<double>::epsilon()).all())
    << "J_pose Auto-Diff =\n" << J_pose_auto_diff.format(HeavyFmt)
    << "\nJ_pose Analytic =\n" << J_pose_analytic.format(HeavyFmt)
    << "\n(J_pose Auto-Diff - J_pose Analytic) =\n" << (J_pose_auto_diff - J_pose_analytic).format(HeavyFmt);
  EXPECT_TRUE(((J_meas_auto_diff - J_meas_analytic).array().abs() < std::numeric_limits<double>::epsilon()).all())
    << "J_meas Auto-Diff =\n" << J_meas_auto_diff.format(HeavyFmt)
    << "\nJ_meas Analytic =\n" << J_meas_analytic.format(HeavyFmt)
    << "\n(J_meas Auto-Diff - J_meas Analytic) =\n" << (J_meas_auto_diff - J_meas_analytic).format(HeavyFmt);
}

/**
 * \brief Check y-axis turning clockwise.
 */
template <class Functor>
void checkYTurnClockwise(const double y, const double y0)
{
  // Check y-axis goes down turning clockwise:
  EXPECT_LT(y, y0);
}

template <>
void checkYTurnClockwise<diff_drive_controller::EulerIntegrateFunctor>(
    const double y, const double y0)
{
  // Check y-axis doesn't change with Euler:
  EXPECT_EQ(y, y0);
}

/**
 * \brief Check y-axis turning counter-clockwise.
 */
template <class Functor>
void checkYTurnCounterClockwise(const double y, const double y0)
{
  // Check y-axis goes up turning clockwise:
  EXPECT_GT(y, y0);
}

template <>
void checkYTurnCounterClockwise<diff_drive_controller::EulerIntegrateFunctor>(
    const double y, const double y0)
{
  // Check y-axis doesn't change with Euler:
  EXPECT_EQ(y, y0);
}

/**
 * \brief Test moving forward.
 */
template <class Functor>
void testForward()
{
  // Integrate:
  const double dp_l = 0.1;  // [rad]
  const double dp_r = dp_l;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is forward only:
  EXPECT_GT(x, x0);
  EXPECT_EQ(y, y0);
  EXPECT_EQ(yaw, yaw0);
}

/**
 * \brief Test moving backwards.
 */
template <class Functor>
void testBackwards()
{
  // Integrate:
  const double dp_l = -0.1;  // [rad]
  const double dp_r = dp_l;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is backwards only:
  EXPECT_LT(x, x0);
  EXPECT_EQ(y, y0);
  EXPECT_EQ(yaw, yaw0);
}

/**
 * \brief Test turning in-place clockwise.
 */
template <class Functor>
void testTurnInPlaceClockwise()
{
  // Integrate:
  const double dp_l = 0.1;  // [rad]
  const double dp_r = -dp_l;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is turning in place clockwise (yaw goes down) only:
  EXPECT_EQ(x, x0);
  EXPECT_EQ(y, y0);
  EXPECT_LT(yaw, yaw0);
}

/**
 * \brief Test turning in-place counter-clockwise.
 */
template <class Functor>
void testTurnInPlaceCounterClockwise()
{
  // Integrate:
  const double dp_l = -0.1;  // [rad]
  const double dp_r = -dp_l;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is turning in place clockwise (yaw goes up) only:
  EXPECT_EQ(x, x0);
  EXPECT_EQ(y, y0);
  EXPECT_GT(yaw, yaw0);
}

/**
 * \brief Test turning clockwise.
 */
template <class Functor>
void testTurnClockwise()
{
  // Integrate:
  const double dp_l = 0.2;  // [rad]
  const double dp_r = 0.1;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is turning clockwise (yaw goes down) only:
  EXPECT_GT(x, x0);
  checkYTurnClockwise<Functor>(y, y0);
  EXPECT_LT(yaw, yaw0);
}

/**
 * \brief Test turning counter-clockwise.
 */
template <class Functor>
void testTurnCounterClockwise()
{
  // Integrate:
  const double dp_l = 0.1;  // [rad]
  const double dp_r = 0.2;  // [rad]

  const double x0   = 0.0;
  const double y0   = 0.0;
  const double yaw0 = 0.0;

  double x   = x0;
  double y   = y0;
  double yaw = yaw0;

  testIntegrateFunctor<Functor>(x, y, yaw, dp_l, dp_r);

  // Check movement is turning clockwise (yaw goes up) only:
  EXPECT_GT(x, x0);
  checkYTurnCounterClockwise<Functor>(y, y0);
  EXPECT_GT(yaw, yaw0);
}

// Euler tests:
TEST(IntegrateFunctorTest, testEulerForward)
{
  testForward<diff_drive_controller::EulerIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testEulerBackwards)
{
  testBackwards<diff_drive_controller::EulerIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testEulerTurnInPlaceClockwise)
{
  testTurnInPlaceClockwise<diff_drive_controller::EulerIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testEulerTurnInPlaceCounterClockwise)
{
  testTurnInPlaceCounterClockwise<diff_drive_controller::EulerIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testEulerTurnClockwise)
{
  testTurnClockwise<diff_drive_controller::EulerIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testEulerTurnCounterClockwise)
{
  testTurnCounterClockwise<diff_drive_controller::EulerIntegrateFunctor>();
}

// RungeKutta2 tests:
TEST(IntegrateFunctorTest, testRungeKutta2Forward)
{
  testForward<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testRungeKutta2Backwards)
{
  testBackwards<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testRungeKutta2TurnInPlaceClockwise)
{
  testTurnInPlaceClockwise<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testRungeKutta2TurnInPlaceCounterClockwise)
{
  testTurnInPlaceCounterClockwise<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testRungeKutta2TurnClockwise)
{
  testTurnClockwise<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testRungeKutta2TurnCounterClockwise)
{
  testTurnCounterClockwise<diff_drive_controller::RungeKutta2IntegrateFunctor>();
}

// Exact tests:
TEST(IntegrateFunctorTest, testExactForward)
{
  testForward<diff_drive_controller::ExactIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testExactBackwards)
{
  testBackwards<diff_drive_controller::ExactIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testExactTurnInPlaceClockwise)
{
  testTurnInPlaceClockwise<diff_drive_controller::ExactIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testExactTurnInPlaceCounterClockwise)
{
  testTurnInPlaceCounterClockwise<diff_drive_controller::ExactIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testExactTurnClockwise)
{
  testTurnClockwise<diff_drive_controller::ExactIntegrateFunctor>();
}

TEST(IntegrateFunctorTest, testExactTurnCounterClockwise)
{
  testTurnCounterClockwise<diff_drive_controller::ExactIntegrateFunctor>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
