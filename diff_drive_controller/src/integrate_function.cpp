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

#include <diff_drive_controller/integrate_function.h>

#include <diff_drive_controller/autodiff_integrate_function.h>
#include <diff_drive_controller/analytic_integrate_function.h>

#include <diff_drive_controller/direct_kinematics_integrate_functor.h>

#include <diff_drive_controller/euler_integrate_functor.h>
#include <diff_drive_controller/runge_kutta_2_integrate_functor.h>
#include <diff_drive_controller/exact_integrate_functor.h>

#include <boost/make_shared.hpp>

namespace diff_drive_controller
{

IntegrateFunction::Ptr IntegrateFunction::create(const std::string& method,
    const std::string& differentiation)
{
  typedef AnalyticIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    EulerIntegrateFunctor>
          AnalyticEulerIntegrateFunction;
  typedef AnalyticIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    RungeKutta2IntegrateFunctor>
          AnalyticRungeKutta2IntegrateFunction;
  typedef AnalyticIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    ExactIntegrateFunctor>
          AnalyticExactIntegrateFunction;

  typedef AutoDiffIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    EulerIntegrateFunctor>
          AutoDiffEulerIntegrateFunction;
  typedef AutoDiffIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    RungeKutta2IntegrateFunctor>
          AutoDiffRungeKutta2IntegrateFunction;
  typedef AutoDiffIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    ExactIntegrateFunctor>
          AutoDiffExactIntegrateFunction;

  if (method == "euler")
  {
    if (differentiation == "analytic")
    {
      return std::allocate_shared<AnalyticEulerIntegrateFunction>(
          Eigen::aligned_allocator<AnalyticEulerIntegrateFunction>());
    }
    else if (differentiation == "autodiff")
    {
      return std::allocate_shared<AutoDiffEulerIntegrateFunction>(
          Eigen::aligned_allocator<AutoDiffEulerIntegrateFunction>());
    }
  }
  else if (method == "rungekutta2")
  {
    if (differentiation == "analytic")
    {
      return std::allocate_shared<AnalyticRungeKutta2IntegrateFunction>(
          Eigen::aligned_allocator<AnalyticRungeKutta2IntegrateFunction>());
    }
    else if (differentiation == "autodiff")
    {
      return std::allocate_shared<AutoDiffRungeKutta2IntegrateFunction>(
          Eigen::aligned_allocator<AutoDiffRungeKutta2IntegrateFunction>());
    }
  }
  else if (method == "exact")
  {
    if (differentiation == "analytic")
    {
      return std::allocate_shared<AnalyticExactIntegrateFunction>(
          Eigen::aligned_allocator<AnalyticExactIntegrateFunction>());
    }
    else if (differentiation == "autodiff")
    {
      return std::allocate_shared<AutoDiffExactIntegrateFunction>(
          Eigen::aligned_allocator<AutoDiffExactIntegrateFunction>());
    }
  }

  return Ptr();
}

}  // namespace diff_drive_controller

