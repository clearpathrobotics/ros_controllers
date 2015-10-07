/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 * Author: Bence Magyar
 * Author: Enrique Fernández
 */

#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <diff_drive_controller/covariance.h>
#include <diff_drive_controller/diff_drive_controller.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x, 2) +
                   std::pow(vec1.y-vec2.y, 2) +
                   std::pow(vec1.z-vec2.z, 2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have collision description. "
        "Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have collision geometry description. "
        "Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(
    const boost::shared_ptr<const urdf::Link>& wheel_link,
    double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name <<
        " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(
        wheel_link->collision->geometry.get()))->radius;
  return true;
}

namespace diff_drive_controller
{

  DiffDriveController::DiffDriveController()
    : open_loop_(false)
    , position_feedback_(true)
    , command_struct_()
    , dynamic_params_struct_()
    , wheel_separation_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_multiplier_(1.0)
    , left_wheel_radius_multiplier_(1.0)
    , right_wheel_radius_multiplier_(1.0)
    , k_l_(1.0)
    , k_r_(1.0)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , wheel_joints_size_(0)
    , publish_cmd_vel_limited_(false)
    , publish_state_(false)
  {
  }

  bool DiffDriveController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_wheel_names, right_wheel_names;
    if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
        !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
    {
      return false;
    }

    if (left_wheel_names.size() != right_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#left wheels (" << left_wheel_names.size() << ") != " <<
          "#right wheels (" << right_wheel_names.size() << ").");
      return false;
    }
    else
    {
      wheel_joints_size_ = left_wheel_names.size();

      left_wheel_joints_.resize(wheel_joints_size_);
      right_wheel_joints_.resize(wheel_joints_size_);
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry will be computed in "
                          << (open_loop_ ? "open" : "close") << " loop.");

    controller_nh.param("position_feedback", position_feedback_, position_feedback_);
    ROS_DEBUG_STREAM_COND_NAMED(!open_loop_, name_,
        "Odometry will be computed using the wheel " <<
        (position_feedback_ ? "postion" : "velocity") << " feedback.");

    controller_nh.param("wheel_separation_multiplier",
        wheel_separation_multiplier_, wheel_separation_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by "
                          << wheel_separation_multiplier_ << ".");

    controller_nh.param("left_wheel_radius_multiplier",
        left_wheel_radius_multiplier_, left_wheel_radius_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Left wheel radius will be multiplied by "
                          << left_wheel_radius_multiplier_ << ".");

    controller_nh.param("right_wheel_radius_multiplier",
        right_wheel_radius_multiplier_, right_wheel_radius_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Right wheel radius will be multiplied by "
                          << right_wheel_radius_multiplier_ << ".");

    controller_nh.param("k_l", k_l_, k_l_);
    controller_nh.param("k_r", k_r_, k_r_);

    if (k_l_ <= 0.0)
    {
      k_l_ = std::abs(k_l_);
      ROS_ERROR_STREAM_NAMED(name_,
          "Left Measurement Covariance multiplier must be positive! "
          "Taking absolute value: " << k_l_ << ".");
    }

    if (k_r_ <= 0.0)
    {
      k_r_ = std::abs(k_r_);
      ROS_ERROR_STREAM_NAMED(name_,
          "Right Measurement Covariance multiplier must be positive! "
          "Taking absolute value: " << k_r_ << ".");
    }

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_,
        "Velocity commands will be considered old if they are older than "
        << cmd_vel_timeout_ << "s.");

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_,
        "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    controller_nh.param("publish_cmd_vel_limited", publish_cmd_vel_limited_, publish_cmd_vel_limited_);
    ROS_INFO_STREAM_NAMED(name_,
        "Publishing the limited velocity command is "
        << (publish_cmd_vel_limited_?"enabled":"disabled"));

    controller_nh.param("publish_state", publish_state_, publish_state_);
    ROS_INFO_STREAM_NAMED(name_,
        "Publishing the joint trajectory controller state is "
        << (publish_state_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits"        , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/max_jerk"               , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
    controller_nh.param("linear/x/min_jerk"               , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
    controller_nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh.getParam("wheel_separation", wheel_separation_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!setOdomParamsFromUrdf(root_nh,
                               left_wheel_names[0],
                               right_wheel_names[0],
                               lookup_wheel_separation,
                               lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double wrl = left_wheel_radius_multiplier_  * wheel_radius_;
    const double wrr = right_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, wrl, wrr);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation " << ws
                          << ", left wheel radius "  << wrl
                          << ", right wheel radius " << wrr);

    odometry_.setMeasCovarianceParams(k_l_, k_r_);
    ROS_INFO_STREAM_NAMED(name_,
                          "Measurement Covariance Model params : k_l " << k_l_
                          << ", k_r " << k_r_);

    dynamic_params_struct_.wheel_separation_multiplier = wheel_separation_multiplier_;

    dynamic_params_struct_.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    dynamic_params_struct_.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;

    dynamic_params_struct_.k_l = k_l_;
    dynamic_params_struct_.k_r = k_r_;

    dynamic_params_struct_.publish_state = publish_state_;
    dynamic_params_struct_.publish_cmd_vel_limited = publish_cmd_vel_limited_;

    dynamic_params_.writeFromNonRT(dynamic_params_struct_);

    setOdomPubFields(root_nh, controller_nh);

    // Set dynamic reconfigure server callback:
    cfg_server_.reset(new ReconfigureServer(controller_nh));
    cfg_server_->setCallback(
        boost::bind(&DiffDriveController::reconfigureCallback, this, _1, _2));

    // Limited velocity command:
    cmd_vel_limited_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_limited", 100));

    cmd_vel_limited_pub_->msg_.header.frame_id = base_frame_id_;

    cmd_vel_limited_pub_->msg_.twist.linear.y = 0.0;
    cmd_vel_limited_pub_->msg_.twist.linear.z = 0.0;

    cmd_vel_limited_pub_->msg_.twist.angular.x = 0.0;
    cmd_vel_limited_pub_->msg_.twist.angular.y = 0.0;

    // Joint trajectory controller state:
    state_pub_.reset(new realtime_tools::RealtimePublisher<DiffDriveControllerState>(controller_nh, "state", 100));

    state_pub_->msg_.header.frame_id = base_frame_id_;

    state_pub_->msg_.joint_names.resize(2 * wheel_joints_size_);

    state_pub_->msg_.desired.positions.resize(2 * wheel_joints_size_);
    state_pub_->msg_.desired.velocities.resize(2 * wheel_joints_size_);
    state_pub_->msg_.desired.accelerations.resize(2 * wheel_joints_size_);
    state_pub_->msg_.desired.effort.resize(2 * wheel_joints_size_);

    state_pub_->msg_.actual.positions.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual.velocities.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual.accelerations.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual.effort.resize(2 * wheel_joints_size_);

    state_pub_->msg_.error.positions.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error.velocities.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error.accelerations.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error.effort.resize(2 * wheel_joints_size_);

    state_pub_->msg_.actual_estimated.positions.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual_estimated.velocities.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual_estimated.accelerations.resize(2 * wheel_joints_size_);
    state_pub_->msg_.actual_estimated.effort.resize(2 * wheel_joints_size_);

    state_pub_->msg_.error_estimated.positions.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error_estimated.velocities.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error_estimated.accelerations.resize(2 * wheel_joints_size_);
    state_pub_->msg_.error_estimated.effort.resize(2 * wheel_joints_size_);

    state_pub_->msg_.actual_side_average.positions.resize(2);
    state_pub_->msg_.actual_side_average.velocities.resize(2);
    state_pub_->msg_.actual_side_average.accelerations.resize(2);
    state_pub_->msg_.actual_side_average.effort.resize(2);

    state_pub_->msg_.error_side_average.positions.resize(2);
    state_pub_->msg_.error_side_average.velocities.resize(2);
    state_pub_->msg_.error_side_average.accelerations.resize(2);
    state_pub_->msg_.error_side_average.effort.resize(2);

    state_pub_->msg_.actual_estimated_side_average.positions.resize(2);
    state_pub_->msg_.actual_estimated_side_average.velocities.resize(2);
    state_pub_->msg_.actual_estimated_side_average.accelerations.resize(2);
    state_pub_->msg_.actual_estimated_side_average.effort.resize(2);

    state_pub_->msg_.error_estimated_side_average.positions.resize(2);
    state_pub_->msg_.error_estimated_side_average.velocities.resize(2);
    state_pub_->msg_.error_estimated_side_average.accelerations.resize(2);
    state_pub_->msg_.error_estimated_side_average.effort.resize(2);

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      state_pub_->msg_.joint_names[i] = left_wheel_names[i];
      state_pub_->msg_.joint_names[i + wheel_joints_size_] = right_wheel_names[i];
    }

    left_positions_.resize(wheel_joints_size_);
    right_positions_.resize(wheel_joints_size_);

    left_velocities_.resize(wheel_joints_size_);
    right_velocities_.resize(wheel_joints_size_);

    left_positions_estimated_.resize(wheel_joints_size_, 0.0);
    right_positions_estimated_.resize(wheel_joints_size_, 0.0);

    left_velocities_estimated_.resize(wheel_joints_size_);
    right_velocities_estimated_.resize(wheel_joints_size_);

    left_positions_previous_.resize(wheel_joints_size_, 0.0);
    right_positions_previous_.resize(wheel_joints_size_, 0.0);

    left_velocities_previous_.resize(wheel_joints_size_, 0.0);
    right_velocities_previous_.resize(wheel_joints_size_, 0.0);

    left_velocities_estimated_previous_.resize(wheel_joints_size_, 0.0);
    right_velocities_estimated_previous_.resize(wheel_joints_size_, 0.0);

    left_velocity_command_previous_ = 0.0;
    right_velocity_command_previous_ = 0.0;

    // Get the joint object to use in the realtime loop
    for (int i = 0; i < wheel_joints_size_; ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left wheel with joint name: " << left_wheel_names[i]
                            << " and right wheel with joint name: " << right_wheel_names[i]);
      left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);  // throws on failure
      right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);

    return true;
  }

  void DiffDriveController::update(const ros::Time& time, const ros::Duration& period)
  {
    // UPDATE DYNAMIC PARAMS
    // Retreive dynamic params:
    DynamicParams dynamic_params = *(dynamic_params_.readFromRT());

    // Update dynamic params:
    //
    // NOTE we cannot do the following because there's no writeFromRT method!
    // see https://github.com/ros-controls/realtime_tools/issues/14
    //
    // if (dynamic_params.changed)
    // {
    //   dynamic_params.changed = false;
    //   dynamic_params_.writeFromRT(dynamic_params);
    //
    //   ...
    // }
    wheel_separation_multiplier_ = dynamic_params.wheel_separation_multiplier;
    left_wheel_radius_multiplier_ = dynamic_params.left_wheel_radius_multiplier;
    right_wheel_radius_multiplier_ = dynamic_params.right_wheel_radius_multiplier;

    k_l_ = dynamic_params.k_l;
    k_r_ = dynamic_params.k_r;

    publish_state_ = dynamic_params.publish_state;
    publish_cmd_vel_limited_ = dynamic_params.publish_cmd_vel_limited;

    // Apply multipliers:
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double wrl = left_wheel_radius_multiplier_  * wheel_radius_;
    const double wrr = right_wheel_radius_multiplier_ * wheel_radius_;

    // Set the odometry parameters:
    odometry_.setWheelParams(ws, wrl, wrr);
    odometry_.setMeasCovarianceParams(k_l_, k_r_);

    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
      if (position_feedback_)
      {
        double left_pos  = 0.0;
        double right_pos = 0.0;
        for (size_t i = 0; i < wheel_joints_size_; ++i)
        {
          const double lp = left_wheel_joints_[i].getPosition();
          const double rp = right_wheel_joints_[i].getPosition();
          if (std::isnan(lp) || std::isnan(rp))
            return;

          left_pos  += lp;
          right_pos += rp;
        }
        left_pos  /= wheel_joints_size_;
        right_pos /= wheel_joints_size_;

        // Estimate linear and angular velocity using joint position information
        odometry_.updateCloseLoop(left_pos, right_pos, time);
      }
      else
      {
        double left_vel  = 0.0;
        double right_vel = 0.0;
        for (size_t i = 0; i < wheel_joints_size_; ++i)
        {
          const double lv = left_wheel_joints_[i].getVelocity();
          const double rv = right_wheel_joints_[i].getVelocity();
          if (std::isnan(lv) || std::isnan(rv))
            return;

          left_vel  += lv;
          right_vel += rv;
        }
        left_vel  /= wheel_joints_size_;
        right_vel /= wheel_joints_size_;

        // Estimate linear and angular velocity using joint velocity information
        odometry_.updateCloseLoopFromVelocity(left_vel, right_vel, time);
      }
    }

    // Publish odometry message
    const ros::Duration half_period(0.5 * period.toSec());
    if (last_odom_publish_time_ + publish_period_ < time + half_period)
    {
      last_odom_publish_time_ = time;

      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;

        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getVx();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getVy();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getVyaw();

        covarianceToMsg(odometry_.getPoseCovariance() , odom_pub_->msg_.pose.covariance);
        covarianceToMsg(odometry_.getTwistCovariance(), odom_pub_->msg_.twist.covariance);

        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin = 0.0;
      curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Compute wheels velocities:
    const double vel_left  = (curr_cmd.lin - curr_cmd.ang * ws / 2.0)/wrl;
    const double vel_right = (curr_cmd.lin + curr_cmd.ang * ws / 2.0)/wrr;

    // Set wheels velocities:
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel_left);
      right_wheel_joints_[i].setCommand(vel_right);
    }

    // Publish limited velocity command:
    if (dynamic_params.publish_cmd_vel_limited && cmd_vel_limited_pub_->trylock())
    {
      cmd_vel_limited_pub_->msg_.header.stamp = time;

      cmd_vel_limited_pub_->msg_.twist.linear.x  = curr_cmd.lin;
      cmd_vel_limited_pub_->msg_.twist.angular.z = curr_cmd.ang;

      cmd_vel_limited_pub_->unlockAndPublish();
    }

    // Publish joint trajectory controller state:
    if (publish_state_)
    {
      if (state_pub_->trylock())
      {
        state_pub_->msg_.header.stamp = time;

        // Set left wheel joints desired, actual (and actual estimated) state:
        for (size_t i = 0; i < wheel_joints_size_; ++i)
        {
          // Desired state:
          state_pub_->msg_.desired.accelerations[i] = (left_velocity_command - left_velocity_command_previous_) / period.toSec();
          state_pub_->msg_.desired.velocities[i] = left_velocity_command;
          state_pub_->msg_.desired.positions[i] += left_velocity_command * period.toSec();
          state_pub_->msg_.desired.effort[i] = std::numeric_limits<double>::quiet_NaN();

          // Actual state:
          const double left_acceleration = (left_velocities_[i] - left_velocities_previous_[i]) / period.toSec();

          state_pub_->msg_.actual.accelerations[i] = left_acceleration;
          state_pub_->msg_.actual.velocities[i] = left_velocities_[i];
          state_pub_->msg_.actual.positions[i] = left_positions_[i];
          state_pub_->msg_.actual.effort[i] = left_wheel_joints_[i].getEffort();

          // Actual estimated state:
          const double left_acceleration_estimated = (left_velocities_estimated_[i] - left_velocities_estimated_previous_[i]) / period.toSec();

          state_pub_->msg_.actual_estimated.accelerations[i] = left_acceleration_estimated;
          state_pub_->msg_.actual_estimated.velocities[i] = left_velocities_estimated_[i];
          state_pub_->msg_.actual_estimated.positions[i] = left_positions_estimated_[i];
          state_pub_->msg_.actual_estimated.effort[i] = state_pub_->msg_.actual.effort[i];
        }

        // Set right wheel joints desired, actual (and actual estimated) state:
        for (size_t i = 0, j = wheel_joints_size_; i < wheel_joints_size_; ++i, ++j)
        {
          // Desired state:
          state_pub_->msg_.desired.accelerations[j] = (right_velocity_command - right_velocity_command_previous_) / period.toSec();
          state_pub_->msg_.desired.velocities[j] = right_velocity_command;
          state_pub_->msg_.desired.positions[j] += right_velocity_command * period.toSec();
          state_pub_->msg_.desired.effort[j] = std::numeric_limits<double>::quiet_NaN();

          // Actual state:
          const double right_acceleration = (right_velocities_[i] - right_velocities_previous_[i]) / period.toSec();

          state_pub_->msg_.actual.accelerations[j] = right_acceleration;
          state_pub_->msg_.actual.velocities[j] = right_velocities_[i];
          state_pub_->msg_.actual.positions[j] = right_positions_[i];
          state_pub_->msg_.actual.effort[j] = right_wheel_joints_[i].getEffort();

          // Actual estimated state:
          const double right_acceleration_estimated = (right_velocities_estimated_[i] - right_velocities_estimated_previous_[i]) / period.toSec();

          state_pub_->msg_.actual_estimated.accelerations[j] = right_acceleration_estimated;
          state_pub_->msg_.actual_estimated.velocities[j] = right_velocities_estimated_[i];
          state_pub_->msg_.actual_estimated.positions[j] = right_positions_estimated_[i];
          state_pub_->msg_.actual_estimated.effort[j] = state_pub_->msg_.actual.effort[j];
        }

        // Set left wheel joints actual (and actual estimated) side average
        // state:
        const double left_acceleration_average = (left_velocity_average - left_velocity_average_previous_) / period.toSec();

        state_pub_->msg_.actual_side_average.accelerations[0] = left_acceleration_average;
        state_pub_->msg_.actual_side_average.velocities[0] = left_velocity_average;
        state_pub_->msg_.actual_side_average.positions[0] = left_position_average;
        state_pub_->msg_.actual_side_average.effort[0] = state_pub_->msg_.actual.effort[0];

        const double left_acceleration_estimated_average = (left_velocity_estimated_average - left_velocity_estimated_average_previous_) / period.toSec();

        state_pub_->msg_.actual_estimated_side_average.accelerations[0] = left_acceleration_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.velocities[0] = left_velocity_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.positions[0] = left_position_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.effort[0] = state_pub_->msg_.actual.effort[0];

        // Set right wheel joints actual (and actual estimated) side average
        // state:
        const double right_acceleration_average = (right_velocity_average - right_velocity_average_previous_) / period.toSec();

        state_pub_->msg_.actual_side_average.accelerations[1] = right_acceleration_average;
        state_pub_->msg_.actual_side_average.velocities[1] = right_velocity_average;
        state_pub_->msg_.actual_side_average.positions[1] = right_position_average;
        state_pub_->msg_.actual_side_average.effort[1] = state_pub_->msg_.actual.effort[wheel_joints_size_];

        const double right_acceleration_estimated_average = (right_velocity_estimated_average - right_velocity_estimated_average_previous_) / period.toSec();

        state_pub_->msg_.actual_estimated_side_average.accelerations[1] = right_acceleration_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.velocities[1] = right_velocity_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.positions[1] = right_position_estimated_average;
        state_pub_->msg_.actual_estimated_side_average.effort[1] = state_pub_->msg_.actual.effort[wheel_joints_size_];

        for (size_t i = 0; i < 2 * wheel_joints_size_; ++i)
        {
          state_pub_->msg_.error.positions[i] =
            state_pub_->msg_.desired.positions[i] - state_pub_->msg_.actual.positions[i];
          state_pub_->msg_.error.velocities[i] =
            state_pub_->msg_.desired.velocities[i] - state_pub_->msg_.actual.velocities[i];
          state_pub_->msg_.error.accelerations[i] =
            state_pub_->msg_.desired.accelerations[i] - state_pub_->msg_.actual.accelerations[i];
          state_pub_->msg_.error.effort[i] =
            state_pub_->msg_.desired.effort[i] - state_pub_->msg_.actual.effort[i];

          state_pub_->msg_.error_estimated.positions[i] =
            state_pub_->msg_.desired.positions[i] - state_pub_->msg_.actual_estimated.positions[i];
          state_pub_->msg_.error_estimated.velocities[i] =
            state_pub_->msg_.desired.velocities[i] - state_pub_->msg_.actual_estimated.velocities[i];
          state_pub_->msg_.error_estimated.accelerations[i] =
            state_pub_->msg_.desired.accelerations[i] - state_pub_->msg_.actual_estimated.accelerations[i];
          state_pub_->msg_.error_estimated.effort[i] =
            state_pub_->msg_.desired.effort[i] - state_pub_->msg_.actual_estimated.effort[i];

          state_pub_->msg_.error_estimated.positions[i] =
            state_pub_->msg_.desired.positions[i] - state_pub_->msg_.actual_estimated.positions[i];
          state_pub_->msg_.error_estimated.velocities[i] =
            state_pub_->msg_.desired.velocities[i] - state_pub_->msg_.actual_estimated.velocities[i];
          state_pub_->msg_.error_estimated.accelerations[i] =
            state_pub_->msg_.desired.accelerations[i] - state_pub_->msg_.actual_estimated.accelerations[i];
          state_pub_->msg_.error_estimated.effort[i] =
            state_pub_->msg_.desired.effort[i] - state_pub_->msg_.actual_estimated.effort[i];

          state_pub_->msg_.error_estimated_side_average.positions[i] =
            state_pub_->msg_.desired.positions[i] - state_pub_->msg_.actual_estimated_side_average.positions[i];
          state_pub_->msg_.error_estimated_side_average.velocities[i] =
            state_pub_->msg_.desired.velocities[i] - state_pub_->msg_.actual_estimated_side_average.velocities[i];
          state_pub_->msg_.error_estimated_side_average.accelerations[i] =
            state_pub_->msg_.desired.accelerations[i] - state_pub_->msg_.actual_estimated_side_average.accelerations[i];
          state_pub_->msg_.error_estimated_side_average.effort[i] =
            state_pub_->msg_.desired.effort[i] - state_pub_->msg_.actual_estimated_side_average.effort[i];
        }

        state_pub_->msg_.desired.time_from_start = ros::Duration(dt);
        state_pub_->msg_.actual.time_from_start = period;
        state_pub_->msg_.error.time_from_start = state_pub_->msg_.actual.time_from_start;

        state_pub_->msg_.actual_estimated.time_from_start = state_pub_->msg_.actual.time_from_start;
        state_pub_->msg_.error_estimated.time_from_start = state_pub_->msg_.actual_estimated.time_from_start;

        state_pub_->msg_.actual_side_average.time_from_start = state_pub_->msg_.actual.time_from_start;
        state_pub_->msg_.error_side_average.time_from_start = state_pub_->msg_.actual_side_average.time_from_start;

        state_pub_->msg_.actual_estimated_side_average.time_from_start = state_pub_->msg_.actual.time_from_start;
        state_pub_->msg_.error_estimated_side_average.time_from_start = state_pub_->msg_.actual_estimated_side_average.time_from_start;

        state_pub_->unlockAndPublish();
      }

      // Save wheel joints positions and velocities; needed to estimate the
      // velocities and accelerations, respectively:
      // @todo part of this should be outside of the if (publish_state_)
      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        // Left wheel joints:
        left_positions_previous_[i] = left_positions_[i];
        left_velocities_previous_[i] = left_velocities_[i];

        left_velocities_estimated_previous_[i] = left_velocities_estimated_[i];

        // Right wheel joints:
        right_positions_previous_[i] = right_positions_[i];
        right_velocities_previous_[i] = right_velocities_[i];

        right_velocities_estimated_previous_[i] = right_velocities_estimated_[i];
      }

      left_velocity_average_previous_ = left_velocity_average;
      right_velocity_average_previous_ = right_velocity_average;

      left_velocity_estimated_average_previous_ = left_velocity_estimated_average;
      right_velocity_estimated_average_previous_ = right_velocity_estimated_average;

      left_velocity_command_previous_ = left_velocity_command;
      right_velocity_command_previous_ = right_velocity_command;
    }
  }

  void DiffDriveController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_odom_publish_time_ = time;

    odometry_.init(time);
  }

  void DiffDriveController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void DiffDriveController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel);
      right_wheel_joints_[i].setCommand(vel);
    }
  }

  void DiffDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      command_struct_.ang   = command.angular.z;
      command_struct_.lin   = command.linear.x;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin: "   << command_struct_.lin << ", "
                             << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  void DiffDriveController::reconfigureCallback(
      DiffDriveControllerConfig& config, uint32_t level)
  {
    dynamic_params_struct_.wheel_separation_multiplier = config.wheel_separation_multiplier;

    dynamic_params_struct_.left_wheel_radius_multiplier  = config.left_wheel_radius_multiplier;
    dynamic_params_struct_.right_wheel_radius_multiplier = config.right_wheel_radius_multiplier;

    dynamic_params_struct_.k_l = config.k_l;
    dynamic_params_struct_.k_r = config.k_r;

    dynamic_params_struct_.publish_state = config.publish_state;
    dynamic_params_struct_.publish_cmd_vel_limited = config.publish_cmd_vel_limited;

    dynamic_params_.writeFromNonRT(dynamic_params_struct_);

    ROS_DEBUG_STREAM_NAMED(name_,
                          "Reconfigured Odometry params. "
                          << "wheel separation:   " << dynamic_params_struct_.wheel_separation_multiplier << ", "
                          << "left wheel radius:  " << dynamic_params_struct_.left_wheel_radius_multiplier << ", "
                          << "right wheel radius: " << dynamic_params_struct_.left_wheel_radius_multiplier);

    ROS_DEBUG_STREAM_NAMED(name_,
                          "Reconfigured Measurement Covariance Model params. "
                          << "k_l: " << dynamic_params_struct_.k_l << ", "
                          << "k_r: " << dynamic_params_struct_.k_r);

    ROS_DEBUG_STREAM_NAMED(name_,
                          "Reconfigured Debug Publishers params. "
                          << "state: " << ( dynamic_params_struct_.publish_state ? "ON" : "OFF" ) << ", "
                          << "cmd_vel_limited: " << ( dynamic_params_struct_.publish_cmd_vel_limited ? "ON" : "OFF" ));
  }

  bool DiffDriveController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }

  bool DiffDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                             const std::string& left_wheel_name,
                             const std::string& right_wheel_name,
                             bool lookup_wheel_separation,
                             bool lookup_wheel_radius)
  {
    if (!(lookup_wheel_separation || lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR_NAMED(name_,
          "Robot descripion couldn't be retrieved from param server.");
      return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));

    boost::shared_ptr<const urdf::Joint> left_wheel_joint(model->getJoint(left_wheel_name));
    boost::shared_ptr<const urdf::Joint> right_wheel_joint(model->getJoint(right_wheel_name));

    if (lookup_wheel_separation)
    {
      // Get wheel separation
      if (!left_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(name_, left_wheel_name
            << " couldn't be retrieved from model description");
        return false;
      }

      if (!right_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(name_, right_wheel_name
            << " couldn't be retrieved from model description");
        return false;
      }

      ROS_INFO_STREAM("left wheel to origin: "
          << left_wheel_joint->parent_to_joint_origin_transform.position.x << ","
          << left_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
          << left_wheel_joint->parent_to_joint_origin_transform.position.z);

      ROS_INFO_STREAM("right wheel to origin: "
          << right_wheel_joint->parent_to_joint_origin_transform.position.x << ","
          << right_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
          << right_wheel_joint->parent_to_joint_origin_transform.position.z);

      wheel_separation_ = euclideanOfVectors(
          left_wheel_joint->parent_to_joint_origin_transform.position,
          right_wheel_joint->parent_to_joint_origin_transform.position);
    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    }

    return true;
  }

  void DiffDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    /// Set odometry initial pose covariance
    XmlRpc::XmlRpcValue pose_cov_list;
    if (controller_nh.getParam("initial_pose_covariance_diagonal", pose_cov_list))
    {
      ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(pose_cov_list.size() == 3);
      for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      Eigen::Vector3d pose_covariance;
      pose_covariance << static_cast<double>(pose_cov_list[0]),
                         static_cast<double>(pose_cov_list[1]),
                         static_cast<double>(pose_cov_list[2]);
      odometry_.setPoseCovariance(pose_covariance.asDiagonal());

      ROS_INFO_STREAM("Pose covariance initialized to: " << pose_covariance);
    }

    /// Set odometry minimum twist covariance
    XmlRpc::XmlRpcValue twist_cov_list;
    if (controller_nh.getParam("minimum_twist_covariance_diagonal", twist_cov_list))
    {
      ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(twist_cov_list.size() == 3);
      for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      Eigen::Vector3d twist_covariance;
      twist_covariance << static_cast<double>(twist_cov_list[0]),
                          static_cast<double>(twist_cov_list[1]),
                          static_cast<double>(twist_cov_list[2]);
      odometry_.setMinimumTwistCovariance(twist_covariance.asDiagonal());

      ROS_INFO_STREAM("Minimum Twist covariance set to: " << twist_covariance);
    }

    /// Setup odometry message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id_;

    odom_pub_->msg_.pose.pose.position.z = 0;

    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;

    odom_pub_->msg_.pose.covariance.fill(0);
    odom_pub_->msg_.twist.covariance.fill(0);

    /// Setup odometry realtime publisher
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }

} // namespace diff_drive_controller
