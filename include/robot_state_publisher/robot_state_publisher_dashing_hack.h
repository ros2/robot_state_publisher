/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_DASHING_HACK_H_
#define ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_DASHING_HACK_H_

#include "robot_state_publisher/robot_state_publisher.h"

namespace robot_state_publisher
{
// Hack for Dashing patch release
// This makes timestamps on static transforms obey use_sim_time in Dashing
class RobotStatePublisherDashingHack : public RobotStatePublisher
{
public:
  RobotStatePublisherDashingHack(
    rclcpp::Node::SharedPtr node_handle, const KDL::Tree & tree,
    const urdf::Model & model, const std::string & model_xml)
    : RobotStatePublisher(node_handle, tree, model, model_xml),
    clock_(node_handle->get_clock())
  {
  }

  virtual ~RobotStatePublisherDashingHack()
  {
  }

  // Copy/pasted from robot_state_publisher.cpp
  inline
  geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame & k)
  {
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = k.p.x();
    t.transform.translation.y = k.p.y();
    t.transform.translation.z = k.p.z();
    k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
      t.transform.rotation.w);
    return t;
  }

  // Mostly copy/pasted from robot_state_publisher.cpp
  void publishFixedTransforms(const std::string & tf_prefix, bool use_tf_static = false) override
  {
    std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
    geometry_msgs::msg::TransformStamped tf_transform;

    // loop over all fixed segments
    for (auto seg = segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
      geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg->second.segment.pose(0));
      rclcpp::Time now = clock_->now();
      if (!use_tf_static) {
        now = now + rclcpp::Duration(std::chrono::milliseconds(500));  // 0.5sec in NS
      }
      tf_transform.header.stamp = now;

      tf_transform.header.frame_id = tf_prefix + "/" + seg->second.root;
      tf_transform.child_frame_id = tf_prefix + "/" + seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
    if (use_tf_static) {
      static_tf_broadcaster_.sendTransform(tf_transforms);
    } else {
      tf_broadcaster_.sendTransform(tf_transforms);
    }
  }

private:
  rclcpp::Clock::SharedPtr clock_;
};
}
#endif
