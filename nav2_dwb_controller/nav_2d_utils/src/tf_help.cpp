/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include "nav_2d_utils/tf_help.hpp"

namespace nav_2d_utils
{

bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & /*transform_tolerance*/
)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }
  rclcpp::Rate rate(10.0);
  std::string warning_msg;
  while (rclcpp::ok() && !tf->canTransform(
      frame,
      in_pose.header.frame_id, tf2::TimePoint(), &warning_msg))
  {
    RCLCPP_INFO(
      rclcpp::get_logger("tf_help"),
      "Waiting for transform %s ->  %s: %s",
      frame.c_str(), in_pose.header.frame_id.c_str(), warning_msg.c_str());
    rate.sleep();
  }
  try {
    geometry_msgs::msg::TransformStamped echo_transform;
    echo_transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePoint());
    tf2::doTransform(in_pose, out_pose, echo_transform);
    auto translation = echo_transform.transform.translation;
    auto rotation = echo_transform.transform.rotation;
    std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " <<
      translation.z << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " <<
      rotation.z << ", " << rotation.w << "]" << std::endl;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
  }
  return true;
}


bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const nav_2d_msgs::msg::Pose2DStamped & in_pose,
  nav_2d_msgs::msg::Pose2DStamped & out_pose,
  rclcpp::Duration & transform_tolerance
)
{
  geometry_msgs::msg::PoseStamped in_3d_pose = pose2DToPoseStamped(in_pose);
  geometry_msgs::msg::PoseStamped out_3d_pose;

  bool ret = transformPose(tf, frame, in_3d_pose, out_3d_pose, transform_tolerance);
  if (ret) {
    out_pose = poseStampedToPose2D(out_3d_pose);
  }
  return ret;
}
}  // namespace nav_2d_utils
