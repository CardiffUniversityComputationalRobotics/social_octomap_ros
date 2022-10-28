/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011-2012.
 * @see http://www.ros.org/wiki/social_octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_ROS_CONVERSIONS_H
#define OCTOMAP_ROS_CONVERSIONS_H

#include <social_octomap/social_octomap.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

namespace social_octomap
{
  /**
   * @brief Conversion from social_octomap::point3d_list (e.g. all occupied nodes from getOccupied()) to
   * sensor_msgs::PointCloud2
   *
   * @param points
   * @param cloud
   */
  void pointsSocialOctomapToPointCloud2(const point3d_list &points, sensor_msgs::PointCloud2 &cloud);

  /**
   * @brief Conversion from a sensor_msgs::PointCLoud2 to social_octomap::Pointcloud, used internally in OctoMap
   *
   * @param cloud
   * @param social_octomapCloud
   */
  void pointCloud2ToSocialOctomap(const sensor_msgs::PointCloud2 &cloud, Pointcloud &social_octomapCloud);

  /// Conversion from social_octomap::point3d to geometry_msgs::Point
  static inline geometry_msgs::Point pointSocialOctomapToMsg(const point3d &social_octomapPt)
  {
    geometry_msgs::Point pt;
    pt.x = social_octomapPt.x();
    pt.y = social_octomapPt.y();
    pt.z = social_octomapPt.z();

    return pt;
  }

  /// Conversion from geometry_msgs::Point to social_octomap::point3d
  static inline social_octomap::point3d pointMsgToSocialOctomap(const geometry_msgs::Point &ptMsg)
  {
    return social_octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
  }

  /// Conversion from social_octomap::point3d to tf::Point
  static inline tf::Point pointSocialOctomapToTf(const point3d &social_octomapPt)
  {
    return tf::Point(social_octomapPt.x(), social_octomapPt.y(), social_octomapPt.z());
  }

  /// Conversion from tf::Point to social_octomap::point3d
  static inline social_octomap::point3d pointTfToSocialOctomap(const tf::Point &ptTf)
  {
    return point3d(ptTf.x(), ptTf.y(), ptTf.z());
  }

  /// Conversion from social_octomap Quaternion to tf::Quaternion
  static inline tf::Quaternion quaternionSocialOctomapToTf(const octomath::Quaternion &social_octomapQ)
  {
    return tf::Quaternion(social_octomapQ.x(), social_octomapQ.y(), social_octomapQ.z(), social_octomapQ.u());
  }

  /// Conversion from tf::Quaternion to social_octomap Quaternion
  static inline octomath::Quaternion quaternionTfToSocialOctomap(const tf::Quaternion &qTf)
  {
    return octomath::Quaternion(qTf.w(), qTf.x(), qTf.y(), qTf.z());
  }

  /// Conversion from social_octomap::pose6f to tf::Pose
  static inline tf::Pose poseSocialOctomapToTf(const social_octomap::pose6d &social_octomapPose)
  {
    return tf::Pose(quaternionSocialOctomapToTf(social_octomapPose.rot()), pointSocialOctomapToTf(social_octomapPose.trans()));
  }

  /// Conversion from tf::Pose to social_octomap::pose6d
  static inline social_octomap::pose6d poseTfToSocialOctomap(const tf::Pose &poseTf)
  {
    return social_octomap::pose6d(pointTfToSocialOctomap(poseTf.getOrigin()), quaternionTfToSocialOctomap(poseTf.getRotation()));
  }

}

#endif
