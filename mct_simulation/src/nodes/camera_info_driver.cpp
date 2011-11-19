// $Id: driver1394.cpp 36902 2011-05-26 23:20:18Z joq $

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
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
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
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

#include "camera_info_driver.h"

/** @file

    @brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

    This is a ROS driver for 1394 cameras, using libdc1394.  It can be
    instantiated as either a node or a nodelet.  It is written with with
    minimal dependencies, intended to fill a role in the ROS image
    pipeline similar to the other ROS camera drivers.

    @par Advertises

    - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
    information for each image.

*/

namespace camera_info_driver
{
  CameraInfoDriver::CameraInfoDriver(ros::NodeHandle priv_nh,
                                     ros::NodeHandle camera_nh):
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("camera"),
    cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_)),
    calibration_matches_(true)
  {
    if (!cinfo_->setCameraName(camera_name_))
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] name not valid"
                        << " for camera_info_manger");
      }
  }

  CameraInfoDriver::~CameraInfoDriver()
  {}

  /** device poll */
  // void CameraInfoDriver::poll(void)
  // {}

  // void CameraInfoDriver::setup(void)
  // {}

  /** driver termination */
  // void CameraInfoDriver::shutdown(void)
  // {}

}; // end namespace camera_info_driver