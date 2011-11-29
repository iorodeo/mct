/* -*- mode: C++ -*- */
/* $Id: driver1394.h 36902 2011-05-26 23:20:18Z joq $ */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010 Jack O'Quin
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

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>


/** @file

    @brief ROS driver interface for simulated camera info.

*/

namespace camera_info_driver
{

  class CameraInfoDriver
  {
  public:

    // public methods
    CameraInfoDriver(ros::NodeHandle priv_nh,
                     ros::NodeHandle camera_nh);
    ~CameraInfoDriver();
    void publish(void);
    // void setup(void);
    // void shutdown(void);

  private:

    // private methods

    ros::NodeHandle priv_nh_;             // private node handle
    ros::NodeHandle camera_nh_;           // camera name space handle
    std::string camera_name_;             // camera name
    std::string camera_info_url_;         // camera info url
    sensor_msgs::CameraInfo camera_info_; // camera info
    ros::Publisher camera_info_pub_;      // camera info publisher
    ros::Rate loop_rate;                  // loop rate

    /** camera calibration information */
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    bool calibration_found_;

  }; // end class CameraInfoDriver

}; // end namespace camera_info_driver
