/* -*- mode: C++ -*- */
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
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>


/** @file

    @brief ROS driver interface for simulated camera info.

*/

namespace sim_camera_driver
{

  class SimCameraDriver
  {
  public:

    // public methods
    SimCameraDriver(ros::NodeHandle priv_nh,
                     ros::NodeHandle camera_nh);
    ~SimCameraDriver();
    void imageCallback(const sensor_msgs::ImageConstPtr& image);

  private:

    // private methods
    void publish(const sensor_msgs::ImageConstPtr &image);

    ros::NodeHandle priv_nh_;             // private node handle
    ros::NodeHandle camera_nh_;           // camera name space handle
    std::string camera_name_;             // camera name
    std::string camera_info_url_;         // camera info url

    /** camera calibration information */
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    bool calibration_found_;

    /** image transport interfaces */
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraPublisher image_pub_;
    image_transport::Subscriber image_sub_;

  }; // end class SimCameraDriver

}; // end namespace sim_camera_driver
