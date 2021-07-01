/**
 * Copyright 2021 U.S. Department of Transportation, Federal Highway Administration
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "c1t_zed_driver/c1t_zed_driver.h"

#include <sensor_msgs/Image.h>

ZedDriverWrapper::ZedDriverWrapper(int argc, char** argv, const std::string& name)
  : cav::DriverWrapper(argc, argv, name)
{
}

void ZedDriverWrapper::initialize()
{
  status_.camera = true;

  private_nh_->param<double>("camera_timeout", camera_timeout_, 0.5);

  camera_sub_ = nh_->subscribe<sensor_msgs::Image>("left_raw/image_raw_color", 1,
                                                   [this](const sensor_msgs::Image::ConstPtr& msg) {
                                                     last_update_time_ = ros::Time::now();
                                                     status_.status = cav_msgs::DriverStatus::OPERATIONAL;
                                                   });
}

void ZedDriverWrapper::pre_spin()
{
  this->checkCameraTimeout();
}

void ZedDriverWrapper::post_spin()
{
}

void ZedDriverWrapper::shutdown()
{
}

void ZedDriverWrapper::checkCameraTimeout()
{
  if (last_update_time_.isZero() || ros::Time::now() - last_update_time_ > ros::Duration(camera_timeout_))
  {
    status_.status = cav_msgs::DriverStatus::OFF;
  }
}
