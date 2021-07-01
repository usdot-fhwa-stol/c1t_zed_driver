#ifndef C1T_ZED_DRIVER_H
#define C1T_ZED_DRIVER_H

/**
 * Copyright 2021 U.S. Department of Transportation, Federal Highway Administration
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cav_driver_utils/driver_wrapper/driver_wrapper.h>

class ZedDriverWrapper : public cav::DriverWrapper
{
public:
  /**
   * @brief ZedDriverWrapper constructor
   *
   * @param argc commandline argument count
   * @param argv array of commandline argument values
   * @param name ROS node name
   */
  ZedDriverWrapper(int argc, char** argv, const std::string& name = "c1t_zed_driver");

  /**
   * @brief ZedDriverWrapper destructor
   */
  ~ZedDriverWrapper() = default;

private:
  ros::Subscriber camera_sub_;
  ros::Time last_update_time_;

  double camera_timeout_;

  /**
   * @brief Initializes the ROS node
   *
   * This function is called before the ROS node starts running.
   */
  void initialize() override;

  /**
   * @brief Process stuff before spinning
   *
   * This function is called before the spinOnce() function.
   */
  void pre_spin() override;

  /**
   * @brief Process stuff after spinning
   *
   * This function is called after the spinOnce() function.
   */
  void post_spin() override;

  /**
   * @brief Prepare ROS node for shutting down
   *
   * This function is called before the node is shut down.
   */
  void shutdown() override;

  /**
   * @brief Check if the ZED camera has timed out
   * 
   * Check if the time between ZED camera messages exceeds the timeout threshold.
   */
  void checkCameraTimeout();
};

#endif  // C1T_ZED_DRIVER_H