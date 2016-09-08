/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * bla: Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: A dynamic controller plugin that publishes ROS image_raw
 *    camera_info topic for generic camera sensor.
*/

#ifndef GAZEBO_ROS_DVS_CAMERA_HH
#define GAZEBO_ROS_DVS_CAMERA_HH

#include <string>
#include <ros/ros.h>

// library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera.h>
//#include <gazebo_plugins/gazebo_ros_utils.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/opencv.hpp>

namespace gazebo
{
  class GazeboRosDvsCamera : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosDvsCamera();

    /// \brief Destructor
    public: ~GazeboRosDvsCamera();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    private: cv::Mat last_image;
    private: bool has_last_image;

    protected: ros::NodeHandle nh_;
    protected: std::string event_topic_name_;
    protected: ros::Publisher event_pub_;

    private: void processDelta(cv::Mat *last_image, cv::Mat *curr_image);
    private: void fillEvents(cv::Mat diff, int polarity, std::vector<dvs_msgs::Event> *events);
    private: void publishEvents(std::vector<dvs_msgs::Event> *events);
  };
}
#endif

