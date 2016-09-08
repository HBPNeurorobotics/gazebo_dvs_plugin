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

#ifndef DVS_PLUGIN_HPP
#define DVS_PLUGIN_HPP

#include <string>
#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace gazebo
{
  class GAZEBO_VISIBLE DvsPlugin : public SensorPlugin
  {
    public: DvsPlugin();

    public: ~DvsPlugin();

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected: virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const string &_format);

    protected: unsigned int width, height, depth;
    protected: string format;

    protected: sensors::CameraSensorPtr parentSensor;
    protected: rendering::CameraPtr camera;

    private: event::ConnectionPtr newFrameConnection;

    protected: ros::NodeHandle node_handle_;
    protected: ros::Publisher event_pub_;
    protected: string namespace_;
    protected: string event_topic_name_;

    private: Mat last_image;
    private: bool has_last_image;

    private: void processDelta(Mat *last_image, Mat *curr_image);
    private: void fillEvents(Mat diff, int polarity, vector<dvs_msgs::Event> *events);
    private: void publishEvents(vector<dvs_msgs::Event> *events);
  };
}
#endif

