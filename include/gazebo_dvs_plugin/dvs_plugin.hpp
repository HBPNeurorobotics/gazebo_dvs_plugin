/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
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

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>


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

    private: Mat last_image;
    private: Mat ref_image;
    private: ros::Time received_last;
    private: bool has_last_image;
    private: float event_threshold;
    private: void updateDVS(ros::Time received_current, Mat *curr_image);
    private: void processDelta(Mat *last_image, Mat *curr_image);
    private: void appendEvent(std::vector<dvs_msgs::Event> *events, int x, int y, ros::Time ts, bool polarity);
    private: void fillEvents(Mat *diff, int polarity, vector<dvs_msgs::Event> *events);
    private: void publishEvents(vector<dvs_msgs::Event> *events);
  };
}
#endif
