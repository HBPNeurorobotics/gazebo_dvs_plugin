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
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * IMPLEMENTATION INSPIRED BY
 * https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp
 */

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <gazebo_dvs_plugin/dvs_plugin.hpp>

using namespace std;
using namespace cv;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(DvsPlugin)

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    DvsPlugin::DvsPlugin()
    : SensorPlugin(), width(0), height(0), depth(0), has_last_image(false)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  DvsPlugin::~DvsPlugin()
  {
    this->parentSensor.reset();
    this->camera.reset();
  }

  void DvsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    if (!_sensor)
      gzerr << "Invalid sensor pointer." << endl;

#if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera = this->parentSensor->Camera();
#else
    this->parentSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera = this->parentSensor->GetCamera();
#endif

    if (!this->parentSensor)
    {
      gzerr << "DvsPlugin not attached to a camera sensor." << endl;
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 7
    this->width = this->camera->ImageWidth();
    this->height = this->camera->ImageHeight();
    this->depth = this->camera->ImageDepth();
    this->format = this->camera->ImageFormat();
#else
    this->width = this->camera->GetImageWidth();
    this->height = this->camera->GetImageHeight();
    this->depth = this->camera->GetImageDepth();
    this->format = this->camera->GetImageFormat();
#endif

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a robotNamespace." << endl;

    node_handle_ = ros::NodeHandle(namespace_);

    string sensorName = "";
    if (_sdf->HasElement("cameraName"))
      sensorName = _sdf->GetElement("cameraName")->Get<std::string>() + "/";
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a cameraName." << endl;

    string topicName = "events";
    if (_sdf->HasElement("eventsTopicName"))
      topicName = _sdf->GetElement("eventsTopicName")->Get<std::string>();

    const string topic = sensorName + topicName;

    if (_sdf->HasElement("eventThreshold"))
      this->event_threshold = _sdf->GetElement("eventThreshold")->Get<float>();
    else
      gzwarn << "[gazebo_ros_dvs_camera] Please specify a DVS threshold." << endl;

    event_pub_ = node_handle_.advertise<dvs_msgs::EventArray>(topic, 10, 10.0);

    this->newFrameConnection = this->camera->ConnectNewImageFrame(
        boost::bind(&DvsPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

    this->parentSensor->SetActive(true);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void DvsPlugin::OnNewFrame(const unsigned char *_image,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string &_format)
  {
    ros::Time received_current = ros::Time::now();
    
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
#else
    _image = this->camera->GetImageData(0);
#endif

    // convert given frame to opencv image
    cv::Mat input_image(_height, _width, CV_8UC3);
    cv::Mat curr_image(_height, _width, CV_8UC1);
    
    input_image.data = (uchar*)_image;

    cvtColor(input_image, input_image, CV_RGB2BGR);
    cvtColor(input_image, curr_image, CV_BGR2GRAY);

/* TODO any encoding configuration should be supported
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*_image, sensor_msgs::image_encodings::BGR8);
      std::cout << "Image: " << std::endl << " " << cv_ptr->image << std::endl << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception %s", e.what());
      std::cout << "ERROR";
    }
*/

    assert(_height == height && _width == width);
    if (this->has_last_image)
    {
      this->updateDVS(received_current, &curr_image);
    }
    else if (curr_image.size().area() > 0)
    {
      this->last_image = curr_image;
      curr_image.convertTo(this->ref_image, CV_32FC1);
      this->received_last = received_current;
      this->has_last_image = true;
    }
    else
    {
      gzwarn << "Ignoring empty image." << endl;
    }
  }
  
  void DvsPlugin::updateDVS(ros::Time received_current, cv::Mat *curr_image)
  {
    if (curr_image->size() == this->last_image.size())
    {
      ros::Duration delta_t = received_current - this->received_last;
      
      std::vector<dvs_msgs::Event> events;

      for(int y = 0; y < curr_image->rows; y++)
      {
      	for(int x = 0; x < curr_image->cols; x++)
	{
          int it = (int)this->last_image.at<uchar>(y,x); // TODO check if pointer method faster than at??
          int it_dt = (int)curr_image->at<uchar>(y,x);
          float ref = this->ref_image.at<float>(y,x);
	  int polarity = (it_dt>=it) ? (1) : (-1);
	  
	  if(fabs(it_dt-it) > 1e-6) // TODO are these float values in rpg davis version??
	  {
	    
	    std::vector<float> list_crossings;
	    bool all_crossings_found = false;
	    float current_crossing = ref;
	    
	    while(!all_crossings_found) // TODO make it directly, so that list_crossings is unnecessary!
	    {
	      current_crossing = current_crossing + polarity*(this->event_threshold);
	      if(polarity > 0)
	      {
		if(current_crossing > it && current_crossing <= it_dt)
		  list_crossings.push_back(current_crossing);
		else
		  all_crossings_found = true;
	      }
	      else
	      {
		if(current_crossing < it && current_crossing >= it_dt)
		  list_crossings.push_back(current_crossing);
		else
		  all_crossings_found = true;
	      }
	    }
	    
	    for(int i = 0; i<list_crossings.size(); ++i)
	    {
	      ros::Time event_ts = this->received_last + delta_t * ((list_crossings[i]-1.0*it) / (1.0*(it_dt-it)));
	      
	      dvs_msgs::Event event_i;
	      event_i.x = x;
	      event_i.y = y;
	      event_i.ts = event_ts;
	      event_i.polarity = (polarity>0);
	      
	      events.push_back(event_i);
	    }
	    
	    if(list_crossings.size() > 0)
	      this->ref_image.at<float>(y,x) = list_crossings.back();
	  }
	}
      }
      std::cout << " Events: " << events.size() << std::endl;

      this->publishEvents(&events);
      curr_image->copyTo(this->last_image);
    }
    else
    {
      gzwarn << "Unexpected change in image size (" << this->last_image.size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << endl;
    }
  }

  void DvsPlugin::processDelta(cv::Mat *last_image, cv::Mat *curr_image)
  {
    if (curr_image->size() == last_image->size())
    {
      cv::Mat pos_diff = *curr_image - *last_image;
      cv::Mat neg_diff = *last_image - *curr_image;

      cv::Mat pos_mask;
      cv::Mat neg_mask;

      cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
      cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);

      *last_image += pos_mask & pos_diff;
      *last_image -= neg_mask & neg_diff;
      
      std::vector<dvs_msgs::Event> events;
      
      this->fillEvents(&pos_mask, 0, &events);
      this->fillEvents(&neg_mask, 1, &events);

      this->publishEvents(&events);
    }
    else
    {
      gzwarn << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << endl;
    }
  }

  void DvsPlugin::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
  {
    // findNonZero fails when there are no zeros
    // TODO is there a better workaround then iterating the binary image twice?
    if (cv::countNonZero(*mask) != 0)
    {
      std::vector<cv::Point> locs;
      cv::findNonZero(*mask, locs);

      for (int i = 0; i < locs.size(); i++)
      {
        dvs_msgs::Event event;
        event.x = locs[i].x;
        event.y = locs[i].y;
        event.ts = ros::Time::now();
        event.polarity = polarity;
        events->push_back(event);
      }
    }
  }

  void DvsPlugin::publishEvents(std::vector<dvs_msgs::Event> *events)
  {
    if (events->size() > 0)
    {
      dvs_msgs::EventArray msg;
      msg.events.clear();
      msg.events.insert(msg.events.end(), events->begin(), events->end());
      msg.width = width;
      msg.height = height;

      // TODO what frame_id is adequate?
      msg.header.frame_id = namespace_;
      msg.header.stamp = ros::Time::now();

      event_pub_.publish(msg);
    }
  }
}
