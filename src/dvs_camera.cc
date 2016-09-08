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
   @mainpage
Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
Author: John Hsu
Date: 24 Sept 2008
 */

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dvs_gazebo_plugin/dvs_camera.h>

#include <stdio.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDvsCamera)

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboRosDvsCamera::GazeboRosDvsCamera():
      nh_("~dvs_camera_plugin"), // TODO private NS
      event_topic_name_("events")
  {
    // also use ImageConnect and ImageDisconnect to control sensor activity
    // - queue size = 100 (with a higher update_rate, a higher queue size becomes necessary)
    // - latch = false
    event_pub_ = nh_.advertise<dvs_msgs::EventArray>(
        event_topic_name_, 100/*,
                                boost::bind(&GazeboRosDvsCamera::ImageConnect, this),
                                boost::bind(&GazeboRosDvsCamera::ImageDisconnect, this),
                                ros::VoidPtr(), false*/);

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosDvsCamera::~GazeboRosDvsCamera()
  {
  }

  void GazeboRosDvsCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  { 
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);

/*
    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

    nh_ = transport::NodePtr(new transport::Node());
    nh_->Init(namespace_);
*/

    /* TODO
       if (this->sdf->HasElement("eventsTopicName"))
       this->event_topic_name_ = this->sdf->Get<std::string>("eventsTopicName");
     */
    this->ImageConnect();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosDvsCamera::OnNewFrame(const unsigned char *_image,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string &_format)
  {
# if GAZEBO_MAJOR_VERSION >= 7
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
# else
    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
# endif

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
        // do this first so there's chance for sensor to run once after activated
        this->parentSensor->SetActive(true);
    }
    else
    {
      common::Time cur_time = this->world_->GetSimTime();

      this->PutCameraData(_image); // TODO ????
      this->PublishCameraInfo();

      if (cur_time - this->last_update_time_ >= this->update_period_)
      {
        // inspired by https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp

        // convert given frame to opencv image
        cv::Mat input_image(_height, _width, CV_8UC3);
        input_image.data = (uchar*)_image;

        // color to grayscale
        cv::Mat curr_image_rgb(_height, _width, CV_8UC3);
        cvtColor(input_image, curr_image_rgb, CV_RGB2BGR); 
        cvtColor(curr_image_rgb, input_image, CV_BGR2GRAY);

        cv::Mat curr_image = input_image;

        /* DEBUG
           std::cout << "Format: " << _format << std::endl;
           std::cout << "Image: " << std::endl << " " << input_image << std::endl << std::endl;
         */

        /* TODO compatibility
           try {
           cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*_image, sensor_msgs::image_encodings::BGR8);
           std::cout << "Image: " << std::endl << " " << cv_ptr->image << std::endl << std::endl;
           } catch (cv_bridge::Exception& e)
           {
           ROS_ERROR("cv_bridge exception %s", e.what());
           std::cout << "ERROR";
           }
         */

        assert(_height == this->height_ && _width == this->width_);
        if (this->has_last_image)
        {
          this->processDelta(&this->last_image, &curr_image);
        }
        this->last_image = curr_image;
        this->has_last_image = true;
        this->last_update_time_ = cur_time;
      }
    }
  }

  void GazeboRosDvsCamera::processDelta(cv::Mat *last_image, cv::Mat *curr_image)
  {
    if (curr_image->size() == last_image->size())
    {
      std::vector<dvs_msgs::Event> events;

      this->fillEvents(*curr_image - *last_image, 0, &events);
      this->fillEvents(*last_image - *curr_image, 1, &events);

      ROS_DEBUG_NAMED("dvs", "processDelta - publishing");
      this->publishEvents(&events);
    }
  }

  void GazeboRosDvsCamera::fillEvents(cv::Mat diff, int polarity, std::vector<dvs_msgs::Event> *events)
  {
    float threshold = 0.001;

    cv::Mat binary = diff > threshold;

    if (cv::countNonZero(binary) != 0)
    {
      std::vector<cv::Point> locs;
      cv::findNonZero(binary, locs);

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

  void GazeboRosDvsCamera::publishEvents(std::vector<dvs_msgs::Event> *events)
  {
    if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
      return;

    /// don't bother if there are no subscribers
    if ((*this->image_connect_count_) > 0)
    {
      // TODO ...
    }

    if (events->size() > 0)
    {
      boost::mutex::scoped_lock lock(this->lock_);

      dvs_msgs::EventArray msg;
      msg.events.clear();
      msg.events.insert(msg.events.end(), events->begin(), events->end());
      msg.width = this->width_;
      msg.height = this->height_;

      // copy data into image
      msg.header.frame_id = this->frame_name_;
      msg.header.stamp.sec = this->sensor_update_time_.sec;
      msg.header.stamp.nsec = this->sensor_update_time_.nsec;

      // publish to ros
      this->event_pub_.publish(msg);
    }
  }
}
