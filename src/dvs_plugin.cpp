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
    : SensorPlugin(), width(0), height(0), depth(0)
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
#if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
#else
    _image = this->camera->GetImageData(0);
#endif

    /*
#if GAZEBO_MAJOR_VERSION >= 7
float rate = this->camera->RenderRate();
#else
float rate = this->camera->GetRenderRate();
#endif
if (!isfinite(rate))
rate =  30.0;
float dt = 1.0 / rate;
     */

    // convert given frame to opencv image
    cv::Mat input_image(_height, _width, CV_8UC3);
    input_image.data = (uchar*)_image;

    // color to grayscale
    cv::Mat curr_image_rgb(_height, _width, CV_8UC3);
    cvtColor(input_image, curr_image_rgb, CV_RGB2BGR);
    cvtColor(curr_image_rgb, input_image, CV_BGR2GRAY);

    cv::Mat curr_image = input_image;

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
      this->processDelta(&this->last_image, &curr_image);
    }
    this->last_image = curr_image;
    this->has_last_image = true;
  }

  void DvsPlugin::processDelta(cv::Mat *last_image, cv::Mat *curr_image)
  {
    if (curr_image->size() == last_image->size())
    {
      std::vector<dvs_msgs::Event> events;

      this->fillEvents(*curr_image - *last_image, 0, &events);
      this->fillEvents(*last_image - *curr_image, 1, &events);

      this->publishEvents(&events);
    }
    else
    {
      gzwarn << "Unexpected change in image size. Publishing no events for this frame change." << endl;
    }
  }

  void DvsPlugin::fillEvents(cv::Mat diff, int polarity, std::vector<dvs_msgs::Event> *events)
  {
    cv::Mat binary = diff > event_threshold;

    // findNonZero fails when there are no zeros
    // TODO is there a better workaround then iterating the binary image twice?
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
