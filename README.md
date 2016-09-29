# DVS Gazebo Plugin

This package provides a DVS simulation implemented as Gazebo plugin.

## Install

First, make sure the DVS datatypes are available in your installation.
For this, clone the [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) package into your catkin workspace.

Then, clone this package into your workspace and rebuild.

Optionally, integrate the [FZI DVS](https://ids-git.fzi.de/hbp/dvs_ros_simulation) package into your catkin workspace
for further DVS related utilities, like event stream visualization.
This plugin package is a candidate for being merged into the just mentioned aggregate package.

## Usage

This plugin can be used as a drop-in replacement for normal Gazebo camera plugins.
Both, the DVS plugin and the [CameraPlugin](https://github.com/arpg/Gazebo/blob/master/plugins/CameraPlugin.cc)
use the Gazebo [CameraSensor](https://github.com/arpg/Gazebo/blob/master/gazebo/sensors/CameraSensor.cc) internally.

The following SDF snippet shows an example usage:

    <sensor name='camera' type='camera'>
        <camera name='__default__'>
            <horizontal_fov>1.8</horizontal_fov>
            <image>
                <width>128</width>
                <height>128</height>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>60</update_rate>
        <visualize>0</visualize>
        <plugin name='camera_controller' filename='libgazebo_dvs_plugin.so'>
            <cameraName>camera_front</cameraName>
            <robotNamespace>AADC_AudiTT</robotNamespace>
            <eventThreshold>10</eventThreshold>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <!-- <eventsTopicName>events</eventsTopicName> -->
        </plugin>
    </sensor>

The parameters `robotNamespace`, `cameraName` and `eventsTopicName` (default: "events") result in `"$robotNamespace/$cameraName/$eventsTopicName"`
as the identifier of the provided events topic.
In this case, events will be accessible from `"/AADC_AudiTT/camera_front/events"`.

The parameter `eventThreshold` specifies the pixel-wise threshold which has to be exceeded for a event to be emitted for this pixel.

The sensor parameter `update_rate` has only limited effect in Gazebo.
The real rate is determined by the rendering pipeline and can be way lower than the specified rate.
Still, this implementation yields a higher event frequency than similar Python-based implementations as a standalone node.


