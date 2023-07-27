/******************************************************************
HIK UVC camera interface under ROS 1

Features:
- abstract HIK UVC interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-11: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_abstract_camera.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <memory>
#include <thread>

namespace whi_hik_uvc
{
	class HikUvc
	{
    public:
        HikUvc(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~HikUvc();

    protected:
        void init();
        void update(const ros::TimerEvent & Event);
        void streaming(std::shared_ptr<WhiCamera> Camera);
        sensor_msgs::Image::Ptr convert(const sensor_msgs::ImageConstPtr Img) const;

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        std::thread th_streaming_;
        std::atomic<bool> terminated_{ false };
        std::string frame_id_{ "camera" };
        std::unique_ptr<image_transport::ImageTransport> image_transport_{ nullptr };
        std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_{ nullptr };
        std::unique_ptr<image_transport::Publisher> pub_image_{ nullptr };
        std::unique_ptr<image_transport::CameraPublisher> pub_info_{ nullptr };
	};
} // namespace whi_hik_uvc
