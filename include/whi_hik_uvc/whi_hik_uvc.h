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
#include "whi_hik_uvc/whi_v4l_camera_device.h"

#include <ros/ros.h>

#include <memory>
#include <thread>

//#include "imu_base.h"

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
        void streaming();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        std::thread th_streaming_;
        std::atomic<bool> terminated_{ false };
        std::unique_ptr<v4l2_camera::V4l2CameraDevice> camera_;
        std::string device_{ "/dev/video0" };
        std::string frame_id_{ "camera" };
	};
} // namespace whi_hik_uvc
