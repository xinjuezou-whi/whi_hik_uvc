/******************************************************************
HIK UVC camera interface under ROS 1

Features:
- HIK UVC interfaces based on SDK
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_abstract_camera.h"
#include "HCUsbSDK.h"

#include <sensor_msgs/Image.h>

#include <memory>
#include <thread>

namespace whi_hik_uvc
{
    class HikSdk : public WhiCamera
	{
    public:
        HikSdk() = default;
        ~HikSdk() = default;

    protected:
        bool open() override;
        bool start() override;
        bool stop() override;
        sensor_msgs::Image::Ptr capture() override;
        std::string getCameraName() const override;

    protected:
        std::string getVersionString() const;
        std::string getDeviceInfo(long UserId) const;

    protected:
        USB_DEVICE_INFO device_info_[64];
        USB_USER_LOGIN_INFO cur_user_login_info_;
        long user_id_{ 0 };
	};
} // namespace whi_hik_uvc
