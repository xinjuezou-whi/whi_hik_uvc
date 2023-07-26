/******************************************************************
HIK UVC camera interface under ROS 1

Features:
- HIK UVC interfaces based on SDK
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_hik_uvc/whi_hik_sdk_device.h"

#include <ros/ros.h>

#include <sstream>

namespace whi_hik_uvc
{
    bool HikSdk::open()
    {
        // sdk init process
        if (!USB_Init())
        {
            DWORD error = USB_GetLastError();
            ROS_ERROR_STREAM("failed to init USB, err " << error << ": " << USB_GetErrorMsg(error));
            return false;
        }
        ROS_INFO_STREAM(getVersionString());
        auto deviceNum = USB_GetDeviceCount();
        std::cout << "nnnnnnnnnnnnnnnnnn " << deviceNum << std::endl;
        if (USB_EnumDevices(deviceNum, &device_info_[0]))
        {
            ROS_INFO_STREAM("enumerated device: VID [" << device_info_[0].dwVID << "], PID [" << device_info_[0].dwPID << "]");
        }
        USB_DEVICE_REG_RES deviceRegRes = { 0 };
        user_id_ = USB_Login(&cur_user_login_info_, &deviceRegRes);
        std::cout << "ddddddddddddddddddddddddddddddddd " << user_id_ << std::endl;
        getDeviceInfo(user_id_);

        return true;
    }

    bool HikSdk::start()
    {
        return true;
    }

    bool HikSdk::stop()
    {
        return true;
    }

    sensor_msgs::Image::Ptr HikSdk::capture()
    {
        return nullptr;
    }

    std::string HikSdk::getCameraName() const
    {
        return "";
    }

    std::string HikSdk::getVersionString() const
    {
        unsigned int version = USB_GetSDKVersion();
        std::stringstream verStream;
        verStream << "HCUSBSDK V"
            << ((0xff000000 & version) >> 24) << "."
            << ((0x00ff0000 & version) >> 16) << "."
            << ((0x0000ff00 & version) >> 8) << "."
            << (0x000000ff & version);
        
        return verStream.str();
    }

    std::string HikSdk::getDeviceInfo(long UserId) const
    {
        std::stringstream info;

	    USB_CONFIG_INPUT_INFO configInputInfo = { 0 };
	    USB_CONFIG_OUTPUT_INFO configOutputInfo = { 0 };
	    USB_SYSTEM_DEVICE_INFO sysDevInfo = { 0 };

	    sysDevInfo.dwSize = sizeof(sysDevInfo);
	    configOutputInfo.lpOutBuffer = &sysDevInfo;
	    configOutputInfo.dwOutBufferSize = sizeof(sysDevInfo);
	    if (!USB_GetDeviceConfig(UserId, USB_GET_SYSTEM_DEVICE_INFO, &configInputInfo, &configOutputInfo))
	    {
		    DWORD error = USB_GetLastError();
            ROS_ERROR_STREAM("failed to get device config, err " << error << ": " << USB_GetErrorMsg(error));
        }
	    else
	    {
            std::stringstream info;
            info << "usb device info: device type[" << sysDevInfo.byDeviceType << "], encoder version ["
                << sysDevInfo.byFirmwareVersion << "] firmware version [" << sysDevInfo.byFirmwareVersion
                << "] hardware version [" << sysDevInfo.byHardwareVersion << "] protocol version ["
                << sysDevInfo.byProtocolVersion << "]";
            ROS_INFO_STREAM(info.str());
	    }

        return info.str();
    }

} // namespace whi_hik_uvc
