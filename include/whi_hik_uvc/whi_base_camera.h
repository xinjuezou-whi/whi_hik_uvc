/******************************************************************
abstract camera interface under ROS 1

Features:
- abstract interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <sensor_msgs/Image.h>

class WhiCamera
{
public:
    WhiCamera() = default;
    virtual ~WhiCamera() = default;

public:
    virtual bool open() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual sensor_msgs::Image::Ptr capture() = 0;
    virtual std::string getCameraName() const = 0;
    
public:
    bool isOpened() { return is_opened_; };

protected:
    bool is_opened_{ false };
};
