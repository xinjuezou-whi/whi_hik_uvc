/******************************************************************
HIK UVC camera interface under ROS 1

Features:
- abstract HIK UVC interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_hik_uvc/whi_hik_uvc.h"

//#include <cv_bridge/cv_bridge.h>

namespace whi_hik_uvc
{
    HikUvc::HikUvc(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    HikUvc::~HikUvc()
    {
        terminated_.store(true);
        if (th_streaming_.joinable())
        {
            th_streaming_.join();
        }
    }

    void HikUvc::init()
    {
        // device params
        node_handle_->param("whi_hik_uvc/device", device_, std::string("/dev/video0"));
        node_handle_->param("whi_hik_uvc/frame_id", frame_id_, std::string("camera"));
        std::string topicImg, topicInfo;
        node_handle_->param("whi_hik_uvc/topic_image", topicImg, std::string("image"));
        node_handle_->param("whi_hik_uvc/cam_info", topicInfo, std::string("cam_info"));

        // publisher
        image_transport_ = std::make_unique<image_transport::ImageTransport>(*node_handle_);
        pub_image_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise(topicImg, 1));
        pub_info_ = std::make_unique<image_transport::CameraPublisher>();

        // spinner
        node_handle_->param("whi_hik_uvc/loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq,
            std::bind(&HikUvc::update, this, std::placeholders::_1)));

        streaming();
    }

    void HikUvc::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        // TODO
        //std::cout << "elapsed " << elapsed_time_.toSec() << std::endl;
    }

    void HikUvc::streaming()
    {
        camera_ = std::make_unique<v4l2_camera::V4l2CameraDevice>(device_);
        camera_->open();
        camera_->start();

        cam_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(*node_handle_, camera_->getCameraName());

        th_streaming_ = std::thread
        {
            [this]() -> void
            {
                while (!terminated_.load())
                {
                    auto img = camera_->capture();
                    if (img == nullptr)
                    {
                        // Failed capturing image, assume it is temporarily and continue a bit later
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }

                    // if (img->encoding != output_encoding_) // TODO
                    // {
                    //     ROS_WARN_STREAM_ONCE(
                    //         "Image encoding not the same as requested output, performing possibly slow conversion: " <<
                    //         img->encoding << " => " << output_encoding_);
                    //     img = convert(*img);
                    // }
                    // img = convert(img);
                    img->header.stamp = ros::Time::now();
                    img->header.frame_id = frame_id_;
                    pub_image_->publish(img);
                    auto camInfo = std::make_unique<sensor_msgs::CameraInfo>(cam_info_->getCameraInfo());
                    pub_info_->publish(*img, *camInfo, img->header.stamp);
                }
            }
        };
    }

    // sensor_msgs::Image::Ptr HikUvc::convert(const sensor_msgs::ImageConstPtr Img) const
    // {
    //     auto cvImg = cv_bridge::toCvShare(Img);
    //     auto outImg = std::make_unique<sensor_msgs::Image>();
    //     auto cvConvertedImg = cv_bridge::cvtColor(cvImg, "bgr8");
    //     cvConvertedImg->toImageMsg(*outImg);

    //     return outImg;
    // }
} // namespace whi_hik_uvc
