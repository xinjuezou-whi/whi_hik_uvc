/******************************************************************
v4l camera device under ROS 1

Features:
- v2l camera
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_hik_uvc/whi_v4l_device.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

namespace v4l2_camera
{
    static int xioctl(int Handle, int Request, void* Arg)
    {
        int r;

        do
        {
            r = ioctl(Handle, Request, Arg);
        } while (-1 == r && EINTR == errno);

        return r;
    }

    bool V4l2CameraDevice::open()
    {
        fd_ = ::open(device_.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            ROS_FATAL_STREAM("Failed opening device " << device_ << ": " << strerror(errno) << " (" << errno << ")");
            return false;
        }

        // List capabilities
        xioctl(fd_, VIDIOC_QUERYCAP, &capabilities_);

        auto canRead = capabilities_.capabilities & V4L2_CAP_READWRITE;
        auto canStream = capabilities_.capabilities & V4L2_CAP_STREAMING;

        ROS_INFO_STREAM("Driver: " << capabilities_.driver);
        ROS_INFO_STREAM("Version: " << capabilities_.version);
        ROS_INFO_STREAM("Device: " << capabilities_.card);
        ROS_INFO_STREAM("Location: " << capabilities_.bus_info);
        ROS_INFO_STREAM("Capabilities:\n" << "  Read/write: " << (canRead ? "YES\n" : "NO\n")
            << "  Streaming: " << (canStream ? "YES" : "NO"));

        // Get current data (pixel) format
        v4l2_format formatReq;
        formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_G_FMT, &formatReq);
        cur_data_format_ = PixelFormat(formatReq.fmt.pix);
        ROS_INFO_STREAM("Current pixel format: " << v4l2_fourcc::toString(cur_data_format_.format_) <<
            " @ " << cur_data_format_.width_ << "x" << cur_data_format_.height_);

        // List all available image formats and controls
        listImageFormats();
        listImageSizes();
        listControls();

        ROS_INFO("Available pixel formats:");
        for (auto const & format : image_formats_)
        {
            ROS_INFO_STREAM("  " << v4l2_fourcc::toString(format.format_) << " - " << format.description_);
        }

        if (controls_.empty())
        {
            ROS_INFO("Available controls: none");
        }
        else
        {
            ROS_INFO("Available controls:");
            for (auto const & control : controls_)
            {
                ROS_INFO_STREAM("  " << control.name_ << " (" << static_cast<unsigned>(control.type_) <<
                    ") = " << getControlValue(control.id_) << (control.inactive_ ? " [inactive]" : ""));
            }
        }

        is_opened_ = true;
        return is_opened_;
    }

    bool V4l2CameraDevice::start()
    {
        ROS_INFO("Starting camera");

        // set to YUYV if it is supported by camera
        auto isYuyv = [](ImageFormat Format) { return Format.format_ == V4L2_PIX_FMT_YUYV; };
        if (auto it = std::find_if(image_formats_.begin(), image_formats_.end(), isYuyv); it != image_formats_.end())
        {
            cur_data_format_.format_ = V4L2_PIX_FMT_YUYV;
            v4l2_format fmtSet;
            fmtSet.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmtSet.fmt.pix.width = cur_data_format_.width_;
            fmtSet.fmt.pix.height = cur_data_format_.height_;
            fmtSet.fmt.pix.pixelformat = cur_data_format_.format_;
            fmtSet.fmt.pix.field = V4L2_FIELD_ANY;
            xioctl(fd_, VIDIOC_S_FMT, &fmtSet);
        }

        if (!initMemoryMapping())
        {
            return false;
        }

        // Queue the buffers
        for (auto const& buffer : buffers_)
        {
            v4l2_buffer buf;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = buffer.index_;
            if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
            {
                ROS_ERROR_STREAM("Buffer failure on capture start: " << strerror(errno) << " (" << errno << ")");
                return false;
            }
        }

        // Start stream
        unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        {
            ROS_ERROR_STREAM("Failed stream start: " << strerror(errno) << " (" << errno << ")");
            return false;
        }
        return true;
    }

    bool V4l2CameraDevice::stop()
    {
        return true;
    }

    sensor_msgs::Image::Ptr V4l2CameraDevice::capture()
    {
        v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        // Dequeue buffer with new image
        if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
        {
            ROS_ERROR_STREAM("Error dequeueing buffer: " << strerror(errno) << " (" << errno << ")");
            return nullptr;
        }

        // Create image object
        auto img = std::make_unique<sensor_msgs::Image>();

        // Copy over buffer data
        const auto& buffer = buffers_[buf.index];
        img->data.resize(cur_data_format_.image_byte_size_);
        std::copy(buffer.start_, buffer.start_ + img->data.size(), img->data.begin());

        // Requeue buffer to be reused for new captures
        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        {
            ROS_ERROR_STREAM("Error re-queueing buffer: " << strerror(errno) << " (" << errno);
            return nullptr;
        }

        // Fill in remaining image information
        img->width = cur_data_format_.width_;
        img->height = cur_data_format_.height_;
        img->step = cur_data_format_.bytes_per_line_;
        if (cur_data_format_.format_ == V4L2_PIX_FMT_YUYV)
        {
            img->encoding = sensor_msgs::image_encodings::BGR8; //sensor_msgs::image_encodings::YUV422_YUY2;

            /// output is YUY2
            cv::Mat yuyv(cur_data_format_.height_, cur_data_format_.width_, CV_8UC2, img->data.data());
            cv::Mat bgr(cur_data_format_.height_, cur_data_format_.width_, CV_8UC3);
	        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
#ifdef DEBUG
            /// separation
            cv::Mat inY = cv::Mat(cv::Size(cur_data_format_.width_, cur_data_format_.height_), CV_8UC1, img->data.data());
            cv::imshow("y", inY);
            cv::Mat inV = cv::Mat(cv::Size(cur_data_format_.width_/2, cur_data_format_.height_/2), CV_8UC1,
                img->data.data() + cur_data_format_.bytes_per_line_ * cur_data_format_.height_ / 2,
                cur_data_format_.bytes_per_line_ / 4);
            cv::imshow("v", inV);
            cv::Mat inU = cv::Mat(cv::Size(cur_data_format_.width_/2, cur_data_format_.height_/2), CV_8UC1,
                img->data.data() + cur_data_format_.bytes_per_line_ * cur_data_format_.height_ / 2 +
                (cur_data_format_.bytes_per_line_ / 4) * (cur_data_format_.height_ / 2),
                cur_data_format_.bytes_per_line_ / 4);
            cv::imshow("u", inU);
            cv::imshow("previewBGR", bgr);
		    cv::waitKey(10);
#endif
            img->data.resize(bgr.cols * bgr.rows * bgr.channels());
            std::copy(bgr.datastart, bgr.dataend, img->data.begin());
        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_UYVY)
        {
            img->encoding = sensor_msgs::image_encodings::YUV422;
        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_GREY)
        {
            img->encoding = sensor_msgs::image_encodings::MONO8;
        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_MJPEG)
        {
            img->encoding = sensor_msgs::image_encodings::RGB8;
        }
        else
        {
            ROS_WARN_STREAM("Current pixel format is not supported yet: " <<
                v4l2_fourcc::toString(cur_data_format_.format_) << " " << cur_data_format_.format_);
        }

        return img;
    }

    std::string V4l2CameraDevice::getCameraName() const
    {
        auto name = std::string((char*)(capabilities_.card));
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        std::replace(name.begin(), name.end(), ' ', '_');

        return name;
    }

    v4l2_camera::Control V4l2CameraDevice::queryControl(uint32_t Id, bool Silent/* = false*/)
    {
        v4l2_queryctrl queryctrl;
        queryctrl.id = Id;
        if (xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) != 0)
        {
            if (!Silent)
            {
                ROS_ERROR_STREAM("Failed querying control with ID: " << queryctrl.id << " - " <<
                    strerror(errno) << " (" << errno << ")");
            }
            return {};
        }

        std::map<int, std::string> menuItems;
        if (queryctrl.type == (unsigned)ControlType::MENU)
        {
            v4l2_querymenu querymenu;
            querymenu.id = queryctrl.id;

            // Query all enum values
            for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++)
            {
                querymenu.index = i;
                if (xioctl(fd_, VIDIOC_QUERYMENU, &querymenu) == 0)
                {
                    menuItems[i] = (const char *)querymenu.name;
                }
            }
        }

        Control control;
        control.id_ = queryctrl.id;
        control.name_ = std::string{reinterpret_cast<char *>(queryctrl.name)};
        control.type_ = static_cast<ControlType>(queryctrl.type);
        control.minimum_ = queryctrl.minimum;
        control.maximum_ = queryctrl.maximum;
        control.default_value_ = queryctrl.default_value;
        control.menu_items_map_ = std::move(menuItems);
        control.disabled_ = (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) != 0;
        control.inactive_ = (queryctrl.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;

        return control;
    }

    int32_t V4l2CameraDevice::getControlValue(uint32_t Id) const
    {
        v4l2_control ctrl;
        ctrl.id = Id;
        if (-1 == xioctl(fd_, VIDIOC_G_CTRL, &ctrl))
        {
            ROS_ERROR_STREAM("Failed getting value for control " << ctrl.id << ": " <<
                strerror(errno) << " (" << errno << "); returning 0!");

            return 0;
        }
        return ctrl.value;
    }

    void V4l2CameraDevice::listImageFormats()
    {
        image_formats_.clear();

        struct v4l2_fmtdesc fmtDesc;
        fmtDesc.index = 0;
        fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (xioctl(fd_, VIDIOC_ENUM_FMT, &fmtDesc) == 0)
        {
            image_formats_.emplace_back(fmtDesc);
            fmtDesc.index++;
        }
    }

    void V4l2CameraDevice::listImageSizes()
    {
        image_sizes_.clear();
        struct v4l2_frmsizeenum frameSizeEnum;
        // Supported sizes can be different per format
        for (auto const& format : image_formats_)
        {
            frameSizeEnum.index = 0;
            frameSizeEnum.pixel_format = format.format_;
            if (-1 == xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frameSizeEnum))
            {
                ROS_ERROR_STREAM("Failed listing frame size " << strerror(errno) << " (" << errno << ")");
                continue;
            }

            switch (frameSizeEnum.type)
            {
            case V4L2_FRMSIZE_TYPE_DISCRETE:
                image_sizes_[format.format_] = listDiscreteImageSizes(frameSizeEnum);
                break;
            case V4L2_FRMSIZE_TYPE_STEPWISE:
                image_sizes_[format.format_] = listStepwiseImageSizes(frameSizeEnum);
                break;
            case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                image_sizes_[format.format_] = listContinuousImageSizes(frameSizeEnum);
                break;
            default:
                ROS_WARN_STREAM("Frame size type not supported: " << frameSizeEnum.type);
                continue;
            }
        }
    }

    void V4l2CameraDevice::listControls()
    {
        controls_.clear();

        auto query_id = V4L2_CID_BASE;
        while (true)
        {
            auto control = queryControl(query_id, true);
            if (control.id_ == 0)
            {
                break;
            }

            if (control.disabled_)
            {
                query_id = control.id_ |= V4L2_CTRL_FLAG_NEXT_CTRL;
                continue;
            }

            controls_.push_back(control);

            // Get ready to query next item
            query_id = control.id_ |= V4L2_CTRL_FLAG_NEXT_CTRL;
        }
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listDiscreteImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        ImageSizesVector sizes;
        do
        {
            sizes.emplace_back(std::make_pair(FrameSizeEnum.discrete.width, FrameSizeEnum.discrete.height));
            FrameSizeEnum.index++;
        } while (xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &FrameSizeEnum) == 0);

        return std::make_pair(ImageSizeType::DISCRETE, std::move(sizes));
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listStepwiseImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        // Three entries: min size, max size and stepsize
        auto sizes = ImageSizesVector(3);
        sizes[0] = std::make_pair(FrameSizeEnum.stepwise.min_width, FrameSizeEnum.stepwise.min_height);
        sizes[1] = std::make_pair(FrameSizeEnum.stepwise.max_width, FrameSizeEnum.stepwise.max_height);
        sizes[2] = std::make_pair(FrameSizeEnum.stepwise.step_width, FrameSizeEnum.stepwise.step_height);

        return std::make_pair(ImageSizeType::STEPWISE, std::move(sizes));
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listContinuousImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        // Two entries: min size and max size, stepsize is implicitly 1
        auto sizes = ImageSizesVector(2);
        sizes[0] = std::make_pair(FrameSizeEnum.stepwise.min_width, FrameSizeEnum.stepwise.min_height);
        sizes[1] = std::make_pair(FrameSizeEnum.stepwise.max_width, FrameSizeEnum.stepwise.max_height);

        return std::make_pair(ImageSizeType::CONTINUOUS, std::move(sizes));
    }

    bool V4l2CameraDevice::initMemoryMapping()
    {
        v4l2_requestbuffers req;
        req.count = 4; // request 4 buffers
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd_, VIDIOC_REQBUFS, &req);
        if (req.count < 2)
        {
            ROS_ERROR("insufficient buffer memory");
            return false;
        }

        buffers_.resize(req.count);
        for (auto i = 0u; i < req.count; ++i)
        {
            v4l2_buffer buf;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            xioctl(fd_, VIDIOC_QUERYBUF, &buf);

            buffers_[i].index_ = buf.index;
            buffers_[i].length_ = buf.length;
            buffers_[i].start_ = (uint8_t*)mmap(NULL /* start anywhere */,
                                                buf.length,
                                                PROT_READ | PROT_WRITE /* required */,
                                                MAP_SHARED /* recommended */,
                                                fd_, buf.m.offset);

            if (MAP_FAILED == buffers_[i].start_)
            {
                ROS_ERROR("Failed mapping device memory");
                return false;
            }
        }

        return true;
    }
}  // namespace v4l2_camera
