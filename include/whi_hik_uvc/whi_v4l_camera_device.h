/******************************************************************
v4l camera device under ROS 1

Features:
- v2l camera
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-15: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <sensor_msgs/Image.h>

#include <linux/videodev2.h>
#include <string>
#include <vector>
#include <map>

namespace v4l2_camera
{
    /// Type of camera control
    enum class ControlType : unsigned
    {
        INT        = 1,
        BOOL       = 2,
        MENU       = 3,
        BUTTON     = 4,
        INT64      = 5,
        CTRL_CLASS = 6,
        STRING     = 7,
        BITMASK    = 8
    };

    struct Control
    {
        /// Identifies the control, set by the application
        unsigned id_;
        /// Human readable name
        std::string name_;
        /// Type of control
        ControlType type_;
        /// Minimum value, inclusive
        int minimum_;
        /// Maximum value, inclusive
        int maximum_;
        /// The default value of of an integer, boolean, bitmask, menu or integer menu control
        int default_value_;
        /// Menu item names by index. Empty if this is not a menu control
        std::map<int, std::string> menu_items_map_;
        /// Whether the control is disabled, meaning it should be ignored
        bool disabled_;
        /// Whether the control is set to inactive, e.g. when it is automatically controlled
        bool inactive_;
    };

    struct PixelFormat
    {
        PixelFormat() = default;
        explicit PixelFormat(const v4l2_pix_format& Pf)
            : width_(Pf.width), height_(Pf.height), format_(Pf.pixelformat), bytes_per_line_(Pf.bytesperline)
            , image_byte_size_(Pf.sizeimage) {};

        /// Image width in pixels
        unsigned width_;
        /// Image height in pixels
        unsigned height_;
        /// The pixel format or type of compression, set by the application
        unsigned format_;
        /// Distance in bytes between the leftmost pixels in two adjacent lines
        unsigned bytes_per_line_;
        /// Size in bytes of the buffer to hold a complete image, set by the driver
        unsigned image_byte_size_;
    };

    struct ImageFormat
    {
        explicit ImageFormat(const v4l2_fmtdesc& Fd)
            : index_(Fd.index), type_(Fd.type), flags_(Fd.flags)
            , description_((const char*)Fd.description), format_(Fd.pixelformat) {}

        /// Number of the format in the enumeration, set by the application
        unsigned index_;
        /// Type of the data stream, set by the application, probably to V4L2_BUF_TYPE_VIDEO_CAPTURE
        unsigned type_;
        /// Image format description flags. Options:
        /// V4L2_FMT_FLAG_COMPRESSED and/or V4L2_FMT_FLAG_EMULATED
        unsigned flags_;
        /// Human readable description of the format
        std::string description_;
        /// The image format identifier as computed by the v4l2_fourcc() macro
        unsigned format_;
    };

    struct v4l2_fourcc
    {
        inline static std::string toString(unsigned Fourcc)
        {
            char chars[5];
            for (unsigned i = 0; i < 4; ++i)
            {
                chars[i] = ((Fourcc >> (i * 8)) & 0xFF);
            }
            chars[4] = 0;
            return std::string{chars};
        }
    };

    class V4l2CameraDevice
    {
    public:
        explicit V4l2CameraDevice(const std::string& Device)
            : device_(Device) {};

        bool open();
        bool start();
        bool stop();
        sensor_msgs::Image::Ptr capture();

        // Query properties and current state of a control
        Control queryControl(uint32_t Id, bool Silent = false);

        // Get all pre-queried controls
        auto const & getControls() const {return controls_;}

        // Get current control value
        int32_t getControlValue(uint32_t Id) const;

        // Attempt to set current control value
        bool setControlValue(uint32_t id, int32_t value);

        // Types used to describe available image sizes
        enum class ImageSizeType
        {
            DISCRETE,
            STEPWISE,
            CONTINUOUS
        };
        using ImageSizesVector = std::vector<std::pair<uint16_t, uint16_t>>;
        using ImageSizesDescription = std::pair<ImageSizeType, ImageSizesVector>;

        const auto& getImageFormats() const { return image_formats_; }
        const auto& getImageSizes() const { return image_sizes_; }
        const auto& getCurrentDataFormat() const { return cur_data_format_; }
        bool requestDataFormat(PixelFormat const & format);

        std::string getCameraName();

    private:
        // Requests and stores all formats available for this camera
        void listImageFormats();
        // Requests and stores all frame sizes available for this camera
        void listImageSizes();
        // Requests and stores all controls available for this camera
        void listControls();

        ImageSizesDescription listDiscreteImageSizes(v4l2_frmsizeenum FrameSizeEnum);
        ImageSizesDescription listStepwiseImageSizes(v4l2_frmsizeenum FrameSizeEnum);
        ImageSizesDescription listContinuousImageSizes(v4l2_frmsizeenum FrameSizeEnum);

        // Set up memory mapping to buffers
        bool initMemoryMapping();

    private:
        /// Image buffer
        struct Buffer
        {
            unsigned index_;
            uint8_t* start_;
            size_t length_;
        };

        std::string device_;
        int fd_;

        v4l2_capability capabilities_;
        std::vector<ImageFormat> image_formats_;
        std::map<unsigned, ImageSizesDescription> image_sizes_;
        std::vector<Control> controls_;

        PixelFormat cur_data_format_;

        std::vector<Buffer> buffers_;
    };
}  // namespace v4l2_camera
