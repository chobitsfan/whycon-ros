#include "whycon_ros/whycon_ros_node.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>


namespace whycon_ros
{

// parameters that may be changed at runtime, applied through CWhycon::updateConfiguration
static const char *kTunableParams[] = {
    "identify", "circle_diameter", "num_markers", "min_size", "field_length", "field_width",
    "initial_circularity_tolerance", "final_circularity_tolerance", "area_ratio_tolerance",
    "center_distance_tolerance_ratio", "center_distance_tolerance_abs"
};

static rcl_interfaces::msg::ParameterDescriptor describe(const std::string &text, double from, double to)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = text;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    desc.floating_point_range.push_back(range);
    return desc;
}

static rcl_interfaces::msg::ParameterDescriptor describe(const std::string &text, int from, int to)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = text;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = from;
    range.to_value = to;
    desc.integer_range.push_back(range);
    return desc;
}

static rcl_interfaces::msg::ParameterDescriptor describe(const std::string &text, bool read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = text;
    desc.read_only = read_only;
    return desc;
}

void CWhyconROSNode::loadCameraCalib(const std::string &path)
{
    std::ifstream file(path);
    if(!file)
        throw std::runtime_error("cannot open camera calibration file '" + path + "'");

    std::vector<std::vector<float>> rows;
    std::string line;
    while(std::getline(file, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        std::vector<float> row;
        float value;
        while(ss >> value)
            row.push_back(value);
        if(!ss.eof())
            throw std::runtime_error("invalid number in camera calibration file '" + path + "': " + line);
        if(!row.empty())
            rows.push_back(row);
    }

    if(rows.size() != 2 || rows[0].size() != 4 || rows[1].size() != 5)
        throw std::runtime_error("camera calibration file '" + path +
                                 "' must have 2 lines: 'fx, fy, cx, cy' and 'k1, k2, p1, p2, k3'");

    const float fx = rows[0][0], fy = rows[0][1], cx = rows[0][2], cy = rows[0][3];
    intrinsic_mat_ = {fx, 0.0f, cx,
                      0.0f, fy, cy,
                      0.0f, 0.0f, 1.0f};
    distortion_coeffs_ = rows[1];

    RCLCPP_INFO(get_logger(), "Camera calibration '%s': fx %f fy %f cx %f cy %f, k1 %f k2 %f p1 %f p2 %f k3 %f",
        path.c_str(), fx, fy, cx, cy, rows[1][0], rows[1][1], rows[1][2], rows[1][3], rows[1][4]);
}

void CWhyconROSNode::updateConfiguration()
{
    identify_ = get_parameter("identify").as_bool();
    circle_diameter_ = get_parameter("circle_diameter").as_double();

    whycon_.updateConfiguration(
        identify_, circle_diameter_,
        get_parameter("num_markers").as_int(),
        get_parameter("min_size").as_int(),
        get_parameter("field_length").as_double(),
        get_parameter("field_width").as_double(),
        get_parameter("initial_circularity_tolerance").as_double(),
        get_parameter("final_circularity_tolerance").as_double(),
        get_parameter("area_ratio_tolerance").as_double(),
        get_parameter("center_distance_tolerance_ratio").as_double(),
        get_parameter("center_distance_tolerance_abs").as_double()
        );
}

void CWhyconROSNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if(msg->encoding != "mono8" && msg->encoding != "8UC1")
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "Unsupported image encoding '%s', expected mono8.", msg->encoding.c_str());
        return;
    }
    if(msg->step < msg->width || msg->data.size() < static_cast<size_t>(msg->step) * msg->height)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Malformed image message.");
        return;
    }

    // WhyCon expects tightly packed rows, strip any row padding
    const unsigned char *data = msg->data.data();
    if(msg->step != msg->width)
    {
        packed_.resize(static_cast<size_t>(msg->width) * msg->height);
        for(uint32_t r = 0; r < msg->height; r++)
            std::memcpy(&packed_[r * msg->width], &msg->data[r * msg->step], msg->width);
        data = packed_.data();
    }
    image_->updateImage(const_cast<unsigned char*>(data), msg->width, msg->height, 1);

    whycon_.processImage(image_, whycon_detections_);

    visualization_msgs::msg::MarkerArray marker_array;
    std::vector<geometry_msgs::msg::TransformStamped> transform_array;

    for(size_t i = 0; i < whycon_detections_.size(); i++)
    {
        const whycon::SMarker &detection = whycon_detections_[i];

        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "whycon";
        // without identification every ID is -1, so fall back to the detection index to keep IDs unique
        marker.id = identify_ ? detection.seg.ID : static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::MODIFY;

        marker.pose.position.x = detection.obj.x;
        marker.pose.position.y = detection.obj.y;
        marker.pose.position.z = detection.obj.z;
        marker.pose.orientation.x = detection.obj.qx;
        marker.pose.orientation.y = detection.obj.qy;
        marker.pose.orientation.z = detection.obj.qz;
        marker.pose.orientation.w = detection.obj.qw;

        marker.scale.x = 0.01;  // meters
        marker.scale.y = circle_diameter_;
        marker.scale.z = circle_diameter_;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker.frame_locked = true;

        // image-space data that has no dedicated field in visualization_msgs/Marker
        char text[128];
        std::snprintf(text, sizeof(text), "u=%.2f v=%.2f size=%d angle=%.4f",
            detection.seg.x, detection.seg.y, detection.seg.size, detection.obj.angle);
        marker.text = text;

        marker_array.markers.push_back(marker);

        if(identify_ && publish_tf_)
        {
            geometry_msgs::msg::TransformStamped transform_stamped;

            transform_stamped.header = msg->header;
            transform_stamped.child_frame_id = "marker_" + std::to_string(detection.seg.ID);
            transform_stamped.transform.translation.x = detection.obj.x;
            transform_stamped.transform.translation.y = detection.obj.y;
            transform_stamped.transform.translation.z = detection.obj.z;
            transform_stamped.transform.rotation = marker.pose.orientation;

            transform_array.push_back(transform_stamped);
        }
    }

    // publishing detected markers
    if(!marker_array.markers.empty())
    {
        markers_pub_->publish(marker_array);

        if(!transform_array.empty())
            tf_broad_->sendTransform(transform_array);
    }

    if(use_gui_)
    {
        sensor_msgs::msg::Image out;
        out.header = msg->header;
        out.height = image_->height_;
        out.width = image_->width_;
        out.encoding = "mono8";
        out.is_bigendian = 0;
        out.step = image_->width_;
        out.data.assign(image_->data_, image_->data_ + image_->size_);
        img_pub_->publish(out);
    }

    whycon_detections_.clear();
}

CWhyconROSNode::CWhyconROSNode() :
    rclcpp::Node("whycon")
{
    // start-up parameters
    std::string camera_calib = declare_parameter("camera_calib", std::string(""),
        describe("CSV file with 'fx, fy, cx, cy' on line 1 and 'k1, k2, p1, p2, k3' on line 2", true));
    use_gui_ = declare_parameter("use_gui", false, describe("Publish processed_image with drawn detections", true));
    publish_tf_ = declare_parameter("pub_tf", false, describe("Publish TF from camera to marker frames", true));
    int id_bits = declare_parameter("id_bits", 4, describe("Number of encoded ID bits", true));
    int id_samples = declare_parameter("id_samples", 360, describe("Number of samples to identify ID", true));
    int hamming_dist = declare_parameter("hamming_dist", 1, describe("Hamming distance of ID code", true));
    std::string calib_path = declare_parameter("calib_file", std::string(""),
        describe("Custom coordinate system calibration file to load on start up", true));
    int coords_method = declare_parameter("coords_method", 0,
        describe("Coordinates applied with calib_file. 0 - none, 1 - 2D, 2 - 3D", true));

    // runtime-tunable parameters (formerly dynamic_reconfigure)
    declare_parameter("identify", true, describe("Whether to identify IDs"));
    circle_diameter_ = declare_parameter("circle_diameter", 0.1556, describe("Marker's outer diameter [m]", 0.0, 100.0));
    int num_markers = declare_parameter("num_markers", 5, describe("Number of markers to track", 0, 1000));
    declare_parameter("min_size", 30, describe("Min size of marker [px]", 10, 1000));
    declare_parameter("field_length", 1.0, describe("Distance of markers in x-axis in custom coordinate system", 0.0, 100.0));
    declare_parameter("field_width", 1.0, describe("Distance of markers in y-axis in custom coordinate system", 0.0, 100.0));
    declare_parameter("initial_circularity_tolerance", 100.0, describe("Initial circularity test tolerance [%]", 0.0, 100.0));
    declare_parameter("final_circularity_tolerance", 2.0, describe("Final circularity test tolerance [%]", 0.0, 100.0));
    declare_parameter("area_ratio_tolerance", 40.0, describe("Tolerance of black and white area ratios [%]", 0.0, 200.0));
    declare_parameter("center_distance_tolerance_ratio", 10.0, describe("Concentricity test ratio [%]", 0.0, 100.0));
    declare_parameter("center_distance_tolerance_abs", 5.0, describe("Concentricity test absolute [px]", 0.0, 25.0));

    if(camera_calib.empty())
        throw std::runtime_error("parameter 'camera_calib' (path to the camera calibration CSV) is required");
    loadCameraCalib(camera_calib);

    int default_width = 640;
    int default_height = 480;
    image_ = new whycon::CRawImage(default_width, default_height, 1);
    whycon_.init(circle_diameter_, use_gui_, id_bits, id_samples, hamming_dist, num_markers, default_width, default_height);
    whycon_.updateCameraInfo(intrinsic_mat_, distortion_coeffs_);
    updateConfiguration();

    param_cb_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
            for(const rclcpp::Parameter &p : params)
            {
                if(std::find_if(std::begin(kTunableParams), std::end(kTunableParams),
                    [&p](const char *name) { return p.get_name() == name; }) != std::end(kTunableParams))
                {
                    RCLCPP_INFO(get_logger(), "Reconfigure: %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
                    updateConfiguration();
                    return;
                }
            }
        });

    if(publish_tf_)
        tf_broad_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if(calib_path.size() > 0)
    {
        try
        {
            whycon_.loadCalibration(calib_path);
            whycon_.setCoordinates(static_cast<whycon::ETransformType>(coords_method));
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Calibration file '%s' could not be loaded. Using camera centric coordinates.", calib_path.c_str());
        }
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Calibration file path empty. Using camera centric coordinates.");
    }

    // advertise markers and GUI visualization, subscribe to the camera
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1);
    if(use_gui_)
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("processed_image", 1);
    img_sub_ = create_subscription<sensor_msgs::msg::Image>("mono_left", rclcpp::SensorDataQoS(),
        std::bind(&CWhyconROSNode::imageCallback, this, std::placeholders::_1));
}

CWhyconROSNode::~CWhyconROSNode()
{
    delete image_;
}

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    int ret = 0;
    try
    {
        rclcpp::spin(std::make_shared<whycon_ros::CWhyconROSNode>());
    }
    catch(const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("whycon"), "%s", e.what());
        ret = 1;
    }

    rclcpp::shutdown();
    return ret;
}
