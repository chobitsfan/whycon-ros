#ifndef WHYCON_ROS_CWHYCONROSNODE_H
#define WHYCON_ROS_CWHYCONROSNODE_H

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "whycon/whycon.h"


namespace whycon_ros
{

class CWhyconROSNode : public rclcpp::Node
{

    public:
        CWhyconROSNode();

        ~CWhyconROSNode();

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        // apply the runtime-tunable parameters (former dynamic_reconfigure) to WhyCon
        void updateConfiguration();

        // read "fx, fy, cx, cy" and "k1, k2, p1, p2, k3" lines from a CSV file
        void loadCameraCalib(const std::string &path);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;              // mono8 image subscriber
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;                 // image publisher for GUI
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;    // publisher of detected markers
        rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;

        bool use_gui_;          // publish images with drawn detections?
        bool identify_;
        bool publish_tf_;
        double circle_diameter_;  // marker diameter [m]

        whycon::CWhycon whycon_;        // WhyCon instance
        whycon::CRawImage *image_;      // image wrapper for WhyCon
        std::vector<unsigned char> packed_;     // row-packing buffer for padded images

        std::vector<whycon::SMarker> whycon_detections_;    // array of detected markers

        std::vector<float> intrinsic_mat_;        // intrinsic matrix from the CSV file
        std::vector<float> distortion_coeffs_;    // distortion parameters from the CSV file

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
};

}


#endif
