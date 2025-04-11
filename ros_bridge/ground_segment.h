#ifndef GROUND_SEGMENTER_H_
#define GROUND_SEGMENTER_H_

#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <utility>
#include <cstdint>

#include "utils/pose.h"
#include "utils/cloud.h"
#include "utils/radians.h"

#include "projections/projection_params.h"

#include "ground_removal/depth_ground_remover.h"

// #include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace depth_clustering;
using namespace sensor_msgs::msg;

class GroundSegmenter : public rclcpp::Node {
    public:
        GroundSegmenter();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;

        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


        Cloud::Ptr RosCloudToCloud(const PointCloud2::ConstPtr& msg);

        DepthGroundRemover depth_ground_remover;

};
#endif // GROUND_SEGMENTER_H_
