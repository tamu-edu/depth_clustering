// #include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/radians.h"

#include "ground_segment.h"

using std::vector;
using namespace depth_clustering;
using namespace sensor_msgs::msg;
using namespace std::placeholders;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundSegmenter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


GroundSegmenter::GroundSegmenter() : Node("ground_segmenter_node") {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&GroundSegmenter::pointcloud_callback, this, _1));

    no_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/seg/no_ground", 10);

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/seg/ground", 10);

    auto proj_params_ptr = ProjectionParams::FLEXX2();
    Radians ground_remove_angle = 7_deg;
    int smooth_window_size = 9;
    depth_ground_remover = DepthGroundRemover(*proj_params_ptr, ground_remove_angle, smooth_window_size);
}


// std::tuple<Cloud, Cloud> DepthGroundRemover::OnNewObjectReceived(const Cloud& cloud, const int) {
void GroundSegmenter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  Cloud::Ptr cloud_ptr = RosCloudToCloud(msg);
  auto [no_ground_cloud, ground_cloud] = depth_ground_remover.OnNewObjectReceived(*cloud_ptr.get(), 0);

  sensor_msgs::msg::PointCloud2 no_ground_cloud_msg;
  auto no_ground_cloud_pcl = *no_ground_cloud.ToPcl().get();
  pcl::PCLPointCloud2 no_ground_cloud_pc2;
  pcl::toPCLPointCloud2(no_ground_cloud_pcl, no_ground_cloud_pc2);
  pcl_conversions::fromPCL(no_ground_cloud_pc2, no_ground_cloud_msg);
  no_ground_cloud_msg.header.stamp = this->get_clock()->now();
  no_ground_cloud_msg.header.frame_id = "no ground pointcloud";

  sensor_msgs::msg::PointCloud2 ground_cloud_msg;
  auto ground_cloud_pcl = *ground_cloud.ToPcl().get();
  pcl::PCLPointCloud2 ground_cloud_pc2;
  pcl::toPCLPointCloud2(ground_cloud_pcl, ground_cloud_pc2);
  pcl_conversions::fromPCL(ground_cloud_pc2, ground_cloud_msg);
  ground_cloud_msg.header.stamp = this->get_clock()->now();
  ground_cloud_msg.header.frame_id = "ground pointcloud";

  no_ground_pub_->publish(no_ground_cloud_msg);
  ground_pub_->publish(ground_cloud_msg);
}

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

Cloud::Ptr GroundSegmenter::RosCloudToCloud(
    const PointCloud2::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[4].offset;

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  return boost::make_shared<Cloud>(cloud);
}

