/**
 * Point Cloud Preprocessor for MID360 LiDAR
 *
 * Applies filtering and segmentation pipeline:
 * 1. Range filter
 * 2. Voxel grid downsampling
 * 3. Statistical outlier removal
 * 4. Ground plane segmentation
 * 5. TF transform to base_link
 *
 * Outputs:
 * - /cloud/obstacles: Obstacle points for navigation
 * - /cloud/ground: Ground points for visualization
 * - /cloud/filtered: All filtered points before segmentation
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

class PointCloudPreprocessor : public rclcpp::Node
{
public:
  PointCloudPreprocessor() : Node("pointcloud_preprocessor")
  {
    // Declare parameters
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_obstacle_topic", "/cloud/obstacles");
    this->declare_parameter("output_ground_topic", "/cloud/ground");
    this->declare_parameter("output_filtered_topic", "/mid360/points_filtered");

    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("source_frame", "mid360_lidar");
    this->declare_parameter("transform_timeout", 0.1);

    // Range filter
    this->declare_parameter("range_filter.enabled", true);
    this->declare_parameter("range_filter.min_range", 0.5);
    this->declare_parameter("range_filter.max_range", 30.0);
    this->declare_parameter("range_filter.min_height", -0.3);
    this->declare_parameter("range_filter.max_height", 2.5);

    // Voxel grid
    this->declare_parameter("voxel_grid.enabled", true);
    this->declare_parameter("voxel_grid.leaf_size", 0.05);

    // Outlier removal
    this->declare_parameter("outlier_removal.enabled", true);
    this->declare_parameter("outlier_removal.mean_k", 50);
    this->declare_parameter("outlier_removal.stddev_mul", 1.0);

    // Ground segmentation
    this->declare_parameter("ground_segmentation.enabled", true);
    this->declare_parameter("ground_segmentation.ransac_max_iterations", 100);
    this->declare_parameter("ground_segmentation.ransac_distance_threshold", 0.02);

    this->declare_parameter("verbose", true);

    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_obstacle_topic_ = this->get_parameter("output_obstacle_topic").as_string();
    output_ground_topic_ = this->get_parameter("output_ground_topic").as_string();
    output_filtered_topic_ = this->get_parameter("output_filtered_topic").as_string();

    target_frame_ = this->get_parameter("target_frame").as_string();
    verbose_ = this->get_parameter("verbose").as_bool();

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publishers
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_obstacle_topic_, 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_ground_topic_, 10);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_filtered_topic_, 10);

    // Create subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&PointCloudPreprocessor::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Point Cloud Preprocessor initialized");
    RCLCPP_INFO(this->get_logger(), "  Input: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output obstacles: %s", output_obstacle_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output ground: %s", output_ground_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert ROS message to PCL
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Processing cloud with %zu points", cloud->size());
    }

    // 1. Range filter
    if (this->get_parameter("range_filter.enabled").as_bool()) {
      cloud = rangeFilter(cloud);
    }

    // 2. Voxel grid downsampling
    if (this->get_parameter("voxel_grid.enabled").as_bool()) {
      cloud = voxelGridFilter(cloud);
    }

    // 3. Statistical outlier removal
    if (this->get_parameter("outlier_removal.enabled").as_bool()) {
      cloud = outlierRemoval(cloud);
    }

    // 4. Transform to target frame (base_link)
    PointCloud::Ptr transformed_cloud(new PointCloud);
    if (!transformPointCloud(cloud, transformed_cloud, msg->header)) {
      RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud, using original frame");
      transformed_cloud = cloud;
    } else {
      cloud = transformed_cloud;
    }

    // Publish filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = target_frame_;
    filtered_pub_->publish(filtered_msg);

    // 5. Ground segmentation
    if (this->get_parameter("ground_segmentation.enabled").as_bool()) {
      PointCloud::Ptr obstacles(new PointCloud);
      PointCloud::Ptr ground(new PointCloud);
      groundSegmentation(cloud, obstacles, ground);

      // Publish obstacle cloud
      sensor_msgs::msg::PointCloud2 obstacle_msg;
      pcl::toROSMsg(*obstacles, obstacle_msg);
      obstacle_msg.header.stamp = msg->header.stamp;
      obstacle_msg.header.frame_id = target_frame_;
      obstacle_pub_->publish(obstacle_msg);

      // Publish ground cloud
      sensor_msgs::msg::PointCloud2 ground_msg;
      pcl::toROSMsg(*ground, ground_msg);
      ground_msg.header.stamp = msg->header.stamp;
      ground_msg.header.frame_id = target_frame_;
      ground_pub_->publish(ground_msg);

      if (verbose_) {
        RCLCPP_INFO(this->get_logger(),
          "  Obstacles: %zu points, Ground: %zu points",
          obstacles->size(), ground->size());
      }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "  Processing time: %ld ms", duration);
    }
  }

  PointCloud::Ptr rangeFilter(const PointCloud::Ptr& cloud)
  {
    PointCloud::Ptr filtered(new PointCloud);
    pcl::PassThrough<PointT> pass;

    float min_range = this->get_parameter("range_filter.min_range").as_double();
    float max_range = this->get_parameter("range_filter.max_range").as_double();
    float min_height = this->get_parameter("range_filter.min_height").as_double();
    float max_height = this->get_parameter("range_filter.max_height").as_double();

    // First filter: Radial distance (3D distance from origin)
    filtered->reserve(cloud->size());
    for (const auto& point : cloud->points) {
      float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (range >= min_range && range <= max_range) {
        filtered->push_back(point);
      }
    }

    // Second filter: Height (Z axis)
    PointCloud::Ptr height_filtered(new PointCloud);
    pass.setInputCloud(filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*height_filtered);

    return height_filtered;
  }

  PointCloud::Ptr voxelGridFilter(const PointCloud::Ptr& cloud)
  {
    pcl::VoxelGrid<PointT> voxel;
    PointCloud::Ptr filtered(new PointCloud);

    voxel.setInputCloud(cloud);
    float leaf_size = this->get_parameter("voxel_grid.leaf_size").as_double();
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*filtered);

    return filtered;
  }

  PointCloud::Ptr outlierRemoval(const PointCloud::Ptr& cloud)
  {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    PointCloud::Ptr filtered(new PointCloud);

    sor.setInputCloud(cloud);
    int mean_k = this->get_parameter("outlier_removal.mean_k").as_int();
    double stddev = this->get_parameter("outlier_removal.stddev_mul").as_double();
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev);
    sor.filter(*filtered);

    return filtered;
  }

  void groundSegmentation(const PointCloud::Ptr& cloud,
                         PointCloud::Ptr& obstacles,
                         PointCloud::Ptr& ground)
  {
    // RANSAC plane segmentation
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    int max_iter = this->get_parameter("ground_segmentation.ransac_max_iterations").as_int();
    double dist_thresh = this->get_parameter("ground_segmentation.ransac_distance_threshold").as_double();
    seg.setMaxIterations(max_iter);
    seg.setDistanceThreshold(dist_thresh);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract ground and obstacle points
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Extract ground
    extract.setNegative(false);
    extract.filter(*ground);

    // Extract obstacles
    extract.setNegative(true);
    extract.filter(*obstacles);
  }

  bool transformPointCloud(const PointCloud::Ptr& cloud_in,
                          PointCloud::Ptr& cloud_out,
                          const std_msgs::msg::Header& header)
  {
    try {
      // Check if we need to transform
      if (header.frame_id == target_frame_) {
        *cloud_out = *cloud_in;
        return true;
      }

      // Get transform
      geometry_msgs::msg::TransformStamped transform_stamped;
      double timeout = this->get_parameter("transform_timeout").as_double();

      transform_stamped = tf_buffer_->lookupTransform(
        target_frame_,
        header.frame_id,
        header.stamp,
        tf2::durationFromSec(timeout));

      // Convert transform to Eigen
      Eigen::Affine3d transform_eigen = Eigen::Affine3d::Identity();
      transform_eigen.translation() <<
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z;

      Eigen::Quaterniond q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);
      transform_eigen.rotate(q);

      // Transform point cloud
      pcl::transformPointCloud(*cloud_in, *cloud_out, transform_eigen);

      return true;

    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "TF transform failed: %s", ex.what());
      return false;
    }
  }

  // Member variables
  std::string input_topic_;
  std::string output_obstacle_topic_;
  std::string output_ground_topic_;
  std::string output_filtered_topic_;
  std::string target_frame_;
  bool verbose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
