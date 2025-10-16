// Legacy FAST-LIO helper node retained until RTAB-Map pipeline replaces it.
// Broadcasts map->odom using the FAST-LIO camera_init frame as reference.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapOdomTfPublisherV2 : public rclcpp::Node
{
public:
    MapOdomTfPublisherV2() : Node("map_odom_tf_publisher_v2")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to FAST-LIO's odometry output to get the camera_init -> body transform
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&MapOdomTfPublisherV2::odom_callback, this, std::placeholders::_1));

        // Timer to periodically publish the map -> odom transform
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MapOdomTfPublisherV2::timer_callback, this));

        // Initialize the map to odom transform as identity
        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id = "odom";
        map_to_odom_.transform.translation.x = 0.0;
        map_to_odom_.transform.translation.y = 0.0;
        map_to_odom_.transform.translation.z = 0.0;
        map_to_odom_.transform.rotation.x = 0.0;
        map_to_odom_.transform.rotation.y = 0.0;
        map_to_odom_.transform.rotation.z = 0.0;
        map_to_odom_.transform.rotation.w = 1.0;
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Save the latest odometry information
        latest_odom_ = *msg;
        odom_received_ = true;
    }

    void timer_callback()
    {
        if (!odom_received_) {
            return;
        }

        try {
            // Get the transform from camera_init to body (from FAST-LIO)
            geometry_msgs::msg::TransformStamped camera_to_body;
            camera_to_body = tf_buffer_->lookupTransform(
                "camera_init", "body", tf2::TimePointZero);

            // Create the map to odom transform
            // Map the camera_init coordinate system to the map coordinate system
            // Map the body coordinate system to the odom coordinate system
            map_to_odom_.header.stamp = this->now(); // Use current time instead of transform time
            map_to_odom_.transform.translation.x = camera_to_body.transform.translation.x;
            map_to_odom_.transform.translation.y = camera_to_body.transform.translation.y;
            map_to_odom_.transform.translation.z = camera_to_body.transform.translation.z;
            map_to_odom_.transform.rotation = camera_to_body.transform.rotation;

            // Publish the transform
            tf_broadcaster_->sendTransform(map_to_odom_);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry latest_odom_;
    bool odom_received_ = false;
    geometry_msgs::msg::TransformStamped map_to_odom_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomTfPublisherV2>());
    rclcpp::shutdown();
    return 0;
}
