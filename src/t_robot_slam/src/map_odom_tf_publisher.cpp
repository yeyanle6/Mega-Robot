// Legacy FAST-LIO helper node retained until RTAB-Map pipeline replaces it.
// Converts FAST-LIO's camera_init/body frames into a map->odom transform.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapOdomTfPublisher : public rclcpp::Node
{
public:
    MapOdomTfPublisher() : Node("map_odom_tf_publisher")
    {
        // 创建TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 创建TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 订阅FAST-LIO的里程计话题
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&MapOdomTfPublisher::odom_callback, this, std::placeholders::_1));
        
        // 创建定时器发布map->odom变换
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&MapOdomTfPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Map-Odom TF Publisher started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 保存最新的里程计信息
        latest_odom_ = *msg;
        odom_received_ = true;
    }
    
    void timer_callback()
    {
        if (!odom_received_) {
            return;
        }
        
        try {
            // 获取camera_init到body的变换
            geometry_msgs::msg::TransformStamped camera_to_body;
            camera_to_body = tf_buffer_->lookupTransform(
                "camera_init", "body", tf2::TimePointZero);
            
            // 创建map到odom的变换
            geometry_msgs::msg::TransformStamped map_to_odom;
            map_to_odom.header.stamp = camera_to_body.header.stamp; // 使用相同的时间戳
            map_to_odom.header.frame_id = "map";
            map_to_odom.child_frame_id = "odom";
            
            // 将camera_init坐标系映射到map坐标系
            // 将body坐标系映射到odom坐标系
            map_to_odom.transform.translation.x = camera_to_body.transform.translation.x;
            map_to_odom.transform.translation.y = camera_to_body.transform.translation.y;
            map_to_odom.transform.translation.z = camera_to_body.transform.translation.z;
            map_to_odom.transform.rotation = camera_to_body.transform.rotation;
            
            // 发布变换
            tf_broadcaster_->sendTransform(map_to_odom);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry latest_odom_;
    bool odom_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
