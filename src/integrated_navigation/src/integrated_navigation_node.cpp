#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h" 
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "integrated_navigation/msg/integrated_data.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#define _USE_MATH_DEFINES

using std::placeholders::_1;

class IntegratedNavigationNode : public rclcpp::Node
{
public:
    IntegratedNavigationNode() : Node("integrated_navigation_node")
    {
        // 初始化TF2相关组件
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 初始化相机到雷达的外参 (camera_link → lidar)
        tf2::Matrix3x3 camera_to_lidar_rot_mat;
        camera_to_lidar_rot_mat.setValue(
            -0.000192,  0.005236, -0.999986,
            0.997441,  0.071499,  0.000183,
            0.071499, -0.997427, -0.005237);
        
        tf2::Vector3 camera_to_lidar_trans(-0.08, -0.05, -0.03);
        
        // 计算雷达到相机的变换 (lidar → camera_link) = inverse(camera_link → lidar)
        tf2::Transform camera_to_lidar_tf(camera_to_lidar_rot_mat, camera_to_lidar_trans);
        lidar_to_camera_tf_ = camera_to_lidar_tf.inverse();
        
        // 初始化雷达到IMU的外参 (lidar → imu)
        tf2::Matrix3x3 lidar_to_imu_rot_mat;
        lidar_to_imu_rot_mat.setValue(
            0.998773, 0.0376798, 0.0321415,
            -0.0385784, 0.998869, 0.0278099,
            -0.0310573, -0.0290158, 0.999096);      

        tf2::Vector3 lidar_to_imu_trans(-0.0290339, 0.0310623, -0.0386066);
        lidar_to_imu_tf_ = tf2::Transform(lidar_to_imu_rot_mat, lidar_to_imu_trans);

        // 初始化订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&IntegratedNavigationNode::odom_callback, this, _1));
        
        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/pointcloud_occupancy_grid", 10, std::bind(&IntegratedNavigationNode::grid_callback, this, _1));
        
        user_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/person_positions", 10, std::bind(&IntegratedNavigationNode::user_pos_callback, this, _1));
        
        // 新增：订阅centroid_point话题
        centroid_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/centroid_point", 10, std::bind(&IntegratedNavigationNode::centroid_callback, this, _1));
        
        // 初始化发布者
        integrated_pub_ = this->create_publisher<integrated_navigation::msg::IntegratedData>(
            "/integrated_navigation_data", 10);
        
        // 发布可视化Marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);
            
        // 新增：发布点云可视化Marker
        centroid_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/centroid_marker", 10);
            
        // 发布最终用户位置
        final_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/user_position", 10);
        
        // 初始化定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&IntegratedNavigationNode::timer_callback, this));
    }

private:
    // 标记常量
    const int USER_MARKER_ID = 0;
    const int ROBOT_MARKER_ID = 1;
    const int CENTROID_MARKER_ID = 2;

    void timer_callback()
    {
        publish_camera_link_transform();
        publish_lidar_to_imu_transform();
        publish_integrated_data();
        publish_centroid_marker(); // 新增：发布质心标记
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = *msg;
        odom_received_ = true;
    }

    void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_grid_ = *msg;
        grid_received_ = true;
    }

    void user_pos_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        try {
            // 1. 修正坐标轴：将z值赋给x（前方距离），x值赋给z（高度），y值取反
            tf2::Vector3 user_pos_local(
                -msg->point.y,    // 前方距离 -> X轴
                msg->point.x,  // 左右位置 -> Y轴（取反）
                msg->point.z);  // 高度 -> Z轴
            
            // 2. 添加旋转修正（解决向右偏移问题）
            tf2::Quaternion adjust_rotation;
            adjust_rotation.setRPY(0, -1.77, 3); // 绕Z轴旋转-0.05弧度（约-2.86度）
            tf2::Transform adjust_tf(adjust_rotation, tf2::Vector3(0,0,0));
            user_pos_local = adjust_tf * user_pos_local;
            
            // 3. 获取小车当前位姿
            if(!odom_received_) {
                RCLCPP_WARN(this->get_logger(), "尚未收到里程计数据");
                return;
            }
            
            // 获取小车位置和方向
            tf2::Vector3 robot_pos(
                latest_odom_.pose.pose.position.x,
                latest_odom_.pose.pose.position.y,
                latest_odom_.pose.pose.position.z);
            
            tf2::Quaternion robot_orient(
                latest_odom_.pose.pose.orientation.x,
                latest_odom_.pose.pose.orientation.y,
                latest_odom_.pose.pose.orientation.z,
                latest_odom_.pose.pose.orientation.w);
            
            // 4. 将用户位置转换到世界坐标系
            tf2::Transform robot_tf(robot_orient, robot_pos);
            tf2::Vector3 user_pos_global = robot_tf * user_pos_local;
            
            // 5. 计算用户朝向（指向机器人）
            tf2::Vector3 direction_to_robot = robot_pos - user_pos_global;
            direction_to_robot.normalize();
            
            // 计算四元数表示的方向
            tf2::Quaternion user_orientation;
            user_orientation.setRPY(
                0,  // 无横滚
                0,  // 无俯仰
                atan2(direction_to_robot.y(), direction_to_robot.x()));  // 偏航角
            
            // 6. 更新最新用户位置和朝向
            latest_user_pos_.header = msg->header;
            latest_user_pos_.header.frame_id = latest_odom_.header.frame_id;
            latest_user_pos_.pose.position.x = user_pos_global.x();
            latest_user_pos_.pose.position.y = user_pos_global.y();
            latest_user_pos_.pose.position.z = user_pos_global.z();
            latest_user_pos_.pose.orientation = tf2::toMsg(user_orientation);
            
            user_pos_received_ = true;
            
            // 发布可视化Marker（现在会显示朝向）
            publish_markers();
            
            // 发布最终用户位置
            geometry_msgs::msg::PointStamped final_pos_msg;
            final_pos_msg.header = latest_user_pos_.header;
            final_pos_msg.point = latest_user_pos_.pose.position;
            final_pos_pub_->publish(final_pos_msg);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "用户位置计算:\n"
                "- 原始本地位置: (%.3f, %.3f, %.3f)\n"
                "- Y轴反转后: (%.3f, %.3f, %.3f)\n"
                "- 旋转修正后: (%.3f, %.3f, %.3f)\n"
                "- 全局坐标系位置: (%.3f, %.3f, %.3f)\n"
                "- 用户朝向: (%.3f, %.3f, %.3f, %.3f)",
                msg->point.z, msg->point.y, msg->point.x,
                msg->point.z, -msg->point.y, msg->point.x,
                user_pos_local.x(), user_pos_local.y(), user_pos_local.z(),
                user_pos_global.x(), user_pos_global.y(), user_pos_global.z(),
                latest_user_pos_.pose.orientation.x,
                latest_user_pos_.pose.orientation.y,
                latest_user_pos_.pose.orientation.z,
                latest_user_pos_.pose.orientation.w);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "处理失败: %s", ex.what());
        }
    }

    // 新增：centroid_point回调函数 - 解析PointXYZI格式
    void centroid_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // 将ROS PointCloud2转换为PCL PointCloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg, *cloud);
            
            // 存储原始消息和转换后的点云
            latest_centroid_ = *msg;
            latest_centroid_cloud_ = cloud;
            centroid_received_ = true;
            
            RCLCPP_DEBUG(this->get_logger(), "收到centroid_point数据，点数: %zu", cloud->points.size());
            
            // 打印前几个点的信息用于调试
            if (!cloud->points.empty()) {
                for (size_t i = 0; i < std::min(cloud->points.size(), size_t(3)); ++i) {
                    const auto& point = cloud->points[i];
                    RCLCPP_DEBUG(this->get_logger(), "点[%zu]: (%.3f, %.3f, %.3f), 强度: %.3f", 
                                i, point.x, point.y, point.z, point.intensity);
                }
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "转换PointCloud2数据失败: %s", e.what());
        }
    }

    void publish_markers()
    {
        size_t sub_count = marker_pub_->get_subscription_count();
        if(sub_count == 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "没有检测到Marker订阅者，请检查RViz配置");
            return;
        }

        // 1. 用户位置Marker (绿色球体)
        visualization_msgs::msg::Marker user_marker;
        user_marker.header.frame_id = latest_user_pos_.header.frame_id;
        user_marker.header.stamp = this->now();
        user_marker.ns = "user_position";
        user_marker.id = USER_MARKER_ID;
        user_marker.type = visualization_msgs::msg::Marker::SPHERE;
        user_marker.action = visualization_msgs::msg::Marker::ADD;
        user_marker.pose = latest_user_pos_.pose;
        user_marker.scale.x = user_marker.scale.y = user_marker.scale.z = 0.3;
        user_marker.color.r = 0.0;
        user_marker.color.g = 1.0;
        user_marker.color.b = 0.0;
        user_marker.color.a = 0.8; // 确保alpha不为0
        user_marker.lifetime = rclcpp::Duration::from_seconds(1);
        
        marker_pub_->publish(user_marker);
        RCLCPP_DEBUG(this->get_logger(), "发布用户Marker于位置(%.2f, %.2f, %.2f)", 
            user_marker.pose.position.x, 
            user_marker.pose.position.y,
            user_marker.pose.position.z);

        // 2. 小车位置Marker (蓝色球体)
        if(odom_received_) {
            visualization_msgs::msg::Marker robot_marker;
            robot_marker.header.frame_id = latest_odom_.header.frame_id;
            robot_marker.header.stamp = this->now();
            robot_marker.ns = "robot_position";
            robot_marker.id = ROBOT_MARKER_ID;
            robot_marker.type = visualization_msgs::msg::Marker::SPHERE;
            robot_marker.action = visualization_msgs::msg::Marker::ADD;
            robot_marker.pose = latest_odom_.pose.pose;
            robot_marker.scale.x = robot_marker.scale.y = robot_marker.scale.z = 0.4;
            robot_marker.color.r = 0.0;
            robot_marker.color.g = 0.0;
            robot_marker.color.b = 1.0;
            robot_marker.color.a = 0.8; // 确保alpha不为0
            robot_marker.lifetime = rclcpp::Duration::from_seconds(1);
            
            marker_pub_->publish(robot_marker);
            RCLCPP_DEBUG(this->get_logger(), "发布小车Marker于位置(%.2f, %.2f, %.2f)", 
                robot_marker.pose.position.x, 
                robot_marker.pose.position.y,
                robot_marker.pose.position.z);
        }
    }

    // 新增：发布质心点云可视化 - 显示所有PointXYZI点
    void publish_centroid_marker()
    {
        if (!centroid_received_ || !latest_centroid_cloud_) {
            return;
        }

        size_t sub_count = centroid_marker_pub_->get_subscription_count();
        if(sub_count == 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "没有检测到Centroid Marker订阅者");
            return;
        }

        visualization_msgs::msg::Marker centroid_marker;
        centroid_marker.header = latest_centroid_.header;
        centroid_marker.header.stamp = this->now();
        centroid_marker.ns = "centroid_points";
        centroid_marker.id = CENTROID_MARKER_ID;
        centroid_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        centroid_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置点的大小
        centroid_marker.scale.x = 0.15;
        centroid_marker.scale.y = 0.15;
        centroid_marker.scale.z = 0.15;
        
        // 设置颜色 (根据强度值设置颜色，强度越高越红)
        centroid_marker.color.r = 1.0;
        centroid_marker.color.g = 0.2;
        centroid_marker.color.b = 0.0;
        centroid_marker.color.a = 1.0;
        
        centroid_marker.lifetime = rclcpp::Duration::from_seconds(1);
        
        // 添加所有点
        for (const auto& point : latest_centroid_cloud_->points) {
            geometry_msgs::msg::Point ros_point;
            ros_point.x = point.x;
            ros_point.y = point.y;
            ros_point.z = point.z;
            centroid_marker.points.push_back(ros_point);
        }
        
        centroid_marker_pub_->publish(centroid_marker);
        RCLCPP_DEBUG(this->get_logger(), "发布centroid marker，包含 %zu 个点", 
                    latest_centroid_cloud_->points.size());
    }

    void publish_camera_link_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "camera_init";
        transform_stamped.child_frame_id = "camera_link";
        
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void publish_lidar_to_imu_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "body";
        transform_stamped.child_frame_id = "rslidar";
        
        transform_stamped.transform.translation.x = lidar_to_imu_tf_.getOrigin().x();
        transform_stamped.transform.translation.y = lidar_to_imu_tf_.getOrigin().y();
        transform_stamped.transform.translation.z = lidar_to_imu_tf_.getOrigin().z();
        
        transform_stamped.transform.rotation = tf2::toMsg(lidar_to_imu_tf_.getRotation());
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void publish_integrated_data()
    {
        if (odom_received_ && grid_received_ && user_pos_received_ && centroid_received_)
        {
            auto message = integrated_navigation::msg::IntegratedData();
            message.odometry = latest_odom_;
            message.occupancy_grid = latest_grid_;
            message.user_position = latest_user_pos_;
            message.centroid_point = latest_centroid_; // 新增：添加centroid数据
            
            integrated_pub_->publish(message);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "成功处理并发布集成导航数据:\n"
                "- 里程计数据(时间戳: %d.%09d)\n"
                "- 占据栅格(分辨率: %.3f, 尺寸: %dx%d)\n"
                "- 用户位置(x=%.3f, y=%.3f, z=%.3f)\n"
                "- 质心点云(点数: %zu)",
                latest_odom_.header.stamp.sec, latest_odom_.header.stamp.nanosec,
                latest_grid_.info.resolution, 
                latest_grid_.info.width, latest_grid_.info.height,
                latest_user_pos_.pose.position.x,
                latest_user_pos_.pose.position.y,
                latest_user_pos_.pose.position.z,
                latest_centroid_cloud_ ? latest_centroid_cloud_->points.size() : 0);
            
            // 确保可视化更新
            publish_markers();
            publish_centroid_marker();
        }
        else
        {
            if (!odom_received_)
                RCLCPP_WARN(this->get_logger(), "等待输入里程计数据");
            if (!grid_received_)
                RCLCPP_WARN(this->get_logger(), "等待输入占据栅格数据");
            if (!user_pos_received_)
                RCLCPP_WARN(this->get_logger(), "等待输入用户位置数据");
            if (!centroid_received_)
                RCLCPP_WARN(this->get_logger(), "等待输入centroid点云数据");
        }
    }

    // TF2相关组件
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 外参变换
    tf2::Transform lidar_to_camera_tf_;
    tf2::Transform lidar_to_imu_tf_;
    
    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr user_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_sub_; // 新增
    
    // 发布者
    rclcpp::Publisher<integrated_navigation::msg::IntegratedData>::SharedPtr integrated_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroid_marker_pub_; // 新增
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr final_pos_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 最新数据
    nav_msgs::msg::Odometry latest_odom_;
    nav_msgs::msg::OccupancyGrid latest_grid_;
    geometry_msgs::msg::PoseStamped latest_user_pos_;
    sensor_msgs::msg::PointCloud2 latest_centroid_; // 原始PointCloud2消息
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_centroid_cloud_; // 解析后的PCL点云
    
    // 数据接收标志
    bool odom_received_ = false;
    bool grid_received_ = false;
    bool user_pos_received_ = false;
    bool centroid_received_ = false; // 新增
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntegratedNavigationNode>());
    rclcpp::shutdown();
    return 0;
}