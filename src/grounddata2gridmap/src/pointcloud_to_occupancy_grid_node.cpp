#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <queue>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <utility>
#include <pcl/surface/mls.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
using namespace cv;
using GridIndex = std::pair<int, int>;

struct GridIndexHash {
  size_t operator()(const GridIndex& idx) const {
    return std::hash<int>()(idx.first) ^ (std::hash<int>()(idx.second) << 1); 
  }
};
struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};
std::unordered_map<GridIndex, int8_t, GridIndexHash> global_occupancy_map_;
std::mutex map_mutex_; 
std::unordered_map<GridIndex, int8_t, GridIndexHash> obstacle_map_;
std::unordered_map<GridIndex, int8_t, GridIndexHash> ground_map_;

using PointT = pcl::PointXYZ;

class PointCloudToOccupancyGridNode : public rclcpp::Node {
public:
  PointCloudToOccupancyGridNode() : Node("pointcloud_to_occupancy_grid_node") {
    int width2 = 200;
    int height2 = 200;
    int value = 100;
    std::vector<int8_t> local_grid(width2 * height2, 0);
    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&PointCloudToOccupancyGridNode::odom_callback, this, std::placeholders::_1));

    sub_obstacle_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/non_ground_points_colored", 10,
      std::bind(&PointCloudToOccupancyGridNode::obstacle_callback, this, std::placeholders::_1));

    sub_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ground_points_colored", 10,
      std::bind(&PointCloudToOccupancyGridNode::ground_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/pointcloud_occupancy_grid", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      std::bind(&PointCloudToOccupancyGridNode::timer_callback, this)
    );

    declare_parameter("grid_resolution", 0.1);
    declare_parameter("grid_width", 200);
    declare_parameter("grid_height", 200);
    declare_parameter("min_z", -2.0);
    declare_parameter("max_z", 1.0);
  }

private:
  double odom_z_ = 0.0;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  std::vector<int8_t> local_grid;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Quaternion current_orientation_;
void update_global_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int8_t occupancy_value) {
    if (!msg) return;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *cloud);

    double resolution = get_parameter("grid_resolution").as_double();
    float min_z = odom_z_ - 1.5;
    float max_z = odom_z_ + 0.2;

    std::unordered_map<GridIndex, int8_t, GridIndexHash> local_map;

    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        if (pt.z < min_z || pt.z > max_z) continue;

        int gx = static_cast<int>(std::floor(pt.x / resolution));
        int gy = static_cast<int>(std::floor(pt.y / resolution));
        local_map[{gx, gy}] = occupancy_value;
    }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (occupancy_value == 100) {
        obstacle_map_.swap(local_map); 
    } else if (occupancy_value == 0) {
        ground_map_.swap(local_map);  
    }

    const int radius_cells = 200;
    auto clean_map = [&](std::unordered_map<GridIndex, int8_t, GridIndexHash>& map) {
        for (auto it = map.begin(); it != map.end(); ) {
            int dx = it->first.first - static_cast<int>(std::floor(odom_x_ / resolution));
            int dy = it->first.second - static_cast<int>(std::floor(odom_y_ / resolution));
            if (std::abs(dx) > radius_cells || std::abs(dy) > radius_cells)
                it = map.erase(it);
            else
                ++it;
        }
    };
    clean_map(obstacle_map_);                                                                         
    clean_map(ground_map_);
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

void timer_callback() {
    double resolution = get_parameter("grid_resolution").as_double();
    int width = get_parameter("grid_width").as_int();
    int height = get_parameter("grid_height").as_int();

    int center_x_idx = static_cast<int>(std::floor(current_position_.x / resolution));
    int center_y_idx = static_cast<int>(std::floor(current_position_.y / resolution));

    std::vector<int8_t> local_grid(width * height, 0);

    std::lock_guard<std::mutex> lock(map_mutex_);
    for (int i = 0; i < height; ++i) {
        int gy = center_y_idx + i - height / 2;
        for (int j = 0; j < width; ++j) {
            int gx = center_x_idx + j - width / 2;
            GridIndex key{gx, gy};
            int idx = i * width + j;
            if (obstacle_map_.count(key))
                local_grid[idx] = 100;
            else if (ground_map_.count(key))
                local_grid[idx] = 0;
            else
                local_grid[idx] = 0;
        }
    }
for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            int8_t cell = local_grid[idx];  // 当前格子的值
            if(cell==100){
                int not_free_count = 0;
                // int not_free_count1 = 0;
                // int not_free_count2 = 0;
                for(int i = 1;i<=1;i++){
                    if (local_grid[(y - i) * width + x] != 100) ++not_free_count;
                    // 下
                    if ( local_grid[(y + i) * width + x] != 100) ++not_free_count;
                    // 左
                    if ( local_grid[y * width + (x - i)] != 100) ++not_free_count;
                    // 右
                    if ( local_grid[y * width + (x + i)] != 100) ++not_free_count;
                    // 左上
                    if ( local_grid[(y - i) * width + (x - i)] != 100) ++not_free_count;
                    // 右上
                    if ( local_grid[(y - i) * width + (x + i)] != 100) ++not_free_count;
                    // 左下
                    if (local_grid[(y + i) * width + (x - i)] != 100) ++not_free_count;
                    // 右下
                    if ( local_grid[(y + i) * width + (x + i)] != 100) ++not_free_count;
                }
                  if (not_free_count>= 5) {  
                      local_grid[idx] = 0;  // 转为空闲
                  }
            }

        }
    }

    // 发布 local_grid
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->get_clock()->now();
    grid_msg.header.frame_id = "camera_init";
    grid_msg.info.resolution = resolution;
    grid_msg.info.width = width;
    grid_msg.info.height = height;
    grid_msg.info.origin.position.x = (center_x_idx - width / 2) * resolution;
    grid_msg.info.origin.position.y = (center_y_idx - height / 2) * resolution;
    grid_msg.info.origin.position.z = odom_z_ - 1.2;
    // grid_msg.info.origin.orientation = current_orientation_;
    grid_msg.info.origin.orientation.w = 1.0;
    grid_msg.data = local_grid;

    pub_->publish(grid_msg);
}


  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    current_orientation_ = msg->pose.pose.orientation;
    odom_z_ = current_position_.z;
    odom_x_ = current_position_.x;
    odom_y_ = current_position_.y;
    rclcpp::Time stamp_time = msg->header.stamp;
    // RCLCPP_INFO(this->get_logger(), "Odom time: %.9f", stamp_time.seconds());
  }

  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto start1 = std::chrono::high_resolution_clock::now(); 

    update_global_map(msg, 100);
    auto end1 = std::chrono::high_resolution_clock::now();    

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    // std::cout << "100 time: " << duration.count() << " ms" << std::endl;
  }

  void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto start2 = std::chrono::high_resolution_clock::now(); 
    update_global_map(msg, 0);
    auto end2 = std::chrono::high_resolution_clock::now();    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    // std::cout << "0 time: " << duration.count() << " ms" << std::endl;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToOccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}
