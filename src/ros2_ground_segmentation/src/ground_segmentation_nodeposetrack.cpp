#include <memory>
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#include <deque>
#include <pcl/common/io.h> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h> 
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>       
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <mutex>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigenvalues>
#include <omp.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

using PointInT = pcl::PointXYZI;    
using PointRGBT = pcl::PointXYZRGB; 

class GroundSegmentationNode : public rclcpp::Node
{
public:
  GroundSegmentationNode() : Node("ground_segmentation_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10,
      std::bind(&GroundSegmentationNode::pointcloud_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&GroundSegmentationNode::odom_callback, this, std::placeholders::_1));

    sub_keypoints_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/keypoint_tracks", 10,
        std::bind(&GroundSegmentationNode::keypoints_callback, this, std::placeholders::_1));
        
    pub_non_human_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/no_human_points", 10);
        
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points_colored", 10);
    pub_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_points_colored", 10);

    // 预计算相机参数
    fx_ = 367.2095642089844f;
    fy_ = 367.2040100097656f;
    cx_ = 316.4378967285156f;
    cy_ = 244.602783203125f;

    // 预计算变换矩阵
    T_lidar_cam_.setIdentity();
    T_lidar_cam_ << -0.000192, 0.005236, -0.999986, 0.08,
                    0.997441, 0.071499, 0.000183, 0.05,
                    0.071499, -0.997427, -0.005237, -0.03,
                    0, 0, 0, 1;
    T_cam_lidar_ = T_lidar_cam_.inverse();

    omp_set_num_threads(4); 
  }

private:
  double odom_z = 0.0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  Eigen::Matrix4f latest_odom_pose_ = Eigen::Matrix4f::Identity();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mutex_; 
  
  float xmin = std::numeric_limits<float>::max();
  float ymin = std::numeric_limits<float>::max();
  float xmax = std::numeric_limits<float>::lowest();
  float ymax = std::numeric_limits<float>::lowest();
  
  // 预计算的相机参数
  float fx_, fy_, cx_, cy_;
  Eigen::Matrix4f T_lidar_cam_;
  Eigen::Matrix4f T_cam_lidar_;

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_keypoints_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_human_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_keypoints_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_z = msg->pose.pose.position.z;
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    Eigen::Matrix4f odom_T_lidar = Eigen::Matrix4f::Identity();
    tf2::Matrix3x3 R(q);
    
    Eigen::Matrix3f R_eigen;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R_eigen(i, j) = R[i][j];
      }
    }

    odom_T_lidar.block<3,3>(0,0) = R_eigen;
    odom_T_lidar(0,3) = msg->pose.pose.position.x;
    odom_T_lidar(1,3) = msg->pose.pose.position.y;
    odom_T_lidar(2,3) = msg->pose.pose.position.z;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_pose_ = odom_T_lidar;
  }

  void keypoints_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    latest_keypoints_ = msg;
    xmin = std::numeric_limits<float>::max();
    ymin = std::numeric_limits<float>::max();
    xmax = std::numeric_limits<float>::lowest();
    ymax = std::numeric_limits<float>::lowest();
    
    for (const auto &pt : msg->polygon.points) {
        if (pt.x < xmin) xmin = pt.x - 5.0f;
        if (pt.y < ymin) ymin = pt.y - 5.0f;
        if (pt.x > xmax) xmax = pt.x + 5.0f;
        if (pt.y > ymax) ymax = pt.y + 5.0f;
    }
  }

  // 内联投影函数，减少重复计算
  inline void projectPoint(const Eigen::Vector4f& p_cam, float& u, float& v) const {
    u = (p_cam(0) / p_cam(2)) * fx_ + cx_;
    v = (p_cam(1) / p_cam(2)) * fy_ + cy_;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    auto callback_start = std::chrono::high_resolution_clock::now();
    
    if (latest_odom_pose_.isZero(1e-6)) {
      RCLCPP_WARN(this->get_logger(), "Odom pose not available, skipping processing");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty input cloud, skipping processing");
      return;
    }

    // 使用直接循环代替ConditionalRemoval，减少开销
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filtered->reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
      if (pt.z > -5.0f && pt.z < -0.4f &&
          pt.x > -10.0f && pt.x < 10.0f &&
          pt.y > -10.0f && pt.y < 10.0f &&
          pt.intensity > 0.2f) {
        cloud_filtered->push_back(pt);
      }
    }

    // 人体区域过滤 - 优化投影计算
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_cloud->reserve(cloud_filtered->size());
    
    for (const auto& pt : cloud_filtered->points) {
      Eigen::Vector4f p_lidar(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f p_cam = T_cam_lidar_ * p_lidar;
      
      // 避免除以零和无效投影
      if (p_cam(2) > 0.1f) {
        float u, v;
        projectPoint(p_cam, u, v);
        
        if (u >= xmin && u <= xmax && v >= ymin && v <= ymax && pt.z > -0.7f) {
          filtered_cloud->push_back(pt);
        }
      }
    }

    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    // 只在有订阅者时发布
    if (pub_non_human_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*filtered_cloud, out_msg);
      out_msg.header = cloud_msg->header;
      pub_non_human_->publish(out_msg);
    }

    // 坐标变换 - 使用并行优化
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    transformed_cloud->resize(cloud_filtered->size());
    
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      #pragma omp parallel for
      for (size_t i = 0; i < cloud_filtered->size(); ++i) {
        const auto& pt = cloud_filtered->points[i];
        Eigen::Vector4f pt_vec(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f transformed_vec = latest_odom_pose_ * pt_vec;
        transformed_cloud->points[i].x = transformed_vec(0);
        transformed_cloud->points[i].y = transformed_vec(1);
        transformed_cloud->points[i].z = transformed_vec(2);
        transformed_cloud->points[i].intensity = pt.intensity;
      }
    }
    
    if (transformed_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Transformed cloud is empty, skipping processing");
      return;
    }
    
    // 地面分割
    pcl::SACSegmentation<PointInT> seg;
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_LMEDS); 
    seg.setDistanceThreshold(0.04);
    seg.setMaxIterations(20); 
    seg.setInputCloud(transformed_cloud);
    seg.segment(*ground_inliers, *coefficients);
    
    if (ground_inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No ground plane found by LMedS.");
      auto cloud_non_ground_rgb = convertToColoredCloud(transformed_cloud, 255, 0, 0);
      publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
      return;
    }
    
    // 提取地面和非地面点 - 预分配内存
    pcl::PointCloud<PointInT>::Ptr cloud_ground(new pcl::PointCloud<PointInT>);
    pcl::PointCloud<PointInT>::Ptr cloud_non_ground(new pcl::PointCloud<PointInT>);
    
    cloud_ground->reserve(ground_inliers->indices.size());
    cloud_non_ground->reserve(transformed_cloud->size() - ground_inliers->indices.size());
    
    // 使用标志位数组提高效率
    std::vector<bool> is_ground_flag(transformed_cloud->size(), false);
    for (const auto& idx : ground_inliers->indices) {
      is_ground_flag[idx] = true;
      cloud_ground->push_back(transformed_cloud->points[idx]);
    }
    
    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      if (!is_ground_flag[i]) {
        cloud_non_ground->push_back(transformed_cloud->points[i]);
      }
    }
    
    if (cloud_ground->size() < 100) {
      RCLCPP_WARN(this->get_logger(), "Too few ground points, skipping refinement.");
      auto cloud_ground_rgb = convertToColoredCloud(cloud_ground, 0, 255, 0);
      auto cloud_non_ground_rgb = convertToColoredCloud(cloud_non_ground, 255, 0, 0);
      publishCloud(cloud_ground_rgb, cloud_msg->header, pub_ground_);
      publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
      return;
    }
    
    // 地面采样 - 优化采样过程
    pcl::RandomSample<pcl::PointXYZI> random_sample;
    random_sample.setInputCloud(cloud_ground);
    random_sample.setSample(std::min(500, static_cast<int>(cloud_ground->size()))); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_ground(new pcl::PointCloud<pcl::PointXYZI>);
    random_sample.filter(*sampled_ground);
    
    // 计算法向量和平面参数 - 使用Eigen的block操作优化
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& pt : sampled_ground->points) {
      centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
    centroid /= sampled_ground->size();
    
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& pt : sampled_ground->points) {
      Eigen::Vector3f centered = Eigen::Vector3f(pt.x, pt.y, pt.z) - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= sampled_ground->size();
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f normal = solver.eigenvectors().col(0);
    float d = -normal.dot(centroid);
    
    float normal_norm = normal.norm();
    float normal_norm_inv = 1.0f / normal_norm;
    
    float dot = normal.z(); 
    float cos_theta = std::abs(dot) * normal_norm_inv;
    cos_theta = std::max(0.0f, std::min(1.0f, cos_theta));
    float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;
    
    // 地面精化 - 预分配内存和并行处理
    pcl::PointCloud<PointInT>::Ptr cloud_refined_ground(new pcl::PointCloud<PointInT>);
    pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground(new pcl::PointCloud<PointInT>);
    
    cloud_refined_ground->reserve(transformed_cloud->size() / 2);
    cloud_refined_non_ground->reserve(transformed_cloud->size() / 2);
    
    std::vector<bool> is_ground(transformed_cloud->size(), false);
    
    // 预计算平面距离
    std::vector<float> distances(transformed_cloud->size());
    #pragma omp parallel for
    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      const auto& pt = transformed_cloud->points[i];
      distances[i] = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
    }
    
    #pragma omp parallel for
    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      if (distances[i] < 0.04f && angle_deg < 45) {
        is_ground[i] = true;
      }
    }
    
    for (size_t i = 0; i < transformed_cloud->size(); ++i) {
      if (is_ground[i]) {
        cloud_refined_ground->push_back(transformed_cloud->points[i]);
      } else {
        cloud_refined_non_ground->push_back(transformed_cloud->points[i]);
      }
    }

    // 坑洞检测 - 优化搜索逻辑
    if (!cloud_refined_ground->empty()) {
      float r_search = 0.3f;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pothole(new pcl::PointCloud<pcl::PointXYZI>);
      cloud_pothole->reserve(cloud_refined_non_ground->size());
      
      // 构建地面点空间索引（简化版，实际可以使用KDTree）
      std::vector<Eigen::Vector3f> ground_points;
      ground_points.reserve(cloud_refined_ground->size());
      for (const auto& gpt : cloud_refined_ground->points) {
        ground_points.emplace_back(gpt.x, gpt.y, gpt.z);
      }
      
      for (const auto& pt : cloud_refined_non_ground->points) {
        bool has_ground_above = false;
        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
        
        for (const auto& gpt_vec : ground_points) {
          float dx = gpt_vec.x() - pt_vec.x();
          float dy = gpt_vec.y() - pt_vec.y();
          float dist_xy_sq = dx * dx + dy * dy;
          
          if (dist_xy_sq <= r_search * r_search) {
            if (gpt_vec.z() > pt_vec.z() && std::abs(gpt_vec.z() - pt_vec.z()) < 0.5f) {
              has_ground_above = true;
              break;
            }
          }
        }
        
        if (has_ground_above) {
          cloud_refined_ground->push_back(pt);
        } else {
          cloud_pothole->push_back(pt);
        }
      }
      
      *cloud_refined_non_ground = *cloud_pothole;
    }
    
    // 非地面点进一步处理
    pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new(new pcl::PointCloud<PointInT>);
    pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new2(new pcl::PointCloud<PointInT>);
    
    cloud_refined_non_ground2_new->reserve(cloud_refined_non_ground->size());
    cloud_refined_non_ground2_new2->reserve(cloud_refined_non_ground->size());
    
    // 构建KDTree - 优化搜索
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud(cloud_refined_non_ground);
    
    const float height_threshold = 0.2f;
    const float z_diff_threshold = 0.05f;
    const float search_radius = 0.1f;
    const int min_neighbors = 3;
    const float odom_radius_sq = 1.44f;
    
    // 预计算所有点的地面距离
    std::vector<float> ground_dists(cloud_refined_non_ground->size());
    #pragma omp parallel for
    for (size_t i = 0; i < cloud_refined_non_ground->size(); ++i) {
      const auto& pt = cloud_refined_non_ground->points[i];
      ground_dists[i] = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
    }
    
    // 处理非地面点
    #pragma omp parallel for
    for (size_t i = 0; i < cloud_refined_non_ground->size(); ++i) {
      const auto& pt = cloud_refined_non_ground->points[i];
      
      if (ground_dists[i] > height_threshold) {
        #pragma omp critical
        cloud_refined_non_ground2_new->push_back(pt);
        continue;
      }
      
      std::vector<int> indices;
      std::vector<float> sqr_distances;
      int num_neighbors = kdtree->radiusSearch(pt, search_radius, indices, sqr_distances);
      
      if (num_neighbors >= min_neighbors) {
        float z_min = pt.z, z_max = pt.z;
        for (int idx : indices) {
          float z = cloud_refined_non_ground->points[idx].z;
          z_min = std::min(z_min, z);
          z_max = std::max(z_max, z);
        }
        
        if ((z_max - z_min) < z_diff_threshold) {
          #pragma omp critical
          cloud_refined_ground->push_back(pt);
        } else {
          #pragma omp critical
          cloud_refined_non_ground2_new->push_back(pt);
        }
      } else {
        if (ground_dists[i] < height_threshold) {
          #pragma omp critical
          cloud_refined_ground->push_back(pt);
        } else {
          #pragma omp critical
          cloud_refined_non_ground2_new->push_back(pt);
        }
      }
    }
    
    // 处理靠近odom的点
    for (const auto& pt : cloud_refined_non_ground2_new->points) {
      float dx = pt.x - odom_x;
      float dy = pt.y - odom_y;
      if (dx * dx + dy * dy < odom_radius_sq) {
        cloud_refined_ground->push_back(pt);
      } else {
        cloud_refined_non_ground2_new2->push_back(pt);
      }
    }
    
    // 发布结果
    auto cloud_ground_rgb = convertToColoredCloud(cloud_refined_ground, 0, 255, 0);
    auto cloud_non_ground_rgb = convertToColoredCloud(cloud_refined_non_ground2_new2, 255, 0, 0);
    
    if (!cloud_ground_rgb->empty() && pub_ground_->get_subscription_count() > 0) {
      publishCloud(cloud_ground_rgb, cloud_msg->header, pub_ground_);
    }
    
    if (!cloud_non_ground_rgb->empty() && pub_non_ground_->get_subscription_count() > 0) {
      publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
    }
    
    auto callback_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end - callback_start);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "Ground segmentation took %ld ms", duration.count());
  }

  pcl::PointCloud<PointRGBT>::Ptr convertToColoredCloud(
    const pcl::PointCloud<PointInT>::Ptr & input_cloud,
    uint8_t r, uint8_t g, uint8_t b) const
  {
    pcl::PointCloud<PointRGBT>::Ptr colored_cloud(new pcl::PointCloud<PointRGBT>);
    colored_cloud->reserve(input_cloud->size());

    for (const auto &pt : input_cloud->points) {
      PointRGBT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.r = r;
      p.g = g;
      p.b = b;
      colored_cloud->push_back(p);
    }

    colored_cloud->width = colored_cloud->size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_cloud->is_dense;

    return colored_cloud;
  }

  void publishCloud(
    const pcl::PointCloud<PointRGBT>::Ptr& cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) const
  {
    if (cloud->empty() || pub->get_subscription_count() == 0) {
      return;
    }
    
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    output.header.frame_id = "camera_init";
    pub->publish(output); 
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundSegmentationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 
      std::thread::hardware_concurrency()
  );

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}