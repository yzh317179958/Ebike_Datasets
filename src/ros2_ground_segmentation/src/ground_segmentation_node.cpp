// ground_segmentation_node.cpp
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
#include "geometry_msgs/msg/point.hpp"
using PointInT = pcl::PointXYZI;    
using PointRGBT = pcl::PointXYZRGB; 
#include <queue>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
class ThreadPool {
public:
    ThreadPool(size_t threads) : stop(false) {
        for(size_t i = 0; i < threads; ++i)
            workers.emplace_back([this] {
                for(;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                        if(this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if(stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace([task](){ (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker: workers)
            worker.join();
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks; 
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};
// 定义网格大小（单位：米）
constexpr float GRID_SIZE = 0.5f;  

// 定义哈希用的key
struct GridIndex {
    int x, y;

    bool operator==(const GridIndex &other) const {
        return x == other.x && y == other.y;
    }
};

// 为unordered_map提供hash函数
struct GridIndexHash {
    std::size_t operator()(const GridIndex& k) const {
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
    }
};
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
//==========================
    sub_keypoints_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/keypoint_tracks", 10,
        std::bind(&GroundSegmentationNode::keypoints_callback, this, std::placeholders::_1));
    centroid_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroid_point", 10);
    pub_non_human_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/no_human_points", 10);
    // sub_save_image_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     "/camera/color/image_raw", 10,
    //     std::bind(&GroundSegmentationNode::color_callback, this, std::placeholders::_1));
    T_lidar_cam_.setIdentity();
//==================================
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points_colored", 10);
    pub_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_points_colored", 10);
    

    omp_set_num_threads(4); 
  }

private:
  double odom_z;
  double odom_x;
  double odom_y;
  Eigen::Matrix4f latest_odom_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mutex_; 
  std::chrono::high_resolution_clock::time_point callback_end5;
  float xmin = std::numeric_limits<float>::max();
  float ymin = std::numeric_limits<float>::max();
  float xmax = std::numeric_limits<float>::lowest();
  float ymax = std::numeric_limits<float>::lowest();
  ///=================================
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_keypoints_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_human_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_keypoints_;

  std::shared_ptr<ThreadPool> thread_pool;
  Eigen::Matrix4f T_lidar_cam_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_pub;
    std::mutex filtered_cloud_mutex;
    std::mutex filtered_cloud_non_mutex;
  //=====================
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

//=============================
void keypoints_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    latest_keypoints_ = msg;
    xmin = std::numeric_limits<float>::max();
    ymin = std::numeric_limits<float>::max();
    xmax = std::numeric_limits<float>::lowest();
    ymax = std::numeric_limits<float>::lowest();
    for (const auto &pt : msg->polygon.points) {
        if (pt.x < xmin) xmin = pt.x-5;
        if (pt.y < ymin) ymin = pt.y-5;
        if (pt.x > xmax) xmax = pt.x+5;
        if (pt.y > ymax) ymax = pt.y+5;
    }
}
//==============================
void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        if (!latest_odom_pose_.isZero(1e-6)) {
            thread_pool = std::make_shared<ThreadPool>(4);
            static int frame_counter = 0;
            // if (frame_counter++ % 2 != 0) { // 每2帧处理一次
            //     return;
            // }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
            if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty input cloud, skipping processing");
            return;
            }            
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -5.0)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, -0.4)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, -10.0)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, 10.0)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, -10.0)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, 10.0)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GT, 0.2f)));
            
            pcl::ConditionalRemoval<pcl::PointXYZI> cond_rem;
            cond_rem.setCondition(range_cond);
            cond_rem.setInputCloud(cloud);
            cond_rem.filter(*cloud_filtered);

            auto callback_end1 = std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end1 - callback_start);
            
pcl::PointCloud<PointInT>::Ptr filtered_cloud(new pcl::PointCloud<PointInT>);
pcl::PointCloud<PointInT>::Ptr filtered_cloud_non(new pcl::PointCloud<PointInT>);

// 使用 Eigen::Affine3f 表示变换
Eigen::Affine3f T_lidar_cam_affine;
T_lidar_cam_affine.matrix() << -0.000192, 0.005236, -0.999986, 0.08,
                               0.997441, 0.071499, 0.000183, 0.05,
                               0.071499, -0.997427, -0.005237, -0.03,
                               0, 0, 0, 1;
Eigen::Affine3f T_cam_lidar_affine = T_lidar_cam_affine.inverse();

// 并行处理投影和过滤
std::vector<std::future<void>> projection_futures;
int num_points = cloud_filtered->points.size();
int num_threads = std::thread::hardware_concurrency();
int chunk_size = num_points / num_threads;

for (int i = 0; i < num_threads; ++i) {
    int start_idx = i * chunk_size;
    int end_idx = (i == num_threads - 1) ? num_points : (i + 1) * chunk_size;

    projection_futures.push_back(thread_pool->enqueue([&, start_idx, end_idx]() {
        for (int j = start_idx; j < end_idx; ++j) {
            const auto& pt = cloud_filtered->points[j];

            // 使用 Affine3f 做点变换
            Eigen::Vector3f p_lidar(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_cam = T_cam_lidar_affine * p_lidar;

            float fx = 367.2095642089844f;
            float fy = 367.2040100097656f;
            float cx = 316.4378967285156f;
            float cy = 244.602783203125f;

            float u = (p_cam(0) / p_cam(2)) * fx + cx;
            float v = (p_cam(1) / p_cam(2)) * fy + cy;

            bool in_human = false;

            if (u >= xmin && u <= xmax && v >= ymin && v <= ymax && pt.z > -0.7) {
                in_human = true;
                std::lock_guard<std::mutex> lock(this->filtered_cloud_mutex);
                filtered_cloud->points.push_back(pt);
            }

            if (!in_human) {
                std::lock_guard<std::mutex> lock(this->filtered_cloud_non_mutex);
                filtered_cloud_non->points.push_back(pt);
            }
        }
    }));
}

// 等待所有投影任务完成
for (auto& future : projection_futures) {
    future.wait();
}

// 更新点云属性
filtered_cloud->width = filtered_cloud->points.size();
filtered_cloud->height = 1;
filtered_cloud->is_dense = true;

filtered_cloud_non->width = filtered_cloud_non->points.size();
filtered_cloud_non->height = 1;
filtered_cloud_non->is_dense = true;

// 并行处理网格索引构建
std::unordered_map<GridIndex, std::vector<int>, GridIndexHash> grid_points;
std::mutex grid_points_mutex;
std::vector<std::future<void>> grid_futures;
num_points = filtered_cloud->points.size();
chunk_size = num_points / num_threads;

for (int i = 0; i < num_threads; ++i) {
    int start_idx = i * chunk_size;
    int end_idx = (i == num_threads - 1) ? num_points : (i + 1) * chunk_size;

    grid_futures.push_back(thread_pool->enqueue([&, start_idx, end_idx]() {
        for (int j = start_idx; j < end_idx; ++j) {
            const auto& pt = filtered_cloud->points[j];
            int gx = static_cast<int>(std::floor(pt.x / GRID_SIZE));
            int gy = static_cast<int>(std::floor(pt.y / GRID_SIZE));
            GridIndex idx{gx, gy};

            std::lock_guard<std::mutex> lock(grid_points_mutex);
            grid_points[idx].push_back(j);
        }
    }));
}

// 等待所有网格任务完成
for (auto& future : grid_futures) {
    future.wait();
}

// 查找最大计数的网格
size_t max_count = 0;
GridIndex max_idx{0, 0};
for (const auto& kv : grid_points) {
    if (kv.second.size() > max_count) {
        max_count = kv.second.size();
        max_idx = kv.first;
    }
}

// 提取最大网格点云
pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
for (int idx : grid_points[max_idx]) {
    new_cloud->push_back(filtered_cloud->points[idx]);
}

filtered_cloud->clear();
*filtered_cloud = *new_cloud;

// 变换到odom坐标系并计算质心
if (!filtered_cloud->empty()) {
Eigen::Affine3f latest_odom_affine;
latest_odom_affine.matrix() = latest_odom_pose_.cast<float>();

    Eigen::Vector4f centroid2;
    pcl::transformPointCloud(*filtered_cloud, *new_cloud2, latest_odom_affine.matrix());
    pcl::compute3DCentroid(*new_cloud2, centroid2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI pt2;
    pt2.x = centroid2[0];
    pt2.y = centroid2[1];
    pt2.z = centroid2[2];
    pt2.intensity = 1.0f;
    centroid_cloud->push_back(pt2);

    sensor_msgs::msg::PointCloud2 out_msg2;
    pcl::toROSMsg(*centroid_cloud, out_msg2);
    out_msg2.header.frame_id = "camera_init";
    centroid_pub->publish(out_msg2);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*new_cloud2, out_msg);
    out_msg.header = cloud_msg->header;
    pub_non_human_->publish(out_msg);
}

auto callback_end2 = std::chrono::high_resolution_clock::now();
auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end2 - callback_end1);
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                     "touying took %ld ms", duration2.count());

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                pcl::transformPointCloud(*filtered_cloud_non, *transformed_cloud, latest_odom_pose_.cast<float>());
            }
            
            if (transformed_cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Transformed cloud is empty, skipping processing");
                return;
            }
            
            // 地面分割和后续处理也可以并行化，但需要更复杂的同步
            // 这里只展示了部分并行化，完整实现需要根据具体逻辑进行
            
            // 创建地面分割任务
            auto ground_seg_task = thread_pool->enqueue([&]() {
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
                
                return std::make_pair(ground_inliers, coefficients);
            });
            
            // 等待地面分割结果
            auto [ground_inliers, coefficients] = ground_seg_task.get();
            
            auto callback_end3 = std::chrono::high_resolution_clock::now();
            auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end3 - callback_end2);
            
            if (ground_inliers->indices.empty()) {
                RCLCPP_WARN(this->get_logger(), "No ground plane found by LMedS.");
                auto cloud_non_ground_rgb = convertToColoredCloud(cloud, 255, 0, 0);
                publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
                return;
            }
            
            // 提取地面和非地面点云
            pcl::PointCloud<PointInT>::Ptr cloud_ground(new pcl::PointCloud<PointInT>);
            pcl::PointCloud<PointInT>::Ptr cloud_non_ground(new pcl::PointCloud<PointInT>);
            pcl::ExtractIndices<PointInT> extract;
            extract.setInputCloud(transformed_cloud);
            extract.setIndices(ground_inliers);
            extract.setNegative(false);
            extract.filter(*cloud_ground);
            extract.setNegative(true);
            extract.filter(*cloud_non_ground);
            
            auto callback_end4 = std::chrono::high_resolution_clock::now();
            auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end4 - callback_end3);
            
            if (cloud_ground->size() < 100) {
                RCLCPP_WARN(this->get_logger(), "Too few ground points, skipping refinement.");
                auto cloud_ground_rgb = convertToColoredCloud(cloud_ground, 0, 255, 0);
                auto cloud_non_ground_rgb = convertToColoredCloud(cloud_non_ground, 255, 0, 0);
                publishCloud(cloud_ground_rgb, cloud_msg->header, pub_ground_);
                publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
                return;
            }
            
            // 并行计算地面点云的中心和协方差
            auto centroid_cov_task = thread_pool->enqueue([&]() {
                pcl::RandomSample<pcl::PointXYZI> random_sample;
                random_sample.setInputCloud(cloud_ground);
                random_sample.setSample(std::min(500, static_cast<int>(cloud_ground->size()))); 
                pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_ground(new pcl::PointCloud<pcl::PointXYZI>);
                random_sample.filter(*sampled_ground);
                
                Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
                for (const auto& pt : sampled_ground->points) {
                    centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
                }
                centroid /= sampled_ground->points.size();
                
                Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
                for (const auto& pt : sampled_ground->points) {
                    Eigen::Vector3f centered = Eigen::Vector3f(pt.x, pt.y, pt.z) - centroid;
                    covariance += centered * centered.transpose();
                }
                covariance /= sampled_ground->points.size();
                
                return std::make_pair(centroid, covariance);
            });
            
            auto [centroid, covariance] = centroid_cov_task.get();
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);
            float d = -normal.dot(centroid);
            float normal_norm = std::sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());
            float normal_norm_inv = 1.0f / normal_norm;
            
            float dot = normal.z(); 
            float cos_theta = std::abs(dot) * normal_norm_inv;
            cos_theta = std::max(0.0f, std::min(1.0f, cos_theta));
            float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;
            
            // 并行细化地面点云
            pcl::PointCloud<PointInT>::Ptr cloud_refined_ground(new pcl::PointCloud<PointInT>);
            pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground(new pcl::PointCloud<PointInT>);
            cloud_refined_ground->reserve(transformed_cloud->points.size() / 2);
            cloud_refined_non_ground->reserve(transformed_cloud->points.size() / 2);
            
            std::vector<bool> is_ground(transformed_cloud->points.size(), false);
            std::mutex is_ground_mutex;
            
            std::vector<std::future<void>> refinement_futures;
            num_points = transformed_cloud->points.size();
            chunk_size = num_points / num_threads;
            
            for (int i = 0; i < num_threads; ++i) {
                int start_idx = i * chunk_size;
                int end_idx = (i == num_threads - 1) ? num_points : (i + 1) * chunk_size;
                
                refinement_futures.push_back(thread_pool->enqueue([&, start_idx, end_idx]() {
                    for (size_t j = start_idx; j < end_idx; ++j) {
                        const auto& pt = transformed_cloud->points[j];
                        float dist = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
                        
                        if ((dist < 0.04f && angle_deg < 45)) {
                            std::lock_guard<std::mutex> lock(is_ground_mutex);
                            is_ground[j] = true;
                        }
                    }
                }));
            }
            
            // 等待所有细化任务完成
            for (auto& future : refinement_futures) {
                future.wait();
            }
            
            // 收集结果
            for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
                if (is_ground[i]) {
                    cloud_refined_ground->points.push_back(transformed_cloud->points[i]);
                } else {
                    cloud_refined_non_ground->points.push_back(transformed_cloud->points[i]);
                }
            }
            
            auto callback_end5 = std::chrono::high_resolution_clock::now();
            auto duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end5 - callback_end4);
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    //                     "qufen took %ld ms", duration5.count());
        pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new(new pcl::PointCloud<PointInT>);
        pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new2(new pcl::PointCloud<PointInT>);
        cloud_refined_non_ground2_new->reserve(cloud_refined_non_ground->points.size());
        cloud_refined_non_ground2_new2->reserve(cloud_refined_non_ground->points.size());

        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(cloud_refined_non_ground);

        const float height_threshold = 0.2f;
        const float z_diff_threshold = 0.05f;
        const float search_radius = 0.1f;
        const int min_neighbors = 3;
        const float odom_radius_sq = 1.44f;

        std::vector<float> ground_dists(cloud_refined_non_ground->points.size());

        // 先计算点到地面的距离
        for (size_t i = 0; i < cloud_refined_non_ground->points.size(); ++i) {
            const auto& pt = cloud_refined_non_ground->points[i];
            ground_dists[i] = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
        }

        // 定义互斥量保护共享点云
        std::mutex mtx_ground, mtx_non_ground;

        std::vector<std::future<void>> futures;

        for (size_t i = 0; i < cloud_refined_non_ground->points.size(); ++i) {
            futures.emplace_back(thread_pool->enqueue([&, i]() {
                const auto& pt = cloud_refined_non_ground->points[i];

                if (ground_dists[i] > height_threshold) {
                    std::lock_guard<std::mutex> lock(mtx_non_ground);
                    cloud_refined_non_ground2_new->points.push_back(pt);
                    return;
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
                        std::lock_guard<std::mutex> lock(mtx_ground);
                        cloud_refined_ground->points.push_back(pt);
                    } else {
                        std::lock_guard<std::mutex> lock(mtx_non_ground);
                        cloud_refined_non_ground2_new->points.push_back(pt);
                    }
                } else {
                    if (ground_dists[i] < height_threshold) {
                        std::lock_guard<std::mutex> lock(mtx_ground);
                        cloud_refined_ground->points.push_back(pt);
                    } else {
                        std::lock_guard<std::mutex> lock(mtx_non_ground);
                        cloud_refined_non_ground2_new->points.push_back(pt);
                    }
                }
            }));
        }

        // 等待所有线程完成
        for (auto& f : futures) {
            f.get();
        }

        // 根据里程计位置进一步分类
        for(const auto& pt : cloud_refined_non_ground2_new->points){
            float dx = pt.x - odom_x;
            float dy = pt.y - odom_y;
            if (dx*dx + dy*dy < odom_radius_sq) {
                cloud_refined_ground->points.push_back(pt);
            } else {
                cloud_refined_non_ground2_new2->points.push_back(pt);
            }
        }

        auto cloud_ground_rgb = convertToColoredCloud(cloud_refined_ground, 0, 255, 0);
        auto cloud_non_ground_rgb = convertToColoredCloud(cloud_refined_non_ground2_new2, 255, 0, 0);

        if (!cloud_ground_rgb->empty()) {
            publishCloud(cloud_ground_rgb, cloud_msg->header, pub_ground_);
        }

        if (!cloud_non_ground_rgb->empty()) {
            publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
        }
    }
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    //                     "zuihou took %ld ms", duration.count());
}

  pcl::PointCloud<PointRGBT>::Ptr convertToColoredCloud(
    const pcl::PointCloud<PointInT>::Ptr & input_cloud,
    uint8_t r, uint8_t g, uint8_t b)
  {
    pcl::PointCloud<PointRGBT>::Ptr colored_cloud(new pcl::PointCloud<PointRGBT>);
    colored_cloud->points.reserve(input_cloud->points.size());

    for (const auto &pt : input_cloud->points) {
      PointRGBT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.r = r;
      p.g = g;
      p.b = b;
      colored_cloud->points.push_back(p);
    }

    colored_cloud->width = static_cast<uint32_t>(colored_cloud->points.size());
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_cloud->is_dense;

    return colored_cloud;
  }

  void publishCloud(
    pcl::PointCloud<PointRGBT>::Ptr cloud,
    const std_msgs::msg::Header & header,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    output.header.frame_id = "camera_init";
    pub->publish(output); 
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;
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