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
constexpr float GRID_SIZE = 0.5f;  

struct GridIndex {
    int x, y;

    bool operator==(const GridIndex &other) const {
        return x == other.x && y == other.y;
    }
};

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
    

  }

private:
  double odom_z;
  double odom_x;
  double odom_y;
  Eigen::Matrix4f latest_odom_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mutex_; 
  std::chrono::high_resolution_clock::time_point callback_end5;
  std::chrono::high_resolution_clock::time_point callback_end7;
  float xmin = std::numeric_limits<float>::max();
  float ymin = std::numeric_limits<float>::max();
  float xmax = std::numeric_limits<float>::lowest();
  float ymax = std::numeric_limits<float>::lowest();
  ///=================================
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_keypoints_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_human_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_keypoints_;


  Eigen::Matrix4f T_lidar_cam_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_pub;
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
std::shared_ptr<geometry_msgs::msg::Point32> last_top_left_;
std::shared_ptr<geometry_msgs::msg::Point32> last_bottom_right_;

    void keypoints_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        const auto &cur_top_left     = msg->polygon.points[1];
        const auto &cur_bottom_right = msg->polygon.points[2];
        xmin = cur_top_left.x;   
        ymin = cur_top_left.y;   
        xmax = cur_bottom_right.x; 
        ymax = cur_bottom_right.y; 
        if(xmin != 0.0 && ymin != 0.0 && ymax != 0.0 && xmax != 0.0){
            xmin -= 5.0f;
            ymin -= 5.0f;
            xmax += 5.0f;
            ymax += 5.0f;
        }


        // RCLCPP_INFO(this->get_logger(),
        //     "Current frame: top-left(%.3f, %.3f), bottom-right(%.3f, %.3f)",
        //     cur_top_left.x, cur_top_left.y,
        //     cur_bottom_right.x, cur_bottom_right.y
        // );

        // if (last_top_left_ && last_bottom_right_) {
        //     RCLCPP_INFO(this->get_logger(),
        //         "Previous frame: top-left(%.3f, %.3f), bottom-right(%.3f, %.3f)",
        //         last_top_left_->x, last_top_left_->y,
        //         last_bottom_right_->x, last_bottom_right_->y
        //     );
        // }

        last_top_left_ = std::make_shared<geometry_msgs::msg::Point32>(cur_top_left);
        last_bottom_right_ = std::make_shared<geometry_msgs::msg::Point32>(cur_bottom_right);

    }


//==============================
void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    auto callback_start = std::chrono::high_resolution_clock::now();
    
    if (!latest_odom_pose_.isZero(1e-6)) {
        static int frame_counter = 0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "shaixuan took %ld ms", duration1.count());
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        // voxel_filter.setInputCloud(cloud_filtered);
        // voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
        // voxel_filter.filter(*cloud_copy);
        //=====================================
        pcl::PointCloud<PointInT>::Ptr filtered_cloud(new pcl::PointCloud<PointInT>);
        pcl::PointCloud<PointInT>::Ptr filtered_cloud_non(new pcl::PointCloud<PointInT>);
        T_lidar_cam_ << -0.000192, 0.005236, -0.999986, 0.08,
                        0.997441, 0.071499, 0.000183, 0.05,
                        0.071499, -0.997427, -0.005237, -0.03,
                        0, 0, 0, 1;
        Eigen::Matrix4f T_cam_lidar_mat = T_lidar_cam_.inverse();
        Eigen::Affine3f T_cam_lidar;
        T_cam_lidar.matrix() = T_cam_lidar_mat;

        for (const auto& pt : cloud_filtered->points)
        {
            Eigen::Vector4f p_lidar(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cam = T_cam_lidar * p_lidar;
            float fx = 367.2095642089844f;
            float fy = 367.2040100097656f;
            float cx = 316.4378967285156f;
            float cy = 244.602783203125f;

            float u = (p_cam(0) / p_cam(2)) * fx + cx;
            float v = (p_cam(1) / p_cam(2)) * fy + cy;

            bool in_human = false;
                if (u >= xmin && u <= xmax && v >= ymin && v <= ymax && pt.z > -0.7) {
                    in_human = true;
                    filtered_cloud->points.push_back(pt);
                }
            if (!in_human) {
                filtered_cloud_non->points.push_back(pt);
                filtered_cloud_non->width = filtered_cloud_non->points.size();
                filtered_cloud_non->height = 1;
                filtered_cloud_non->is_dense = true;
            }
        }

        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        std::unordered_map<GridIndex, std::vector<int>, GridIndexHash> grid_points;
        for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
            const auto& pt = filtered_cloud->points[i];
            int gx = static_cast<int>(std::floor(pt.x / GRID_SIZE));
            int gy = static_cast<int>(std::floor(pt.y / GRID_SIZE));
            GridIndex idx{gx, gy};
            grid_points[idx].push_back(i);
        }

        size_t max_count = 0;
        GridIndex max_idx{0, 0};
        for (const auto& kv : grid_points) {
            if (kv.second.size() > max_count) {
                max_count = kv.second.size();
                max_idx = kv.first;
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
        for (int idx : grid_points[max_idx]) {
            new_cloud->push_back(filtered_cloud->points[idx]);
        }

        filtered_cloud->clear();
        *filtered_cloud = *new_cloud;

        if (!filtered_cloud->empty()) {
            Eigen::Vector4f centroid2;
            Eigen::Affine3f latest_odom_affine;
            latest_odom_affine.matrix() = latest_odom_pose_.cast<float>();

            pcl::transformPointCloud(*filtered_cloud, *new_cloud2, latest_odom_affine.matrix());
            pcl::compute3DCentroid(*new_cloud2, centroid2);

            pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            
            pcl::PointXYZI pt2;
            pt2.x = centroid2[0];
            pt2.y = centroid2[1];
            pt2.z = centroid2[2];
            pt2.intensity = 1.0f;
            centroid_cloud->push_back(pt2);
            if(xmin != 0.0 && ymin != 0.0 && ymax != 0.0 && xmax != 0.0){
                sensor_msgs::msg::PointCloud2 out_msg2;
                pcl::toROSMsg(*centroid_cloud, out_msg2);
                out_msg2.header.frame_id = "camera_init";
                centroid_pub->publish(out_msg2);
            }



            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(*new_cloud2, out_msg);
            out_msg.header = cloud_msg->header;
            pub_non_human_->publish(out_msg);
            filtered_cloud->clear();
        }

        auto callback_end2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end2 - callback_end1);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "touying took %ld ms", duration2.count());

        //===================================
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            // RCLCPP_INFO(this->get_logger(), "filtered_cloud_non has %zu points", filtered_cloud_non->points.size());
            // std::cout << "latest_odom_pose_:\n" << latest_odom_pose_ << std::endl;

            pcl::transformPointCloud(*filtered_cloud_non, *transformed_cloud, latest_odom_pose_.cast<float>());
        }
        
        if (transformed_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Transformed cloud is empty, skipping processing");
            return;
        }
        
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
        auto callback_end3 = std::chrono::high_resolution_clock::now();
        auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end3 - callback_end2);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "seg took %ld ms", duration3.count());        
        if (ground_inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No ground plane found by LMedS.");
            auto cloud_non_ground_rgb = convertToColoredCloud(cloud, 255, 0, 0);
            publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
            return;
        }
        
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "tiqu took %ld ms", duration4.count());        
        if (cloud_ground->size() < 100) {
            RCLCPP_WARN(this->get_logger(), "Too few ground points, skipping refinement.");
            auto cloud_ground_rgb = convertToColoredCloud(cloud_ground, 0, 255, 0);
            auto cloud_non_ground_rgb = convertToColoredCloud(cloud_non_ground, 255, 0, 0);
            publishCloud(cloud_ground_rgb, cloud_msg->header, pub_ground_);
            publishCloud(cloud_non_ground_rgb, cloud_msg->header, pub_non_ground_);
            return;
        }
        
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
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);
        float d = -normal.dot(centroid);
        float normal_norm = std::sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());
        float normal_norm_inv = 1.0f / normal_norm;





        
        float dot = normal.z(); 
        float cos_theta = std::abs(dot) * normal_norm_inv;
        cos_theta = std::max(0.0f, std::min(1.0f, cos_theta));
        float angle_deg = std::acos(cos_theta) * 180.0f / M_PI;
        
        pcl::PointCloud<PointInT>::Ptr cloud_refined_ground(new pcl::PointCloud<PointInT>);
        pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground(new pcl::PointCloud<PointInT>);
        cloud_refined_ground->reserve(transformed_cloud->points.size() / 2);
        cloud_refined_non_ground->reserve(transformed_cloud->points.size() / 2);
        
        std::vector<bool> is_ground(transformed_cloud->points.size(), false);
        for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
            const auto& pt = transformed_cloud->points[i];
            float dist = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
            
            if ((dist < 0.04f && angle_deg < 45)) {
                is_ground[i] = true;
            }
        }
        
        for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
            if (is_ground[i]) {
                cloud_refined_ground->points.push_back(transformed_cloud->points[i]);
            } else {
                cloud_refined_non_ground->points.push_back(transformed_cloud->points[i]);
            }
        }
        callback_end5 = std::chrono::high_resolution_clock::now();
        auto duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end5 - callback_end4);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "qufen took %ld ms", duration5.count());
        //=============================
        // pcl::KdTreeFLANN<pcl::PointXYZI> kdtree2;
        // kdtree2.setInputCloud(cloud_refined_ground);

        // float r_search = 0.4f;
        // std::vector<int> indices;
        // std::vector<float> sqr_distances;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pothole(new pcl::PointCloud<pcl::PointXYZI>);

        // for (const auto& pt : cloud_refined_non_ground->points) {
        //     if (kdtree2.radiusSearch(pt, r_search, indices, sqr_distances) > 0) {
        //         bool has_ground_above = false;
        //         for (int idx : indices) {
        //             const auto& gpt = cloud_refined_ground->points[idx];
        //             if (gpt.z > pt.z && std::abs(gpt.z - pt.z) < 0.5f) {
        //                 has_ground_above = true;
        //                 break;
        //             }
        //         }
        //         if (has_ground_above) {
        //             cloud_refined_ground->points.push_back(pt);
        //         } else {
        //             cloud_pothole->points.push_back(pt);
        //         }
        //     } else {
        //         cloud_pothole->points.push_back(pt);
        //     }
        // }

        // cloud_refined_non_ground->swap(*cloud_pothole);

        //================================================
        pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new(new pcl::PointCloud<PointInT>);
        pcl::PointCloud<PointInT>::Ptr cloud_refined_non_ground2_new2(new pcl::PointCloud<PointInT>);
        cloud_refined_non_ground2_new->reserve(cloud_refined_non_ground->points.size());
        cloud_refined_non_ground2_new2->reserve(cloud_refined_non_ground->points.size());
        auto callback_end6 = std::chrono::high_resolution_clock::now();
        auto duration6 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end6 - callback_end5);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "jinggai took %ld ms", duration6.count());
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(cloud_refined_non_ground);
        
        const float height_threshold = 0.2f;
        const float z_diff_threshold = 0.05f;
        const float search_radius = 0.1f;
        const int min_neighbors = 3;
        const float odom_radius_sq = 1.44f ; 
        
        std::vector<float> ground_dists(cloud_refined_non_ground->points.size());
        for (size_t i = 0; i < cloud_refined_non_ground->points.size(); ++i) {
            const auto& pt = cloud_refined_non_ground->points[i];
            ground_dists[i] = std::fabs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d) * normal_norm_inv;
        }
        
        for (size_t i = 0; i < cloud_refined_non_ground->points.size(); ++i) {
            const auto& pt = cloud_refined_non_ground->points[i];
            
            if (ground_dists[i] > height_threshold) {
                cloud_refined_non_ground2_new->points.push_back(pt);
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
                
                if (((z_max - z_min) < z_diff_threshold) ) {
                    cloud_refined_ground->points.push_back(pt);
                } else {

                    cloud_refined_non_ground2_new->points.push_back(pt);
                }
            } else {
                if (ground_dists[i] < height_threshold) {

                    cloud_refined_ground->points.push_back(pt);
                } else {

                    cloud_refined_non_ground2_new->points.push_back(pt);
                }
            }
        }
        callback_end7 = std::chrono::high_resolution_clock::now();
        auto duration7 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end7 - callback_end6);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "pingmian took %ld ms", duration7.count());        
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
    
    auto callback_end = std::chrono::high_resolution_clock::now();
    auto duration8 = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end - callback_end7);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "zuihou took %ld ms", duration8.count());
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