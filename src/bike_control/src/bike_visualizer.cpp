#include "bike_control/bike_visualizer.h"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <random>
#include <algorithm>

BikeVisualizer::BikeVisualizer(rclcpp::Node* node, const std::string& topic_name,
                             double bike_length, double bike_width,
                             double front_length, double rear_length)
    : visualization_pub_(node->create_publisher<sensor_msgs::msg::Image>(topic_name, 10)),
      visualization_image_(800, 1000, CV_8UC3, cv::Scalar(255, 255, 255)),
      scale_(40.0),// 缩放系数
      grid_resolution_(0.1),  // 默认分辨率，将被实际值覆盖
      bike_length_(bike_length),
      bike_width_(bike_width),
      front_len_(front_length),
      rear_len_(rear_length),
      bike_color_(100, 100, 100),
      person_color_(0, 255, 0),
      front_traj_color_(0, 0, 255),
      rear_traj_color_(255, 0, 0),
      person_traj_color_(0, 255, 0),
      obstacle_color_(0, 0, 0),  // 黑色障碍物
      grid_color_(200, 200, 200), // 网格线颜色
      min_x_(0.0), max_x_(50.0), min_y_(0.0), max_y_(40.0),
      draw_fine_grid_(true)  // 默认关闭
{
    // 初始化网格地图为空
    grid_map_ = std::vector<std::vector<int>>();
}

void BikeVisualizer::setGridMap(const std::vector<std::vector<int>>& grid_map, 
                               double resolution,
                               double min_x, double min_y) {
    grid_map_ = grid_map;
    grid_resolution_ = resolution;
    min_x_ = min_x;
    min_y_ = min_y;
    max_x_ = min_x_ + grid_map_[0].size() * resolution;
    max_y_ = min_y_ + grid_map_.size() * resolution;
}

void BikeVisualizer::update(
    double front_x, double front_y, double front_yaw,
    double rear_x, double rear_y, double rear_yaw,
    double person_x, double person_y,
    double steering, double throttle,
    const std::deque<std::pair<double, double>>& person_history,
    const std::deque<std::pair<double, double>>& front_history,
    const std::deque<std::pair<double, double>>& rear_history,
    double bike_speed,
    const std::vector<BikeController::Trajectory>& all_trajectories,
    const std::vector<std::pair<double, double>>& best_trajectory,
    double goal_x, double goal_y) {
    
    // 清空图像为白色
    visualization_image_.setTo(cv::Scalar(255, 255, 255));
    
    // 先绘制网格地图
    drawGridMap();
    
    // 再绘制轨迹和其他信息
    drawTrajectories(all_trajectories, best_trajectory);
    drawGridAndInfo(front_x, front_y, rear_x, rear_y, front_yaw, steering, throttle, bike_speed,person_x,person_y);
    
    // 绘制前轮轨迹(蓝色)
    for(size_t i = 1; i < front_history.size(); i++) {
        int x1 = worldToImageX(front_history[i-1].first);
        int y1 = worldToImageY(front_history[i-1].second);
        int x2 = worldToImageX(front_history[i].first);
        int y2 = worldToImageY(front_history[i].second);
        cv::line(visualization_image_, 
                cv::Point(x1, y1), cv::Point(x2, y2),
                front_traj_color_, 2);
    }
    
    // 绘制后轮轨迹(青色)
    for(size_t i = 1; i < rear_history.size(); i++) {
        int x1 = worldToImageX(rear_history[i-1].first);
        int y1 = worldToImageY(rear_history[i-1].second);
        int x2 = worldToImageX(rear_history[i].first);
        int y2 = worldToImageY(rear_history[i].second);
        cv::line(visualization_image_, 
                cv::Point(x1, y1), cv::Point(x2, y2),
                rear_traj_color_, 2);
    }
    
    // 绘制行人轨迹(绿色)
    for(size_t i = 1; i < person_history.size(); i++) {
        int x1 = worldToImageX(person_history[i-1].first);
        int y1 = worldToImageY(person_history[i-1].second);
        int x2 = worldToImageX(person_history[i].first);
        int y2 = worldToImageY(person_history[i].second);
        cv::line(visualization_image_, 
                cv::Point(x1, y1), cv::Point(x2, y2),
                person_traj_color_, 2);
    }

    drawGoalPoint(goal_x, goal_y);
    // 最后绘制自行车和行人（确保在最上层）
    drawBike(front_x, front_y, front_yaw, rear_x, rear_y, rear_yaw);
    drawPerson(person_x, person_y);

    // 发布可视化图像
    publishVisualization();
}

// 坐标转换函数
int BikeVisualizer::worldToImageX(double world_x) const {
    return static_cast<int>((world_x - min_x_) * scale_);
}

int BikeVisualizer::worldToImageY(double world_y) const {
    return visualization_image_.rows - static_cast<int>((world_y - min_y_) * scale_);
}

void BikeVisualizer::drawGridMap() {
    // 如果没有地图数据，不绘制
    if (grid_map_.empty()) return;
    
    int grid_width = grid_map_[0].size();
    int grid_height = grid_map_.size();
    
    for (int gy = 0; gy < grid_height; gy++) {
        for (int gx = 0; gx < grid_width; gx++) {
            // 如果值大于0（障碍物），绘制障碍物
            if (grid_map_[gy][gx] ==100 ) {
                // 计算网格的世界坐标
                double world_x = min_x_ + gx * grid_resolution_;
                double world_y = min_y_ + gy * grid_resolution_;
                
                // 转换为图像坐标
                int img_x = worldToImageX(world_x);
                int img_y = worldToImageY(world_y);
                
                // 计算网格大小（像素）
                int cell_size = static_cast<int>(grid_resolution_ * scale_);
                
                // 绘制黑色方块（障碍物）
                cv::rectangle(visualization_image_,
                            cv::Point(img_x, img_y),
                            cv::Point(img_x + cell_size, img_y + cell_size),
                            obstacle_color_, -1); // -1 表示实心
            }
        }
    }
}

void BikeVisualizer::drawBike(double front_x, double front_y, double front_yaw,
                              double rear_x, double rear_y, double rear_yaw) {
    // 1. 计算前后轮在图像中的位置
    int front_img_x = worldToImageX(front_x);
    int front_img_y = worldToImageY(front_y);
    int rear_img_x = worldToImageX(rear_x);
    int rear_img_y = worldToImageY(rear_y);
    
    // 2. 定义轮子尺寸
    double wheel_width = bike_width_ / 2.0;
    double wheel_length = bike_length_ / 5.0; // 轮子长度
    
    // 3. 绘制前轮长方形
    std::vector<cv::Point> front_wheel = {
        cv::Point(-wheel_length*scale_, wheel_width*scale_),
        cv::Point(wheel_length*scale_, wheel_width*scale_),
        cv::Point(wheel_length*scale_, -wheel_width*scale_),
        cv::Point(-wheel_length*scale_, -wheel_width*scale_)
    };
    
    // 旋转前轮（考虑前轮偏航角）
    double front_wheel_angle = front_yaw * 180 / M_PI;
    cv::Mat front_rot_mat = cv::getRotationMatrix2D(cv::Point(0, 0), front_wheel_angle, 1.0);
    cv::transform(front_wheel, front_wheel, front_rot_mat);
    
    // 平移至前轮位置
    for(auto& p : front_wheel) {
        p.x += front_img_x;
        p.y += front_img_y;
    }
    
    cv::fillPoly(visualization_image_, 
                std::vector<std::vector<cv::Point>>{front_wheel},
                cv::Scalar(0, 0, 255)); // 前轮红色
    
    // 4. 绘制后轮长方形
    std::vector<cv::Point> rear_wheel = {
        cv::Point(-wheel_length*scale_, wheel_width*scale_),
        cv::Point(wheel_length*scale_, wheel_width*scale_),
        cv::Point(wheel_length*scale_, -wheel_width*scale_),
        cv::Point(-wheel_length*scale_, -wheel_width*scale_)
    };
    
    // 旋转后轮（考虑后轮偏航角）
    double rear_wheel_angle = rear_yaw * 180 / M_PI;
    cv::Mat rear_rot_mat = cv::getRotationMatrix2D(cv::Point(0, 0), rear_wheel_angle, 1.0);
    cv::transform(rear_wheel, rear_wheel, rear_rot_mat);
    
    // 平移至后轮位置
    for(auto& p : rear_wheel) {
        p.x += rear_img_x;
        p.y += rear_img_y;
    }
    
    cv::fillPoly(visualization_image_, 
                std::vector<std::vector<cv::Point>>{rear_wheel},
                cv::Scalar(255, 0, 0)); // 后轮蓝色
    
    // 5. 连接前后轮（车身）
    cv::Point front_center(front_img_x, front_img_y);
    cv::Point rear_center(rear_img_x, rear_img_y);
    cv::line(visualization_image_, front_center, rear_center, 
            cv::Scalar(100, 100, 100), 3); // 灰色车身
    
    // 6. 标记前轮中心（白色）
    cv::circle(visualization_image_, front_center, 
              3, cv::Scalar(255, 255, 255), -1);
    
    // 7. 标记后轮中心（白色）
    cv::circle(visualization_image_, rear_center, 
              3, cv::Scalar(255, 255, 255), -1);
    
    // 8. 添加方向指示箭头（从前轮中心指向前轮方向）
    cv::arrowedLine(visualization_image_,
                   front_center,
                   cv::Point(front_center.x + 30 * cos(front_yaw), 
                            front_center.y - 30 * sin(front_yaw)),  // 注意：图像坐标系y向下
                   cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
}

void BikeVisualizer::drawPerson(double px, double py) {
    int person_x = worldToImageX(px);
    int person_y = worldToImageY(py);
    
    // 绘制行人图标（十字形）
    cv::line(visualization_image_, 
            cv::Point(person_x-8, person_y), cv::Point(person_x+8, person_y),
            person_color_, 2);
    cv::line(visualization_image_,
            cv::Point(person_x, person_y-8), cv::Point(person_x, person_y+8),
            person_color_, 2);
}

void BikeVisualizer::drawGridAndInfo(double front_x, double front_y, double rear_x, double rear_y,
                                    double yaw, double steering, double throttle,
                                    double bike_speed, 
                                    double person_x, double person_y) {
    // 绘制网格线（基于固定地图范围）
    // 如果开启精细网格绘制
    if (draw_fine_grid_) {
        double grid_size = 0.1; // 精细网格大小0.1米
        
        // X方向精细网格线（垂直线）
        for(double x = min_x_; x <= max_x_; x += grid_size) {
            int px = worldToImageX(x);
            cv::line(visualization_image_,
                    cv::Point(px, 0),
                    cv::Point(px, visualization_image_.rows),
                    cv::Scalar(200, 200, 200), 1); // 浅灰色
        }
        
        // Y方向精细网格线（水平线）
        for(double y = min_y_; y <= max_y_; y += grid_size) {
            int py = worldToImageY(y);
            cv::line(visualization_image_,
                    cv::Point(0, py),
                    cv::Point(visualization_image_.cols, py),
                    cv::Scalar(200, 200, 200), 1); // 浅灰色
        }
    }
    
    // 仍然绘制粗网格（1米网格）
    double coarse_grid_size = 1.0; // 粗网格大小1米
    
    // X方向粗网格线（垂直线）
    for(double x = min_x_; x <= max_x_; x += coarse_grid_size) {
        int px = worldToImageX(x);
        cv::line(visualization_image_,
                cv::Point(px, 0),
                cv::Point(px, visualization_image_.rows),
                grid_color_, 2); // 较深颜色，粗线
    }
    
    // Y方向粗网格线（水平线）
    for(double y = min_y_; y <= max_y_; y += coarse_grid_size) {
        int py = worldToImageY(y);
        cv::line(visualization_image_,
                cv::Point(0, py),
                cv::Point(visualization_image_.cols, py),
                grid_color_, 2); // 较深颜色，粗线
    }

    // 计算到行人的距离
    double dx = person_x - front_x;
    double dy = person_y - front_y;
    double distance_to_person = std::hypot(dx, dy);

    // 将转向角显示为度
    double steering_deg = steering * 180.0 / M_PI;
    std::string info = "Steering: " + std::to_string(steering_deg).substr(0,5) + "deg | " +
                     "Throttle: " + std::to_string(throttle).substr(0,4);
    cv::putText(visualization_image_, info, cv::Point(20, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

    std::string distance_info = "Dist to Person: " + std::to_string(distance_to_person).substr(0,4) + " m";
    cv::putText(visualization_image_, distance_info, cv::Point(20, 60),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);    
    
    // 将偏航角显示为度
    double yaw_deg = yaw * 180.0 / M_PI;
    std::string yaw_info = "Yaw: " + std::to_string(yaw_deg).substr(0,6) + "deg";
    cv::putText(visualization_image_, yaw_info, cv::Point(20, 90),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    // 添加速度信息
    std::string speed_info = "Speed: " + std::to_string(bike_speed).substr(0,4) + " m/s";
    cv::putText(visualization_image_, speed_info, cv::Point(20, 120),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
}

void BikeVisualizer::drawTrajectories(
    const std::vector<BikeController::Trajectory>& all_trajectories,
    const std::vector<std::pair<double, double>>& best_trajectory) {
    
    // 绘制所有轨迹（深绿色）
    for (const auto& traj : all_trajectories) {
        // if (traj.cost.total_cost >= std::numeric_limits<double>::max()) continue;
        
        for (size_t i = 1; i < traj.path.size(); i++) {
            int x1 = worldToImageX(traj.path[i-1].first);
            int y1 = worldToImageY(traj.path[i-1].second);
            int x2 = worldToImageX(traj.path[i].first);
            int y2 = worldToImageY(traj.path[i].second);
            
            cv::line(visualization_image_, 
                    cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(0, 100, 0), 1); // 深绿色
        }
    }
    
    // 绘制最佳轨迹（红色）
    for (size_t i = 1; i < best_trajectory.size(); i++) {
        int x1 = worldToImageX(best_trajectory[i-1].first);
        int y1 = worldToImageY(best_trajectory[i-1].second);
        int x2 = worldToImageX(best_trajectory[i].first);
        int y2 = worldToImageY(best_trajectory[i].second);
        
        cv::line(visualization_image_, 
                cv::Point(x1, y1), cv::Point(x2, y2),
                cv::Scalar(0, 0, 255), 2); // 红色
    }
}

void BikeVisualizer::publishVisualization() {
    auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", visualization_image_).toImageMsg();
    visualization_pub_->publish(*msg);
}

void BikeVisualizer::drawGoalPoint(double goal_x, double goal_y) {
    // 转换为图像坐标
    int img_x = worldToImageX(goal_x);
    int img_y = worldToImageY(goal_y);
    
    // 绘制醒目的目标点标记
    cv::circle(visualization_image_, cv::Point(img_x, img_y), 
              10, goal_point_color_, 2); // 外圆
    cv::circle(visualization_image_, cv::Point(img_x, img_y), 
              4, goal_point_color_, -1); // 内圆
    
    // 添加十字标记
    cv::line(visualization_image_, 
             cv::Point(img_x - 8, img_y), cv::Point(img_x + 8, img_y),
             goal_point_color_, 2);
    cv::line(visualization_image_, 
             cv::Point(img_x, img_y - 8), cv::Point(img_x, img_y + 8),
             goal_point_color_, 2);
}