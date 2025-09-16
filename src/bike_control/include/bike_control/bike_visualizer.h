#ifndef BIKE_VISUALIZER_H
#define BIKE_VISUALIZER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>
#include <iomanip>
#include <sstream>
#include "bike_control/bike_controller.h"

class BikeVisualizer {
public:
    BikeVisualizer(rclcpp::Node* node, const std::string& topic_name,
                  double bike_length, double bike_width,
                  double front_length, double rear_length);
    void drawGoalPoint(double goal_x, double goal_y);
    // 设置网格地图
    void setGridMap(const std::vector<std::vector<int>>& grid_map,
                   double resolution,
                   double min_x, double min_y);
    
    void update(
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
        double goal_x, double goal_y);
    
    void enableFineGrid(bool enable) { draw_fine_grid_ = enable; }
    
private:
    void drawBike(double front_x, double front_y, double front_yaw,
                 double rear_x, double rear_y, double rear_yaw);
    void drawPerson(double px, double py);
    void drawGridMap();  // 绘制网格地图
    void drawGridAndInfo(double front_x, double front_y, double rear_x, double rear_y,
                        double yaw, double steering, double throttle,
                        double bike_speed, 
                        double person_x, double person_y);
    void drawTrajectories(
        const std::vector<BikeController::Trajectory>& all_trajectories,
        const std::vector<std::pair<double, double>>& best_trajectory);
    void publishVisualization();
    
    int worldToImageX(double world_x) const;
    int worldToImageY(double world_y) const;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
    cv::Mat visualization_image_;
    double scale_;
    double grid_resolution_;  // 网格分辨率（米）
    double bike_length_;
    double bike_width_;
    double front_len_;
    double rear_len_;
    cv::Scalar bike_color_;
    cv::Scalar person_color_;
    cv::Scalar front_traj_color_;
    cv::Scalar rear_traj_color_;
    cv::Scalar person_traj_color_;
    cv::Scalar obstacle_color_;  // 障碍物颜色
    cv::Scalar grid_color_;      // 网格线颜色
    cv::Scalar goal_point_color_ = cv::Scalar(0, 255, 255); // 预瞄点颜色黄色
    // 地图范围
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    
    // 网格地图 (0 = 可通行, 1 = 障碍)
    std::vector<std::vector<int>> grid_map_;
    
    // 添加成员变量
    bool draw_fine_grid_;  // 是否绘制精细网格
};

#endif // BIKE_VISUALIZER_H