#include "bike_control/trajectory_generator.h"
#include <cmath>
#include <iostream>

// 实现构造函数（初始化成员变量）
TrajectoryGenerator::TrajectoryGenerator() 
    : t_(0.0), x_(0.0), y_(0.0), 
      last_x_(0.0), last_y_(0.0), 
      current_v_(0.0) {}

void TrajectoryGenerator::update(double dt) {
    t_ += dt;
    double d = (5.0/3.6) * t_;
    
    // 保存上一时刻位置
    last_x_ = x_;
    last_y_ = y_;
    
    // 更新当前位置
    x_ = 10.0 + d;
    y_ = 2.0 * sin(4*M_PI*d/30.0);
    
    // 计算瞬时速度
    double dx = x_ - last_x_;
    double dy = y_ - last_y_;
    current_v_ = std::hypot(dx, dy) / dt;
    
    history_.emplace_back(x_, y_);
    if (history_.size() > 1000) history_.pop_front();
}

std::pair<double, double> TrajectoryGenerator::get_position() const {
    return {x_, y_};
}

const std::deque<std::pair<double, double>>& TrajectoryGenerator::get_history() const {
    return history_;
}

double TrajectoryGenerator::get_velocity() const {
    return current_v_;
}

void TrajectoryGenerator::update_Foxglove(double p_pos_x, double p_pos_y, double dt) {
    // 添加新点
    if (!last_point_received_) {
        // 第一个点：添加到轨迹中
        history_.emplace_back(p_pos_x, p_pos_y);
        last_point_received_ = true;
        return;
    }
    
    // 添加到队列前端（最新点）
    history_.push_front({p_pos_x, p_pos_y});
    
    // 确保最后一个点距离当前点约2米
    while (history_.size() > 1) {
        // 获取当前点（front）和最后一个点（back）
        const auto& last_point = history_.back();
        const auto& current_point = history_.front();
        
        // 计算两点距离
        double dx = current_point.first - last_point.first;
        double dy = current_point.second - last_point.second;
        double dist = std::hypot(dx, dy);
        
        double max_follow_distance = 0.1;
        // 如果距离大于0.1米，删除最后一个点
        if (dist > max_follow_distance) {
            history_.pop_back();
        } 
        else {
            break;
        }
    }
}