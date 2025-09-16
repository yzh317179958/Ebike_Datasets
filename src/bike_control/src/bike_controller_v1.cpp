#include "bike_control/bike_controller.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

BikeController::BikeController() :
    wheelbase_(1.0),      // 轴距
    max_speed_(0.7),      // 最大速度 (m/s)
    min_speed_(0.0),      // 最小速度
    max_steer_(M_PI/4),   // 最大转向角 (45度)
    max_accel_add(1.0),   // 最大加速度 (m/s²)
    max_accel_red(1.0),   // 最大减速度 (m/s²)
    dt_(0.1),
    max_dyaw_rate_(M_PI/4), // 最大偏航率 (45度/秒)
    predict_time_(1.5),   // 预测时间长度 (s)

    // // 代价权重
    // goal_cost_gain_(0.35),   // 目标代价增益
    // speed_cost_gain_(0.15),  // 速度代价增益
    // obstacle_cost_gain_(0.29), // 障碍物代价增益
    // to_goal_cost_gain_(0.25),  // 朝向目标代价增益

    // 代价权重
    // goal_cost_gain_(0.4),   // 目标代价增益
    // speed_cost_gain_(0.1),  // 速度代价增益
    // obstacle_cost_gain_(0.2), // 障碍物代价增益
    // to_goal_cost_gain_(0.30),  // 朝向目标代价增益

    goal_cost_gain_(0.5),   // 目标代价增益
    speed_cost_gain_(0.1),  // 速度代价增益
    obstacle_cost_gain_(0.1), // 障碍物代价增益
    to_goal_cost_gain_(0.3),  // 朝向目标代价增益

    last_steering_(0.0),  // 上一时刻转向角
    max_steer_rate_(30.0 * DEG_TO_RAD), // 30度/秒
    safety_margin_(0.3),  // 安全余量 (米)
    OBSTACLE_INFLUENCE_DIST(0.3) // 障碍物影响半径
{}

BikeController::Command BikeController::calculate(
    double bike_x, double bike_y, double bike_yaw, double bike_v,
    double person_x, double person_y,
    const std::deque<std::pair<double, double>>& global_path,
    const std::vector<std::vector<int>>& grid_map,
    double grid_resolution,
    double min_x, double min_y,
    std::vector<Trajectory>& all_trajectories,
    std::vector<std::pair<double, double>>& best_trajectory) 
{
    // 检查全局轨迹是否有效
    if (global_path.empty()) {
        return {last_steering_, -max_accel_red}; // 安全停止
    }
    
    // 直接从行人轨迹中获取最后一个点作为目标点
    double goal_x = global_path.back().first;
    double goal_y = global_path.back().second;
    
    // 使用这个目标点进行轨迹规划
    return calculateWithGoal(bike_x, bike_y, bike_yaw, bike_v, 
                            person_x, person_y, goal_x, goal_y,
                            grid_map, grid_resolution, min_x, min_y,
                            all_trajectories, best_trajectory);
}

BikeController::Command BikeController::calculateWithGoal(
    double bike_x, double bike_y, double bike_yaw, double bike_v,
    double person_x, double person_y, double goal_x, double goal_y,
    const std::vector<std::vector<int>>& grid_map,
    double grid_resolution,
    double min_x, double min_y,
    std::vector<Trajectory>& all_trajectories,
    std::vector<std::pair<double, double>>& best_trajectory) 
{
    // 记录目标点
    // last_goal_x_ = goal_x;
    // last_goal_y_ = goal_y;
    last_goal_x_ = person_x;
    last_goal_y_ = person_y;
    // 计算动态窗口
    double min_v = std::max(min_speed_, bike_v - max_accel_red * dt_);
    double max_v = std::min(max_speed_, bike_v + max_accel_add * dt_);
    double min_w = -max_dyaw_rate_;
    double max_w = max_dyaw_rate_;
    
    // 搜索最佳速度和角速度
    double best_total_cost = std::numeric_limits<double>::max();
    std::vector<TrajectoryData> all_traj_data;
    best_trajectory.clear();
    all_trajectories.clear();
    
    const int v_samples = 5;
    const int w_samples = 30; // 增加转向采样
    
    // 轨迹计数器
    int traj_count = 0;
    
    // 存储所有轨迹的原始代价
    std::vector<TrajectoryCost> all_costs;
    
    // 收集所有轨迹数据
    for (int i = 0; i <= v_samples; i++) {
        double v = min_v + i * (max_v - min_v) / v_samples;
        
        for (int j = 0; j <= w_samples; j++) {
            // 非线性采样（两端密集，中间稀疏）
            double ratio = static_cast<double>(j) / w_samples;
            double w = min_w + ratio * (max_w - min_w);
            
            // 创建轨迹数据对象
            TrajectoryData traj_data;
            traj_data.idx = traj_count++;
            traj_data.v = v;
            traj_data.w = w;
            
            // 模拟轨迹
            TrajectoryCost cost = simulate_trajectory(
                bike_x, bike_y, bike_yaw, v, w, 
                person_x, person_y,
                grid_map, grid_resolution,
                min_x, min_y,
                traj_data.path
            );
            
            // 存储代价
            traj_data.cost = cost;
            all_traj_data.push_back(traj_data);
            all_costs.push_back(cost);
        }
    }
    
    // 计算代价的最大最小值
    double min_goal_cost = std::numeric_limits<double>::max();
    double max_goal_cost = std::numeric_limits<double>::min();
    double min_to_goal_cost = std::numeric_limits<double>::max();
    double max_to_goal_cost = std::numeric_limits<double>::min();
    double min_speed_cost = std::numeric_limits<double>::max();
    double max_speed_cost = std::numeric_limits<double>::min();
    double min_obstacle_cost = std::numeric_limits<double>::max();
    double max_obstacle_cost = std::numeric_limits<double>::min();
    
    for (const auto& cost : all_costs) {
        if (!cost.collision) {
            min_goal_cost = std::min(min_goal_cost, cost.goal_cost);
            max_goal_cost = std::max(max_goal_cost, cost.goal_cost);
            min_to_goal_cost = std::min(min_to_goal_cost, cost.to_goal_cost);
            max_to_goal_cost = std::max(max_to_goal_cost, cost.to_goal_cost);
            min_speed_cost = std::min(min_speed_cost, cost.speed_cost);
            max_speed_cost = std::max(max_speed_cost, cost.speed_cost);
            min_obstacle_cost = std::min(min_obstacle_cost, cost.obstacle_cost);
            max_obstacle_cost = std::max(max_obstacle_cost, cost.obstacle_cost);
        }
    }
    
    // 归一化所有代价并计算总代价
    TrajectoryData best_traj_data;
    best_traj_data.idx = -1;
    
    // 在计算总代价之前，先计算各代价的总和
    double sum_goal_cost = 0.0;
    double sum_to_goal_cost = 0.0;
    double sum_speed_cost = 0.0;
    double sum_obstacle_cost = 0.0;
    int valid_count = 0;

    for (const auto& traj_data : all_traj_data) {
        if (!traj_data.cost.collision) {
            sum_goal_cost += traj_data.cost.goal_cost;
            sum_to_goal_cost += traj_data.cost.to_goal_cost;
            sum_speed_cost += traj_data.cost.speed_cost;
            sum_obstacle_cost += traj_data.cost.obstacle_cost;
            valid_count++;
        }
    }

    // 计算平均代价
    double avg_goal_cost = valid_count > 0 ? sum_goal_cost / valid_count : 0.0;
    double avg_to_goal_cost = valid_count > 0 ? sum_to_goal_cost / valid_count : 0.0;
    double avg_speed_cost = valid_count > 0 ? sum_speed_cost / valid_count : 0.0;
    double avg_obstacle_cost = valid_count > 0 ? sum_obstacle_cost / valid_count : 0.0;

    // 然后计算归一化代价
    for (auto& traj_data : all_traj_data) {
        const auto& cost = traj_data.cost;
        double total_cost = 0.0;
        
        if (cost.collision) {
            total_cost = std::numeric_limits<double>::max();
        } else {
            // 使用平均代价进行归一化
            double norm_goal = avg_goal_cost > 1e-5 ? cost.goal_cost / avg_goal_cost : 0.0;
            double norm_to_goal = avg_to_goal_cost > 1e-5 ? cost.to_goal_cost / avg_to_goal_cost : 0.0;
            double norm_speed = avg_speed_cost > 1e-5 ? cost.speed_cost / avg_speed_cost : 0.0;
            double norm_obstacle = avg_obstacle_cost > 1e-5 ? cost.obstacle_cost / avg_obstacle_cost : 0.0;
            
            // 计算总代价
            total_cost = goal_cost_gain_ * norm_goal
                    + to_goal_cost_gain_ * norm_to_goal
                    + speed_cost_gain_ * norm_speed
                    + obstacle_cost_gain_ * norm_obstacle;
        }
        
        // 存储轨迹和代价
        Trajectory traj;
        traj.path = traj_data.path;
        traj.cost.total_cost = total_cost;
        traj.cost.goal_cost = cost.goal_cost;
        traj.cost.to_goal_cost = cost.to_goal_cost;
        traj.cost.speed_cost = cost.speed_cost;
        traj.cost.obstacle_cost = cost.obstacle_cost;
        all_trajectories.push_back(traj);
        
        // 更新最佳轨迹
        if (total_cost < best_total_cost) {
            best_total_cost = total_cost;
            best_traj_data = traj_data;
        }
    }
    
    // 直接提取最佳轨迹的动作
    best_trajectory = best_traj_data.path;
    double best_v = best_traj_data.v;
    double best_w = best_traj_data.w;
    

    // ==== 添加最佳轨迹代价分析 ====
    if (best_traj_data.idx != -1) {
        const auto& cost = best_traj_data.cost;
        
        auto normalize = [](double value, double min_val, double max_val) {
            if (max_val - min_val < 1e-5) return 0.0;
            return (value - min_val) / (max_val - min_val);
        };
        
        // 使用之前计算的平均代价进行归一化
        double norm_goal = avg_goal_cost > 1e-5 ? cost.goal_cost / avg_goal_cost : 0.0;
        double norm_to_goal = avg_to_goal_cost > 1e-5 ? cost.to_goal_cost / avg_to_goal_cost : 0.0;
        double norm_speed = avg_speed_cost > 1e-5 ? cost.speed_cost / avg_speed_cost : 0.0;
        double norm_obstacle = avg_obstacle_cost > 1e-5 ? cost.obstacle_cost / avg_obstacle_cost : 0.0;
        
        // 计算加权代价
        double weighted_goal = goal_cost_gain_ * norm_goal;
        double weighted_to_goal = to_goal_cost_gain_ * norm_to_goal;
        double weighted_speed = speed_cost_gain_ * norm_speed;
        double weighted_obstacle = obstacle_cost_gain_ * norm_obstacle;
        
        // 输出最佳轨迹关键参数
        std::cout << "\n===== 最佳轨迹参数分析 =====" << std::endl;
        std::cout << "轨迹ID: " << best_traj_data.idx << std::endl;
        
        // 输出速度和角速度
        std::cout << "速度: " << best_v << " m/s" << std::endl;
        std::cout << "角速度: " << best_w << " rad/s (" << best_w * RAD_TO_DEG << "°/s)" << std::endl;
        
        // 计算理论转向角
        double theoretical_steering = 0.0;
        if (std::abs(best_v) > 0.1) {
            theoretical_steering = std::atan(wheelbase_ * best_w / best_v);
        } else {
            theoretical_steering = best_w * 0.5;
        }
        std::cout << "理论转向角: " << theoretical_steering * RAD_TO_DEG << "°" << std::endl;
        
        // 输出原始代价
        std::cout << "\n===== 原始代价 =====" << std::endl;
        std::cout << "目标距离代价: " << cost.goal_cost << std::endl;
        std::cout << "朝向目标代价: " << cost.to_goal_cost << std::endl;
        std::cout << "速度代价: " << cost.speed_cost << std::endl;
        std::cout << "障碍物代价: " << cost.obstacle_cost << std::endl;
        
        // 输出归一化代价
        std::cout << "\n===== 归一化代价 =====" << std::endl;
        std::cout << "目标距离: " << norm_goal << std::endl;
        std::cout << "朝向目标: " << norm_to_goal << std::endl;
        std::cout << "速度: " << norm_speed << std::endl;
        std::cout << "障碍物: " << norm_obstacle << std::endl;
        
        // 输出加权代价
        std::cout << "\n===== 加权代价 =====" << std::endl;
        std::cout << "目标距离(" << goal_cost_gain_ << "): " << weighted_goal << std::endl;
        std::cout << "朝向目标(" << to_goal_cost_gain_ << "): " << weighted_to_goal << std::endl;
        std::cout << "速度(" << speed_cost_gain_ << "): " << weighted_speed << std::endl;
        std::cout << "障碍物(" << obstacle_cost_gain_ << "): " << weighted_obstacle << std::endl;
        
        // 输出总代价
        double total_cost = weighted_goal + weighted_to_goal + weighted_speed + weighted_obstacle;
        std::cout << "\n总代价: " << total_cost << std::endl;
        
        // 输出轨迹终点信息
        if (!best_traj_data.path.empty()) {
            auto& last_point = best_traj_data.path.back();
            double end_dist = std::hypot(last_point.first - goal_x, last_point.second - goal_y);
            double end_yaw = atan2(last_point.second - bike_y, last_point.first - bike_x);
            double angle_diff = std::fabs(std::fmod((end_yaw - bike_yaw) + M_PI, 2*M_PI) - M_PI);
            
            std::cout << "\n===== 轨迹终点信息 =====" << std::endl;
            std::cout << "终点位置: (" << last_point.first << ", " << last_point.second << ")" << std::endl;
            std::cout << "终点到目标距离: " << end_dist << " m" << std::endl;
            std::cout << "终点偏航误差: " << angle_diff * RAD_TO_DEG << "°" << std::endl;
        }
        
        std::cout << "===========================\n" << std::endl;
    }
    // ==== 结束详细分析 ====


    // // 转换为控制指令
    // double steering = 0.0;
    // if (std::abs(best_v) > 0.1) {
    //     steering = std::atan(wheelbase_ * best_w / best_v);
    // } else {
    //     steering = best_w * 0.5;
    // }

    
    // // 添加转向角变化率约束
    // double max_steer_change = max_steer_rate_ * dt_;
    // steering = std::clamp(steering, 
    //                       last_steering_ - max_steer_change,
    //                       last_steering_ + max_steer_change);
    
    // // 确保在物理极限范围内
    // steering = std::clamp(steering, -max_steer_, max_steer_);
    

    // 直接使用轨迹规划结果
    double steering = atan2(wheelbase_ * best_w, std::max(0.0, best_v));

    // 加入低通滤波（更平滑的过渡）
    const double filter_gain = 0.7; // 保留70%新值，30%旧值
    steering = filter_gain * steering + (1.0 - filter_gain) * last_steering_;
    // 更新上次转向角
    
    
    // // 加速度输出
    // double throttle = (best_v - bike_v) / dt_;
    // throttle = std::clamp(throttle, -max_accel_red, max_accel_add);

    // 速度输出
    double throttle = best_v;
    throttle = std::clamp(throttle, 0.0, max_speed_);
    std::cout<<"速度:  "<< throttle <<std::endl;
    
    // 行人安全距离检测
    double dx = person_x - bike_x;
    double dy = person_y - bike_y;
    double distance = std::hypot(dx, dy);
    std::cout<<"人与车之间距离:  "<< distance <<std::endl;
    if (distance < 1.5) {
        throttle = -0.6;
    }

    return {steering, throttle};
}

BikeController::TrajectoryCost BikeController::simulate_trajectory(
    double x, double y, double yaw, double v, double w,
    double goal_x, double goal_y,
    const std::vector<std::vector<int>>& grid_map,
    double grid_resolution,
    double min_x, double min_y,
    std::vector<std::pair<double, double>>& trajectory) 
{
    TrajectoryCost cost;
    cost.goal_cost = 0.0;
    cost.to_goal_cost = 0.0;
    cost.speed_cost = 0.0;
    cost.obstacle_cost = 0.0; // 初始化为0
    cost.collision = false;
    
    double min_obs_dist = std::numeric_limits<double>::max();
    trajectory.clear();
    double initial_x = x;
    double initial_y = y;
    // 模拟轨迹
    for (double t = 0.0; t <= predict_time_; t += dt_) {
        // 更新状态
        yaw += w * dt_;
        x += v * std::cos(yaw) * dt_;
        y += v * std::sin(yaw) * dt_;
        
        // 存储轨迹点
        trajectory.push_back({x, y});
        
        // 检查障碍物碰撞
        double obs_dist = calculateObstacleDistance(x, y, yaw, grid_map, grid_resolution, min_x, min_y);
        
        // 碰撞检测（考虑安全余量）
        if (obs_dist < safety_margin_) {
            cost.collision = true;
            cost.obstacle_cost = std::numeric_limits<double>::max();
            return cost;
        }
        
        // 更新最小障碍物距离
        min_obs_dist = std::min(min_obs_dist, obs_dist);
        
        // 计算到目标的距离
        double dx = goal_x - x;
        double dy = goal_y - y;
        double goal_dist = std::hypot(dx, dy);
        cost.goal_cost += goal_dist * (t/predict_time_ + 0.1); // 随时间增加权重
        
        // 计算朝向目标的代价
        // // 使用初始位置计算角度差
        // double initial_target_angle = std::atan2(goal_y - initial_y, goal_x - initial_x);
        // double initial_angle_diff = initial_target_angle - yaw;
        // initial_angle_diff = std::fmod(initial_angle_diff + M_PI, 2*M_PI) - M_PI;
        // cost.to_goal_cost = std::abs(initial_angle_diff);
        

        // 计算朝向目标的代价
        double target_angle = std::atan2(dy, dx);
        double angle_diff = std::abs(std::fmod(target_angle - yaw + M_PI, 2*M_PI) - M_PI);
        cost.to_goal_cost += angle_diff * (t/predict_time_ + 0.1); // 随时间增加权重

        // 速度代价
        cost.speed_cost += (max_speed_ - v);
    }
    
    // 计算障碍物代价（指数衰减函数）
    if (min_obs_dist < OBSTACLE_INFLUENCE_DIST) {
        cost.obstacle_cost = std::exp(-min_obs_dist); // 距离越近代价越大
    } else {
        cost.obstacle_cost = 0.0; // 超出影响范围不计代价
    }
    
    return cost;
}

double BikeController::calculateObstacleDistance(
    double x, double y, double yaw,
    const std::vector<std::vector<int>>& grid_map,
    double grid_resolution,
    double min_x, double min_y) 
{
    double min_dist = std::numeric_limits<double>::max();
    
    // 将世界坐标转换为网格坐标
    int grid_x = static_cast<int>((x - min_x) / grid_resolution);
    int grid_y = static_cast<int>((y - min_y) / grid_resolution);
    
    // 检查坐标是否在网格范围内
    if (grid_y < 0 || grid_y >= grid_map.size() || 
        grid_x < 0 || grid_x >= grid_map[0].size()) {
        return min_dist; // 返回大值表示安全
    }
    
    // 计算前轮位置
    double front_x = x + wheelbase_ * cos(yaw);
    double front_y = y + wheelbase_ * sin(yaw);
    
    // 计算障碍物圆的半径
    const double obstacle_radius = grid_resolution / 2.0;
    
    // 检查自行车周围的网格
    int search_radius = static_cast<int>(OBSTACLE_INFLUENCE_DIST / grid_resolution) + 1;
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;
            
            if (check_y >= 0 && check_y < grid_map.size() && 
                check_x >= 0 && check_x < grid_map[check_y].size()) 
            {
                if (grid_map[check_y][check_x] == 100) {
                    // 计算障碍物中心的世界坐标
                    double obs_x = min_x + (check_x + 0.5) * grid_resolution;
                    double obs_y = min_y + (check_y + 0.5) * grid_resolution;
                    
                    // 计算到后轮的距离
                    double rear_dist = std::hypot(x - obs_x, y - obs_y) - obstacle_radius;
                    
                    // 计算到前轮的距离
                    double front_dist = std::hypot(front_x - obs_x, front_y - obs_y) - obstacle_radius;
                    
                    // 取两者中较小的距离
                    double dist = std::min(rear_dist, front_dist);
                    
                    min_dist = std::min(min_dist, dist);
                }
            }
        }
    }
    
    return min_dist;
}

BikeController::MotorCommand BikeController::convertToMotorCommand(
    const Command& cmd, double current_speed,double dt ,int speed_limit) 
{
    MotorCommand motor_cmd;
    
    // 1. 转把电机控制（转向）
    // 将转向角度（弧度）转换为协议单位（0.1度）
    double steering_deg = cmd.steering * (180.0 / M_PI); // 弧度转角度
    
    // 限制在±45度范围内
    steering_deg = std::clamp(steering_deg, -45.0, 45.0);
    
    // 转换为协议单位（0.1度）
    motor_cmd.rotor_steering = static_cast<int>(steering_deg * 10.0);
    
    // 限制范围（-450到450）
    motor_cmd.rotor_steering = std::clamp(motor_cmd.rotor_steering, -450, 450);
    
    // 2. 轮毂电机控制（油门）
    if (cmd.throttle >= 0) {
        // 正油门：加速
        motor_cmd.hub_throttle = static_cast<int>((cmd.throttle*3.6*10));
    } else {
        // 负油门：刹车，轮毂电机不工作
        motor_cmd.hub_throttle = 0;
    }
    
    // 限制范围（0-80）
    motor_cmd.hub_throttle = std::clamp(motor_cmd.hub_throttle, 0, speed_limit);
    
    // 3. 刹车电机控制
    double brake_value = 0.0;
    if (cmd.throttle < 0) {
        // 负油门值转换为刹车力度
        brake_value = -cmd.throttle * 1000.0;
    }
    
    // 限制刹车力度范围
    brake_value = std::clamp(brake_value, 0.0, 1000.0);
    int brake_int = static_cast<int>(brake_value);
    
    // 假设前后刹车平均分配刹车力度
    motor_cmd.front_brake = brake_int;
    motor_cmd.rear_brake = brake_int;
    
    return motor_cmd;
}