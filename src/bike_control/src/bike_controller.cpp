#include "bike_control/bike_controller.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <chrono>   
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

BikeController::BikeController() :
    wheelbase_(1.0),      // 轴距
    max_speed_(1.0),      // 最大速度 (m/s)
    min_speed_(0.0),      // 最小速度
    max_steer_(M_PI/3),   // 最大转向角 (60度)
    max_accel_add(1.0),   // 最大加速度 (m/s²)
    max_accel_red(1.0),   // 最大减速度 (m/s²)
    dt_(0.1),
    max_dyaw_rate_(M_PI/3), // 最大偏航率 (60度/秒)
    predict_time_(3.5),   // 预测时间长度 (s)

    goal_cost_gain_(0.1),   // 目标代价增益
    speed_cost_gain_(0.2),  // 速度代价增益
    obstacle_cost_gain_(0.55), // 障碍物代价增益
    to_goal_cost_gain_(0.05),  // 朝向目标代价增益

    // goal_cost_gain_(0.1),   // 目标代价增益
    // speed_cost_gain_(0.05),  // 速度代价增益
    // obstacle_cost_gain_(0.7), // 障碍物代价增益
    // to_goal_cost_gain_(0.1),  // 朝向目标代价增益

    last_steering_(0.0),  // 上一时刻转向角
    max_steer_rate_(60.0 * DEG_TO_RAD), // 30度/秒
    safety_margin_(0.0),  // 安全余量 (米)
    OBSTACLE_INFLUENCE_DIST(0.8) // 障碍物影响半径
{}

void BikeController::init(double wheelbase, double max_speed, double min_speed, 
        double max_steer, double max_accel_add, double max_accel_red,
        double max_dyaw_rate, double predict_time, double goal_cost_gain,
        double speed_cost_gain, double obstacle_cost_gain, 
        double to_goal_cost_gain, double max_steer_rate,
        double safety_margin, double obstacle_influence_dist) {
    
    // 使用参数初始化成员变量
    wheelbase_ = wheelbase;
    max_speed_ = max_speed;
    min_speed_ = min_speed;
    max_steer_ = max_steer * DEG_TO_RAD; // 度转弧度
    max_accel_add = max_accel_add;
    max_accel_red = max_accel_red;
    max_dyaw_rate_ = max_dyaw_rate * DEG_TO_RAD; // 度转弧度
    predict_time_ = predict_time;
    goal_cost_gain_ = goal_cost_gain;
    speed_cost_gain_ = speed_cost_gain;
    obstacle_cost_gain_ = obstacle_cost_gain;
    to_goal_cost_gain_ = to_goal_cost_gain;
    max_steer_rate_ = max_steer_rate * DEG_TO_RAD; // 度转弧度
    safety_margin_ = safety_margin;
    OBSTACLE_INFLUENCE_DIST = obstacle_influence_dist;
}

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
    last_goal_x_ = goal_x;
    last_goal_y_ = goal_y;
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
    
    const int v_samples = 2;
    const int w_samples = 30; // 增加转向采样
    
    // 轨迹计数器
    int traj_count = 0;
    
    // 存储所有轨迹的原始代价
    std::vector<TrajectoryCost> all_costs;

    double person_vx = (person_x - prev_person_x) ;
    double person_vy = (person_y - prev_person_y) ;
    double distance_person = std::hypot(person_vx, person_vy);
    double person_speed_1 = distance_person / 0.1; // 行人速度大小

    // 收集所有轨迹数据
    for (int i = 1; i <= v_samples; i++) {
        auto cycle_start = std::chrono::high_resolution_clock::now();
        double v = min_v + i * (max_v - min_v) / v_samples;
        for (int j = 1; j <= w_samples; j++) {
            // 非线性采样（两端密集，中间稀疏）
            double ratio = static_cast<double>(j) / w_samples;
            double w = min_w + (max_w - min_w) * 
                       (0.5 - 0.5 * std::cos(ratio * M_PI));
            // // 均匀采样
            // double ratio = static_cast<double>(j) / w_samples;
            // double w = min_w + ratio * (max_w - min_w);
            
            // 创建轨迹数据对象
            TrajectoryData traj_data;
            traj_data.idx = traj_count++;
            traj_data.v = v;
            traj_data.w = w;
            
            // 模拟轨迹
            TrajectoryCost cost = simulate_trajectory(
                bike_x, bike_y, bike_yaw, v, w, 
                person_x, person_y, goal_x, goal_y,
                grid_map, grid_resolution,
                min_x, min_y,
                traj_data.path,
                person_speed_1
            );
            
            // 存储代价
            traj_data.cost = cost;
            all_traj_data.push_back(traj_data);
            all_costs.push_back(cost);
        }

    auto cycle_end_1 = std::chrono::high_resolution_clock::now();
    
    // 计算执行时间（毫秒）
    auto cycle_duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end_1 - cycle_start);
    
    // 记录执行时间
    std::string time_msg_1 = "控制周期执行时间: " + std::to_string(cycle_duration_1.count()) + " ms";
    std::cout << "运行时间："<<time_msg_1 << std::endl;
    
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
        double weighted_goal = 0;
        double weighted_to_goal = 0;
        double weighted_speed = 0;
        double weighted_obstacle = 0;
        if (cost.collision) {
            total_cost = std::numeric_limits<double>::max();
        } else {
            // 使用平均代价进行归一化
            double norm_goal = avg_goal_cost > 1e-5 ? cost.goal_cost / avg_goal_cost : 0.0;
            double norm_to_goal = avg_to_goal_cost > 1e-5 ? cost.to_goal_cost / avg_to_goal_cost : 0.0;
            double norm_speed = avg_speed_cost > 1e-5 ? cost.speed_cost / avg_speed_cost : 0.0;
            double norm_obstacle = avg_obstacle_cost > 1e-5 ? cost.obstacle_cost / avg_obstacle_cost : 0.0;
            
            // 计算加权代价
            weighted_goal = goal_cost_gain_ * norm_goal;
            weighted_to_goal = to_goal_cost_gain_ * norm_to_goal;
            weighted_speed = speed_cost_gain_ * norm_speed;
            weighted_obstacle = obstacle_cost_gain_ * norm_obstacle;
            
            // 计算总代价
            total_cost = weighted_goal + weighted_to_goal + weighted_speed + weighted_obstacle;    
        }

        // // 打印每条轨迹的加权代价
        // std::cout << "轨迹 " << (traj_data.idx + 1) << ": ";
        // if (cost.collision) {
        //     std::cout << "碰撞";
        // } else {
        //     std::cout<< "速度=" << traj_data.v << ", 角速度=" << traj_data.w * RAD_TO_DEG << "deg/s, "
        //     << "目标=" << weighted_goal
        //     << ", 朝向=" << weighted_to_goal
        //     << ", 障碍物=" << weighted_obstacle
        //     << ", 速度=" << weighted_speed
        //     << ", 总代价=" << total_cost;
        // }
        // std::cout << std::endl;

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
    // // 打印最佳轨迹信息
    // std::cout << "\n===== 最佳轨迹 =====" << std::endl;
    // std::cout << "轨迹ID: " << (best_traj_data.idx + 1) << std::endl;
    // std::cout << "总代价: " << best_total_cost << std::endl;
    // std::cout << "====================\n" << std::endl;
    // std::cout<<"-------------------------目标代价系数：   "<<goal_cost_gain_<<std::endl;

    // 直接提取最佳轨迹的动作
    best_trajectory = best_traj_data.path;
    double best_v = best_traj_data.v;
    double best_w = best_traj_data.w;


    if (best_traj_data.idx == -1)
    {
        std::cout<<"无效轨迹，返回安全停止指令：------------------"<<std::endl;
        return {last_steering_, -max_accel_red};
    }

    // 转换为控制指令
    double steering = 0.0;
    if (std::abs(best_v) > 0.1) {
        steering = std::atan(wheelbase_ * best_w / best_v);
    } else {
        steering = best_w * 0.5;
    }

    
    // 添加转向角变化率约束
    double max_steer_change = max_steer_rate_ * dt_;
    steering = std::clamp(steering, 
                          last_steering_ - max_steer_change,
                          last_steering_ + max_steer_change);
                              // 更新上次转向角
    last_steering_ = steering;
    
    // 确保在物理极限范围内
    steering = std::clamp(steering, -max_steer_, max_steer_);
    
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
    
    if (distance < 2.0) {
        throttle -= 0.6;
    }
    const double DEAD_ZONE_THRESHOLD_1 = 15.0 * DEG_TO_RAD; // 60度死区
    const double COMPENSATION_GAIN_1 = 1; // 补偿增益
    const double DEAD_ZONE_THRESHOLD_2 = 35.0 * DEG_TO_RAD; // 60度死区
    const double COMPENSATION_GAIN_2 = 1.4; // 补偿增益
    const double DEAD_ZONE_THRESHOLD_3 = 60.0 * DEG_TO_RAD; // 60度死区
    const double COMPENSATION_GAIN_3 = 1.5; // 补偿增益

    if (std::abs(steering) > 0 && std::abs(steering) <= DEAD_ZONE_THRESHOLD_1) {
    // 在小转向范围内增加补偿
    steering *= COMPENSATION_GAIN_1;
    }

    else if (std::abs(steering) > DEAD_ZONE_THRESHOLD_1 && std::abs(steering) <= DEAD_ZONE_THRESHOLD_2) {
    // 在小转向范围内增加补偿
    steering *= COMPENSATION_GAIN_2;
    }

    else if (std::abs(steering) > DEAD_ZONE_THRESHOLD_2 && std::abs(steering) <= DEAD_ZONE_THRESHOLD_3) {
    // 在小转向范围内增加补偿
    steering *= COMPENSATION_GAIN_3;
    }
    

    
    std::cout<<"有效轨迹，返回正常油门指令：------------------"<<std::endl;
    return {steering, throttle};
}

BikeController::TrajectoryCost BikeController::simulate_trajectory(
    double x, double y, double yaw, double v, double w,
    double person_x, double person_y,double goal_x, double goal_y,
    const std::vector<std::vector<int>>& grid_map,
    double grid_resolution,
    double min_x, double min_y,
    std::vector<std::pair<double, double>>& trajectory,double person_speed) 
{
    TrajectoryCost cost;
    cost.goal_cost = 0.0;
    cost.to_goal_cost = 0.0;
    cost.speed_cost = 0.0;
    cost.obstacle_cost = 0.0; // 初始化为0
    cost.collision = false;
    
    double min_obs_dist = std::numeric_limits<double>::max();
    trajectory.clear();

    double dx = person_x - x;
    double dy = person_y - y;
    double predict_position = std::hypot(dx, dy);
    double predict_time_limit = predict_position / v;

    if(3.5 > predict_time_limit)
    {
        predict_time_ = predict_time_limit;
    }
    else
    {
        predict_time_ = 3.5;
    }
    // 模拟轨迹
    for (double t = 0.0; t <= predict_time_; t += dt_) {
        // 更新状态
        yaw += w * dt_;
        yaw = std::fmod(yaw, 2.0 * M_PI);
        if (yaw > M_PI) {
            yaw -= 2.0 * M_PI;
        } else if (yaw < -M_PI) {
            yaw += 2.0 * M_PI;
        }
        x += v * std::cos(yaw) * dt_;
        y += v * std::sin(yaw) * dt_;
        
        // 存储轨迹点
        trajectory.push_back({x, y});
        
        // 检查障碍物碰撞
        double obs_dist = calculateObstacleDistance(x, y, yaw, grid_map, grid_resolution, min_x, min_y, person_x, person_y);
        
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
        
        // // 计算朝向目标的代价
        // // 使用初始位置计算角度差
        // if (t >= predict_time_ - dt_) {
        //     double target_angle = std::atan2(dy, dx);
        //     double angle_diff = std::abs(std::fmod(target_angle - yaw + M_PI, 2*M_PI) - M_PI);
        //     cost.to_goal_cost = angle_diff;
        // }
        

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
    double min_x, double min_y,
    double person_x, double person_y)
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

                    // 关键修改：忽略行人附近的障碍物
                    double person_dist = std::hypot(person_x - obs_x, person_y - obs_y);
                    if (person_dist < 0.7) {
                        continue; // 跳过行人附近的障碍物
                    }

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
    steering_deg = std::clamp(steering_deg, -90.0, 90.0);
    
    // 转换为协议单位（0.1度）
    motor_cmd.rotor_steering = static_cast<int>(steering_deg * 10.0);
    
    // 限制范围（-450到450）
    motor_cmd.rotor_steering = std::clamp(motor_cmd.rotor_steering, -900, 900);
    
    // 2. 轮毂电机控制（油门）
    if (cmd.throttle > 0) {
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
    if (cmd.throttle <= 0) {
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
