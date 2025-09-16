#ifndef BIKE_CONTROLLER_H
#define BIKE_CONTROLLER_H

#include <vector>
#include <deque>
#include <utility>
#include <cmath>
#include <algorithm>
#include <limits>

class BikeController {
public:
    struct Command {
        double steering;
        double throttle;
    };
    
    struct TrajectoryCost {
        double goal_cost;       // 目标距离代价
        double to_goal_cost;   // 朝向目标代价
        double speed_cost;      // 速度代价
        double obstacle_cost;  // 障碍物代价
        bool collision;        // 是否碰撞
    };
    
    struct TrajectoryData {
        std::vector<std::pair<double, double>> path;
        TrajectoryCost cost;
        double v;  // 轨迹对应的速度
        double w;  // 轨迹对应的角速度
        int idx;   // 轨迹索引
    };
    
    struct Trajectory {
        std::vector<std::pair<double, double>> path;
        struct {
            double total_cost;
            double goal_cost;
            double to_goal_cost;
            double speed_cost;
            double obstacle_cost;
        } cost;
    };
    
    BikeController();
    
    void init(double wheelbase, double max_speed, double min_speed, 
              double max_steer, double max_accel_add, double max_accel_red,
              double max_dyaw_rate, double predict_time, double goal_cost_gain,
              double speed_cost_gain, double obstacle_cost_gain, 
              double to_goal_cost_gain, double max_steer_rate,
              double safety_margin, double obstacle_influence_dist);
    

    Command calculate(
        double bike_x, double bike_y, double bike_yaw, double bike_v,
        double person_x, double person_y,
        const std::deque<std::pair<double, double>>& global_path,
        const std::vector<std::vector<int>>& grid_map,
        double grid_resolution,
        double min_x, double min_y,
        std::vector<Trajectory>& all_trajectories,
        std::vector<std::pair<double, double>>& best_trajectory
    );
    
    // 添加电机控制指令结构体
    struct MotorCommand {
        int rotor_steering;   // 转把电机转向指令 (单位: 0.1度)
        int hub_throttle;     // 轮毂电机油门指令 (0-1000)
        int front_brake;      // 前刹车力度 (0-1000)
        int rear_brake;       // 后刹车力度 (0-1000)
    };         


    std::pair<double, double> getLastGoal() const { 
        return {last_goal_x_, last_goal_y_}; 
    }
                    
    // 添加转换函数声明
    MotorCommand convertToMotorCommand(
    const Command& cmd, double current_speed,double dt ,int speed_limit);
        
private:
    // DWA算法参数
    double wheelbase_;          // 轴距
    double max_speed_;          // 最大速度
    double min_speed_;          // 最小速度
    double max_steer_;          // 最大转向角 (弧度)
    double max_accel_add;       // 最大加速度
    double max_accel_red;       // 最大减速度
    double dt_;                 // 时间步长
    double max_dyaw_rate_;      // 最大偏航率
    double predict_time_;       // 预测时间长度
    double goal_cost_gain_;     // 目标代价增益
    double speed_cost_gain_;    // 速度代价增益
    double obstacle_cost_gain_; // 障碍物代价增益
    double to_goal_cost_gain_;  // 朝向目标代价增益
    double last_steering_;      // 上一时刻转向角
    double max_steer_rate_; // 最大转向变化率 (20度/秒)
    double safety_margin_;      // 安全余量
    double OBSTACLE_INFLUENCE_DIST; // 障碍物影响半径
    double last_goal_x_ = 0.0;
    double last_goal_y_ = 0.0;
    double prev_person_x = 0.0;
    double prev_person_y = 0.0;
    // 辅助函数声明
    double calculateObstacleDistance(
        double x, double y, double yaw,
        const std::vector<std::vector<int>>& grid_map,
        double grid_resolution,
        double min_x, double min_y,double person_x, double person_y);

    // 轨迹模拟函数声明
    TrajectoryCost simulate_trajectory(
        double x, double y, double yaw, double v, double w,
        double person_x, double person_y,double goal_x, double goal_y,
        const std::vector<std::vector<int>>& grid_map,
        double grid_resolution,
        double min_x, double min_y,
        std::vector<std::pair<double, double>>& trajectory,double person_speed
    );
    
    // 新增函数声明
    Command calculateWithGoal(
        double bike_x, double bike_y, double bike_yaw, double bike_v,
        double person_x, double person_y, double goal_x, double goal_y,
        const std::vector<std::vector<int>>& grid_map,
        double grid_resolution,
        double min_x, double min_y,
        std::vector<Trajectory>& all_trajectories,
        std::vector<std::pair<double, double>>& best_trajectory
    );
    

};

#endif // BIKE_CONTROLLER_H