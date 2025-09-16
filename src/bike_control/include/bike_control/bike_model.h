#ifndef BICYCLE_MODEL_H
#define BICYCLE_MODEL_H

#include <cmath>
#include <utility>

struct BikeParameters {
    double wheelbase;          // 轴距
    double max_steering_angle; // 最大转向角
    double max_acceleration;   // 最大加速度
    double max_deceleration;   // 最大减速度
    
    BikeParameters(double wb = 1.0, double msa = 50.0, 
                  double max_acc = 3.0, double max_dec = 5.0);
};

struct BikeState {
    double rear_x;            // 后轮中心x坐标
    double rear_y;            // 后轮中心y坐标
    double yaw;               // 车身偏航角（后轮方向）
    double speed;             // 车速
    double steering_angle;    // 前轮转向角（相对于车身）
    double angular_velocity;  // 车身角速度（绕垂直轴）
    
    // 前轮位置
    double front_x;
    double front_y;
    
    // 前轮偏航角
    double front_yaw;
    
    BikeState(double x = 0.0, double y = 0.0, double yaw_ = 0.0, 
             double speed_ = 0.0, double sa = 0.0);
};

class BicycleModel {
private:
    BikeParameters params;  // 机械参数
    BikeState state;        // 当前状态
    double last_time;       // 上次更新时间戳
    double last_steering_angle; // 上次转向角（用于计算角速度）
    
    // 角度标准化到[-π, π]
    double normalizeAngle(double angle) const;
    
public:
    BicycleModel(const BikeParameters& p, const BikeState& s);
    
    void update(double dt, double bike_speed, 
                double bike_front_yaw, double bike_rear_yaw,
                double odom_x, double odom_y);
    
    // 获取当前状态
    const BikeState& getState() const;
};

#endif