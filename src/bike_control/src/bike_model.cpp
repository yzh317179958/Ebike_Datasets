#include "bike_control/bike_model.h"
#include <iostream>
#include <cmath>

const double PI = 3.14159265358979323846;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

// 参数构造函数
BikeParameters::BikeParameters(double wb, double msa, double max_acc, double max_dec)
    : wheelbase(wb), max_steering_angle(msa * DEG_TO_RAD), // 弧度
      max_acceleration(max_acc), max_deceleration(max_dec) {}

// 状态构造函数
BikeState::BikeState(double x, double y, double yaw_, double speed_, double sa)
    : rear_x(x), rear_y(y), yaw(yaw_), speed(speed_), steering_angle(sa), 
      angular_velocity(0.0), front_x(0.0), front_y(0.0), front_yaw(0.0) {}

BicycleModel::BicycleModel(const BikeParameters& p, const BikeState& s)
    : params(p), state(s), last_time(0.0), last_steering_angle(s.steering_angle) {}

// 角度标准化函数（弧度）
double BicycleModel::normalizeAngle(double angle) const {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

void BicycleModel::update(double dt, double bike_speed, 
                         double bike_front_yaw, double bike_rear_yaw,
                         double odom_x, double odom_y) {

    // 更新状态
    if(bike_speed <= 0)
    {
        state.speed = 0;// 速度(m/s)
    }
    else
    {
        state.speed = bike_speed;// 速度(m/s)
    }
    
    state.yaw = normalizeAngle(bike_rear_yaw); // 后轮方向即车身方向
    // state.steering_angle = normalizeAngle(bike_front_yaw * DEG_TO_RAD);// 前轮转角(rad)
    state.steering_angle = normalizeAngle(bike_front_yaw);// 前轮转角(rad)

    // 计算角度差（考虑角度环绕）
    double angle_diff = state.steering_angle - last_steering_angle;
    
    // 处理角度环绕（超过180度的情况）
    if (angle_diff > PI) {
        angle_diff -= 2.0 * PI;
    } else if (angle_diff < -PI) {
        angle_diff += 2.0 * PI;
    }
    
    state.angular_velocity = angle_diff / dt;

    last_steering_angle = state.steering_angle;
    
    // 计算后轮中心位置
    // 里程计位于后轮中心前方0.97米处（沿车身方向）
    state.rear_x = odom_x - 0.97 * std::cos(state.yaw);
    state.rear_y = odom_y - 0.97 * std::sin(state.yaw);
    
    // 计算前轮方向
    state.front_yaw = normalizeAngle(state.yaw + state.steering_angle);
    // 里程计位于前轮中心后方0.03米处（沿车身方向）计算前轮中心位置
    state.front_x = state.rear_x + 1 * std::cos(state.front_yaw);
    state.front_y = state.rear_y + 1 * std::sin(state.front_yaw);
}

const BikeState& BicycleModel::getState() const {
    return state;
}