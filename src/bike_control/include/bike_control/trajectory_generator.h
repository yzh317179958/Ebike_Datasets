#pragma once
#include <deque>
#include <utility>

class TrajectoryGenerator {
public:
    TrajectoryGenerator(); 
    void update(double dt);
    void update_Foxglove(double p_pos_x,double p_pos_y, double dt);
    std::pair<double, double> get_position() const;
    const std::deque<std::pair<double, double>>& get_history() const;
    double get_velocity() const;
    double x_ = 0.0, y_ = 0.0;
private:
    double t_ = 0.0;
    double last_x_ = 0.0, last_y_ = 0.0;
    double d_x_last = 0.0, d_y_last = 0.0; 
    double d_x = 10,d_y = 5;
    double current_v_ = 0.0;
    bool last_point_received_ = false;
    std::deque<std::pair<double, double>> history_;
};