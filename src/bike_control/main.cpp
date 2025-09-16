#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <deque>
#include <memory>
#include <random>
#include <iostream>
#include <filesystem> 
#include <fstream> 
#include <chrono>   
#include <iomanip> 
#include "bike_control/bike_model.h"
#include "bike_control/trajectory_generator.h"
#include "bike_control/bike_controller.h"
#include "bike_control/bike_visualizer.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>  
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "integrated_navigation/msg/integrated_data.hpp"
#include "bike_control/motor.h"
#include <serial/serial.h>

namespace fs = std::filesystem;
class BikeNode : public rclcpp::Node {
public:
    BikeNode() : Node("ebike_control_node"),
                model_(
                    BikeParameters(1.0, 45.0, 1.0, 45.0), // 轴距1.0m, 最大转向角45°
                    BikeState(0.0, 0.0, 0, 0, 0)            // 初始状态
                ),
                trajectory_(),
                controller_(),
                visualizer_(std::make_unique<BikeVisualizer>(
                    this, 
                    "/bike_control/visualization",
                    1.0,    // bike_length
                    0.5,    // bike_width
                    0.5,    // front_length
                    0.5     // rear_length
                )),
                motor_(serial_port_)  
    {
        // 声明日志路径参数，默认为当前目录下的logs文件夹
        this->declare_parameter<std::string>("log_path", "logs");

        // 声明速度参数默认值
        this->declare_parameter<int>("hub_throttle_value", 8); // 电机固定速度值，可选
        this->declare_parameter<int>("speed_limit", 65); // 电机自由速度值上限，
        // 获取参数值
        hub_throttle_value_ = this->get_parameter("hub_throttle_value").as_int();
        speed_limit = this->get_parameter("speed_limit").as_int();

        // 声明控制器参数
        this->declare_parameter<double>("wheelbase_", 8.0);
        this->declare_parameter<double>("max_speed_", 0.7);
        this->declare_parameter<double>("min_speed_", 0.0);
        this->declare_parameter<double>("max_steer_", 45.0);
        this->declare_parameter<double>("max_accel_add", 1.0);
        this->declare_parameter<double>("max_accel_red", 1.0);
        this->declare_parameter<double>("max_dyaw_rate_", 45.0);
        this->declare_parameter<double>("predict_time_", 3.0);
        this->declare_parameter<double>("goal_cost_gain_", 0.3);
        this->declare_parameter<double>("speed_cost_gain_", 0.1);
        this->declare_parameter<double>("obstacle_cost_gain_", 0.2);
        this->declare_parameter<double>("to_goal_cost_gain_", 0.4);
        this->declare_parameter<double>("max_steer_rate_", 30.0);
        this->declare_parameter<double>("safety_margin_", 0.1);
        this->declare_parameter<double>("OBSTACLE_INFLUENCE_DIST", 0.1);

        double wheelbase=this->get_parameter("wheelbase_").as_double();          // 轴距
        double max_speed_=this->get_parameter("max_speed_").as_double();         // 最大速度
        double min_speed_=this->get_parameter("min_speed_").as_double();          // 最小速度
        double max_steer_=this->get_parameter("max_steer_").as_double();          // 最大转向角 (弧度)
        double max_accel_add=this->get_parameter("max_accel_add").as_double();       // 最大加速度
        double max_accel_red=this->get_parameter("max_accel_red").as_double();       // 最大减速度
        double max_dyaw_rate_=this->get_parameter("max_dyaw_rate_").as_double();      // 最大偏航率
        double predict_time_=this->get_parameter("predict_time_").as_double();       // 预测时间长度
        double goal_cost_gain_=this->get_parameter("goal_cost_gain_").as_double();    // 目标代价增益
        double speed_cost_gain_=this->get_parameter("speed_cost_gain_").as_double();   // 速度代价增益
        double obstacle_cost_gain_=this->get_parameter("obstacle_cost_gain_").as_double();// 障碍物代价增益
        double to_goal_cost_gain_=this->get_parameter("to_goal_cost_gain_").as_double();// 朝向目标代价增益
        double max_steer_rate_=this->get_parameter("max_steer_rate_").as_double();// 最大转向变化率 (20度/秒)
        double safety_margin_=this->get_parameter("safety_margin_").as_double();     // 安全余量
        double obstacle_influence_dist=this->get_parameter("OBSTACLE_INFLUENCE_DIST").as_double();// 障碍物影响半径

        controller_.init(wheelbase, max_speed_, min_speed_,
                        max_steer_, max_accel_add, max_accel_red,
                        max_dyaw_rate_, predict_time_, goal_cost_gain_,
                        speed_cost_gain_, obstacle_cost_gain_, to_goal_cost_gain_, 
                        max_steer_rate_, safety_margin_, obstacle_influence_dist);
        // 声明参数
        this->declare_parameter<bool>("draw_fine_grid", false); // false关闭精细网格
        
        // 获取参数值
        bool draw_fine_grid = this->get_parameter("draw_fine_grid").as_bool();
        
        // 初始化日志文件
        initLogFile();
        
        // 创建集成导航信息订阅者
        integrated_data_sub_ = this->create_subscription<integrated_navigation::msg::IntegratedData>(
            "/integrated_navigation_data", 10, 
            std::bind(&BikeNode::integratedDataCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "节点已启动");
        
        // 初始化串口和电机
        initSerialAndMotor();
        
        // 记录初始化完成
        logToFile("INFO", "BikeNode initialized");
    }

    ~BikeNode() {
        // 节点销毁时关闭日志文件
        if (log_file_.is_open()) {
            logToFile("INFO", "BikeNode shutting down");
            log_file_.close();
        }
    }

    void closeSerialPort() {
        if (serial_port_.isOpen()) {
            try {
                serial_port_.close();
                RCLCPP_INFO(this->get_logger(), "串口已关闭");
                logToFile("INFO", "Serial port closed");
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "关闭串口失败: %s", e.what());
                logToFile("ERROR", "Failed to close serial port: " + std::string(e.what()));
            }
        }
    }

private:
    void initLogFile() {
        // 获取日志路径参数
        std::string log_path = this->get_parameter("log_path").as_string();
        fs::path full_log_path = fs::absolute(log_path);
        
        // 确保路径存在
        try {
            if (!fs::exists(full_log_path)) {
                fs::create_directories(full_log_path);
            }
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "无法创建日志目录: %s", e.what());
            full_log_path = fs::current_path();
        }

        // 获取当前时间作为日志文件名
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "bike_control_log_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".log";
        
        // 使用完整路径
        log_filename_ = (full_log_path / ss.str()).string();
        
        // 打开日志文件
        log_file_.open(log_filename_, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "日志文件已创建: %s", log_filename_.c_str());
        }
    }
    
    void logToFile(const std::string& level, const std::string& message) {
        if (log_file_.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            
            log_file_ << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "] "
                     << "[" << level << "] "
                     << message << std::endl;
        }
    }

    void initSerialAndMotor() {
        //constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
        constexpr const char* SERIAL_PORT = "/dev/ttyS1";
        constexpr uint32_t BAUD_RATE = 57600;
        constexpr int TIMEOUT_MS = 1000;
        constexpr int RETRY_COUNT = 3;
        
        bool port_opened = false;
        hub_throttle_value_ = this->get_parameter("hub_throttle_value").as_int();
        speed_limit = this->get_parameter("speed_limit").as_int();

        for(int i = 0; i < RETRY_COUNT; i++) {
            try {
                serial_port_.setPort(SERIAL_PORT);
                serial_port_.setBaudrate(BAUD_RATE);
                serial_port_.setTimeout(serial::Timeout::max(), TIMEOUT_MS, 0, TIMEOUT_MS, 0);
                serial_port_.open();
                
                if(serial_port_.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "串口已成功打开: %s", SERIAL_PORT);
                    logToFile("INFO", "Serial port opened successfully: " + std::string(SERIAL_PORT));
                    port_opened = true;
                    serial_port_.flushInput();
                    serial_port_.flushOutput();
                    break;
                }
            } catch (const serial::IOException& e) {
                std::string error_msg = "Attempt " + std::to_string(i+1) + " failed to open serial port: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
                logToFile("ERROR", error_msg);
                if(i < RETRY_COUNT - 1) {
                    usleep(500000);
                }
            }
        }
        
        if(!port_opened) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口");
            logToFile("ERROR", "Failed to open serial port after " + std::to_string(RETRY_COUNT) + " attempts");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "电机控制器已初始化");
            logToFile("INFO", "Motor controller initialized");
        }
    }

    // 全局地图函数
    void integratedDataCallback(const integrated_navigation::msg::IntegratedData::SharedPtr msg) {
        // 记录接收到的消息
        
        latest_integrated_data_ = *msg;
        has_integrated_data_ = true;

        const auto& odom_header = msg->odometry.header;
        const auto& user_header = msg->user_position.header;
        const auto& grid = msg->occupancy_grid;
        
        rclcpp::Time odom_time(odom_header.stamp.sec, odom_header.stamp.nanosec);
        rclcpp::Time user_time(user_header.stamp.sec, user_header.stamp.nanosec);
        
        const auto& odom = msg->odometry.pose.pose;
        const auto& person_pos = msg->user_position.pose.position;
        
        // 提取地图信息
        grid_resolution_ = grid.info.resolution;
        grid_min_x_ = grid.info.origin.position.x;
        grid_min_y_ = grid.info.origin.position.y;
        
        // 将一维地图数据转换为二维网格
        grid_map_.clear();
        grid_map_.resize(grid.info.height);
        for (int i = 0; i < grid.info.height; i++) {
            grid_map_[i].resize(grid.info.width);
            for (int j = 0; j < grid.info.width; j++) {
                int index = i * grid.info.width + j;
                grid_map_[i][j] = grid.data[index];
            }
        }

        // 设置可视化器的地图
        visualizer_->setGridMap(grid_map_, grid.info.resolution, 
                               grid.info.origin.position.x, 
                               grid.info.origin.position.y);

        double current_bike_x = odom.position.x;
        double current_bike_y = odom.position.y;
        double current_person_x = person_pos.x;
        double current_person_y = person_pos.y;

        tf2::Quaternion q(
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double current_bike_yaw = yaw;
        
        processData(current_bike_x, current_bike_y, current_bike_yaw,
                    current_person_x, current_person_y,
                    odom_time, user_time);
        
        executeControlCycle();
    }

    // // 局部地图
    // void integratedDataCallback(const integrated_navigation::msg::IntegratedData::SharedPtr msg) {
    //     // 记录接收到的消息
    //     latest_integrated_data_ = *msg;
    //     has_integrated_data_ = true;

    //     const auto& odom_header = msg->odometry.header;
    //     const auto& user_header = msg->user_position.header;
    //     const auto& grid = msg->occupancy_grid;
        
    //     rclcpp::Time odom_time(odom_header.stamp.sec, odom_header.stamp.nanosec);
    //     rclcpp::Time user_time(user_header.stamp.sec, user_header.stamp.nanosec);
        
    //     const auto& odom = msg->odometry.pose.pose;
    //     const auto& person_pos = msg->user_position.pose.position;
        
    //     // 提取地图信息
    //     grid_resolution_ = grid.info.resolution;
    //     double global_min_x = grid.info.origin.position.x;
    //     double global_min_y = grid.info.origin.position.y;
        
    //     // 直接提取车辆当前位置
    //     double current_bike_x = odom.position.x;
    //     double current_bike_y = odom.position.y;
    //     double current_person_x = person_pos.x;
    //     double current_person_y = person_pos.y;

    //     // 计算车辆朝向
    //     tf2::Quaternion q(
    //         odom.orientation.x,
    //         odom.orientation.y,
    //         odom.orientation.z,
    //         odom.orientation.w);
    //     tf2::Matrix3x3 m(q);
    //     double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);
    //     double current_bike_yaw = yaw;
        
    //     // 处理数据并更新状态
    //     processData(current_bike_x, current_bike_y, current_bike_yaw,
    //                 current_person_x, current_person_y,
    //                 odom_time, user_time);
        
    //     // 提取局部地图
    //     const double LOCAL_MAP_RADIUS = 10.0; // 10米半径的局部地图
    //     // grid_resolution_ = grid.info.resolution;
    //     // grid_min_x_ = grid.info.origin.position.x;
    //     // grid_min_y_ = grid.info.origin.position.y;
    //     RCLCPP_INFO(this->get_logger(), "地图原点x: %f",grid_min_x_);
    //     RCLCPP_INFO(this->get_logger(), "地图原点y: %f",grid_min_y_);
    //     // 局部地图
    //     grid_map_ = extractLocalMapFrom1D(
    //         current_bike_x, current_bike_y, current_bike_yaw,
    //         grid,  
    //         LOCAL_MAP_RADIUS,
    //         grid_min_x_, grid_min_y_  // 更新为局部地图的原点
    //     );
        
    //     // 执行控制循环
    //     executeControlCycle();
    // }

    // // 提取函数，直接返回二维网格
    // std::vector<std::vector<int>> extractLocalMapFrom1D(
    //     double bike_x, double bike_y, double bike_yaw,
    //     const nav_msgs::msg::OccupancyGrid& grid,
    //     double radius,
    //     double& out_min_x, double& out_min_y)  // 输出参数：局部地图原点
    // {
    //     double resolution = grid.info.resolution;
    //     double global_min_x = grid.info.origin.position.x;
    //     double global_min_y = grid.info.origin.position.y;
    //     int global_width = grid.info.width;
    //     int global_height = grid.info.height;
        
    //     // 计算车辆前方中心点（70%的半径在前方）
    //     double forward_distance = radius * 0.7;
    //     double center_x = bike_x + forward_distance * std::cos(bike_yaw);
    //     double center_y = bike_y + forward_distance * std::sin(bike_yaw);
        
    //     // 计算局部地图边界
    //     double local_min_x = center_x - radius;
    //     double local_min_y = center_y - radius;
    //     double local_max_x = center_x + radius;
    //     double local_max_y = center_y + radius;
        
    //     // 转换为网格坐标
    //     int grid_min_x = static_cast<int>((local_min_x - global_min_x) / resolution);
    //     int grid_min_y = static_cast<int>((local_min_y - global_min_y) / resolution);
    //     int grid_max_x = static_cast<int>((local_max_x - global_min_x) / resolution);
    //     int grid_max_y = static_cast<int>((local_max_y - global_min_y) / resolution);
        
    //     // 确保在全局地图范围内
    //     grid_min_x = std::max(0, grid_min_x);
    //     grid_min_y = std::max(0, grid_min_y);
    //     grid_max_x = std::min(global_width - 1, grid_max_x);
    //     grid_max_y = std::min(global_height - 1, grid_max_y);
        
    //     // 计算局部地图尺寸
    //     int local_width = grid_max_x - grid_min_x + 1;
    //     int local_height = grid_max_y - grid_min_y + 1;
        
    //     // 创建局部地图
    //     std::vector<std::vector<int>> local_map;
    //     local_map.resize(local_height);
    //     for (int i = 0; i < local_height; i++) {
    //         local_map[i].resize(local_width);
    //         for (int j = 0; j < local_width; j++) {
    //             int global_x = grid_min_x + j;
    //             int global_y = grid_min_y + i;
    //             int index = global_y * global_width + global_x;
    //             local_map[i][j] = grid.data[index];
    //         }
    //     }
        
    //     // 计算局部地图的世界坐标原点
    //     out_min_x = global_min_x + grid_min_x * resolution;
    //     out_min_y = global_min_y + grid_min_y * resolution;
        
    //     return local_map;
    // }


    void processData(double bike_x, double bike_y, double bike_yaw,
                     double person_x, double person_y,
                     rclcpp::Time bike_stamp, rclcpp::Time person_stamp) {
        bike_x_ = bike_x;
        bike_y_ = bike_y;
        bike_yaw_ = bike_yaw;
        person_x_ = person_x;
        person_y_ = person_y;
        bike_pos_stamp_ = bike_stamp;
        person_pos_stamp_ = person_stamp;

        if (last_bike_time_.nanoseconds() == 0) {
            last_bike_time_ = bike_pos_stamp_;
            last_person_time_ = person_pos_stamp_;
            last_bike_pos_x_ = bike_x_;
            last_bike_pos_y_ = bike_y_;
            last_bike_yaw_ = bike_yaw_;
            
            // 记录初始状态
            std::stringstream ss;
            ss << "Initial state - Bike: (" << bike_x_ << ", " << bike_y_ << ", " << bike_yaw_ 
               << "), Person: (" << person_x_ << ", " << person_y_ << ")";
            logToFile("INFO", ss.str());
            
            return;
        }

        bike_time_history_.push_back(bike_stamp);
        if (bike_time_history_.size() > 100) {
            bike_time_history_.pop_front();
        }
    }

    void executeControlCycle() {
        // 添加循环开始分隔符
        logToFile("INFO", "==================================================");
        // 记录循环起始时间
        auto cycle_start = std::chrono::high_resolution_clock::now();
        if (!has_integrated_data_ || last_bike_time_.nanoseconds() == 0) {
            logToFile("WARN", "Skipping control cycle - no valid data");
            return;
        }
        
        // person_x_=1.2;
        // person_y_=-3;
        const auto& hub_state = motor_.getHubState();// 轮毂电机状态
        const auto& rotor_state = motor_.getRotorState();// 转把电机状态
        // const auto& left_aux_state = motor_.getLeftAuxState();// 左辅助轮状态
        // const auto& right_aux_state = motor_.getRightAuxState();// 右辅助轮状态

        auto state = model_.getState();

        bike_step = (bike_pos_stamp_ - last_bike_time_).seconds();
        if(bike_pos_stamp_ == last_bike_time_ )
        {
            bike_step=0.1;
        }

        double bike_dx = bike_x_ - last_bike_pos_x_;// 里程计位置-上一时刻里里程计位置
        double bike_dy = bike_y_ - last_bike_pos_y_;// 里程计位置-上一时刻里里程计位置
        double distance_bike = std::hypot(bike_dx, bike_dy);// 上一时刻到当前时刻的距离

        double person_step = (person_pos_stamp_ - last_person_time_).seconds();//

        // 记录行人轨迹
        trajectory_.update_Foxglove(person_x_, person_y_, person_step);

        last_bike_time_ = bike_pos_stamp_;//记录上一时刻时间戳
        last_person_time_ = person_pos_stamp_;
        last_bike_pos_x_ = bike_x_;// 上一时刻自行车位置
        last_bike_pos_y_ = bike_y_;// 上一时刻自行车位置
        last_bike_yaw_ = bike_f_yaw_ + state.yaw;// 上一时刻前轮转角计算

        std::vector<BikeController::Trajectory> all_trajectories;
        std::vector<std::pair<double, double>> best_trajectory;
        // std::cout<<"目标点x: "<<bike_x_ <<std::endl;
        // std::cout<<"目标点y: "<<bike_y_ <<std::endl;
        
        // std::cout<<"行人x位置: "<< person_x_ <<std::endl;
        // std::cout<<"行人y位置: "<< person_y_ <<std::endl;
        // std::cout<<"自行车x位置: "<< state.front_x <<std::endl;
        // std::cout<<"自行车y位置: "<< state.front_y <<std::endl;
        // std::cout<<"自行车前轮角: "<< state.front_yaw * 180 / M_PI <<std::endl;
        // std::cout<< "速度输出:   "<<state.speed<<std::endl;
        // 计算自行车与行人之间的距离和角度差
        double dx = person_x_ - state.front_x;
        double dy = person_y_ - state.front_y;
        double distance = std::hypot(dx, dy);
        // bike_f_yaw_ = rotor_state.position / 10;
        
        // 记录参数 - 相关状态
        logToFile("DATA", "行人位置: x=" + std::to_string(person_x_) + "(m) , y=" + std::to_string(person_y_) + "(m)");
        logToFile("DATA", "车辆位置: x=" + std::to_string(state.front_x) + "(m) , y=" + std::to_string( state.front_y)+"(m)");
        logToFile("DATA", "车辆后轮角度(deg): " + std::to_string(state.yaw * (180.0 / M_PI))+ "(deg)");
        logToFile("DATA", "车辆前轮角度(deg): " + std::to_string(state.front_yaw* (180.0 / M_PI))+ "(deg)");
        logToFile("DATA", "自行车与行人距离: " + std::to_string(distance) + "(m)");
        logToFile("DATA", "自行车当前速度: " + std::to_string(state.speed) + "(m/s)");
        // logToFile("DATA", "角度差(deg): " + std::to_string(angle_diff * (180.0 / M_PI)) + "(deg)");

        // 计算控制指令 (DWA算法)
        auto cmd = controller_.calculate(
            state.front_x, state.front_y, state.front_yaw, state.speed,//yaw (rad)
            person_x_, person_y_, 
            trajectory_.get_history(),  // 使用行人轨迹作为全局路径
            grid_map_,     // 网格地图
            grid_resolution_, // 网格分辨率
            grid_min_x_, grid_min_y_,  // 地图原点
            all_trajectories, // 所有模拟轨迹
            best_trajectory   // 最佳轨迹
        );

        bike_speed = cmd.throttle;
        bike_f_yaw_ = cmd.steering;
        // bike_f_yaw_ = rotor_state.position / 10;
        model_.update(bike_step, bike_speed, bike_f_yaw_, bike_yaw_ ,bike_x_, bike_y_);
        // 记录控制端指令
        logToFile("DATA", "油门区间控制: " + std::to_string(cmd.throttle) + 
                          ", 转向区间控制: " + std::to_string(cmd.steering));
            
        // RCLCPP_INFO(this->get_logger(), "控制指令转向角: %f",cmd.steering);

        try {
            auto motor_cmd = controller_.convertToMotorCommand(cmd, state.speed, bike_step, speed_limit);
            
            // 记录实际电机指令
            logToFile("DATA", "油门实际控制: " + std::to_string(motor_cmd.hub_throttle) + 
                                ", 转向实际控制: " + std::to_string(motor_cmd.rotor_steering/10));

            // 对电机发送控制指令
            // 转把电机（转向控制）
            motor_.controlRotorMotor(22, motor_cmd.rotor_steering);
            // RCLCPP_INFO(this->get_logger(), "实际转向角度: %d",motor_cmd.rotor_steering);
            // 轮毂电机（油门控制）
            if (motor_cmd.hub_throttle <=0) {
                motor_.controlHubMotor(1, 0, 1);
                motor_.controlFrontBrakeMotor(1000, 1000);
                motor_.controlRearBrakeMotor(1000, 1000);
                first_up = true;
            } 
            else {
                if(first_up)
                {
                    motor_.controlRearBrakeMotor(1000, 600);
                    motor_.controlFrontBrakeMotor(1000, 600);
                    auto time_motor_start = std::chrono::high_resolution_clock::now();
                    auto time_motor = std::chrono::duration_cast<std::chrono::milliseconds>(time_motor_start - cycle_start);
                    if (time_motor.count()>0)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds((90-time_motor.count()))); // 延时(90-cycle_duration.count())ms
                    }
                    // auto time_motor_end = std::chrono::high_resolution_clock::now();
                    motor_.controlHubMotor(1, motor_cmd.hub_throttle, 0);
                    first_up = false;
                }
                else
                {
                    motor_.controlHubMotor(1, motor_cmd.hub_throttle, 0);
                    motor_.controlRearBrakeMotor(1000, 600);
                    motor_.controlFrontBrakeMotor(1000, 600);
                }
            }
            // motor_.controlHubMotor(1, hub_throttle_value_, 0);// 启动文件控制
            motor_.processReceivedFrame();

            // // 轮毂电机状态
            // std::cout << "轮毂电机: "
            //         << "速度=" << hub_state.speed / 10.0f << ", "
            //         << "错误码=" << static_cast<int>(hub_state.error_code)
            //         << std::endl;
            
            std::stringstream motor_ss;
            motor_ss << "电机状态 - "
                        << "转把电机位置: " << rotor_state.position / 10 << "(deg) , "
                        << "轮毂电机速度: " << (hub_state.speed / 10.0f)/3.6 << "(m/s)";
            logToFile("DEBUG", motor_ss.str());

        } 
        catch (const std::exception& e) {
            std::string error_msg = "Motor control failed: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
            logToFile("ERROR", error_msg);
        }

        bike_history_.push_back(std::make_pair(bike_x_, bike_y_));
        if(bike_history_.size() > 100) {
            bike_history_.pop_front();
        }

        // 更新自行车轨迹历史
        bike_history_front.push_back(std::make_pair(state.front_x, state.front_y));
        if(bike_history_front.size() > 100) {
            bike_history_front.pop_front();
        }
        bike_history_rear.push_back(std::make_pair(state.rear_x, state.rear_y));
        if(bike_history_rear.size() > 100) {
            bike_history_rear.pop_front();
        }

        // 获取目标点坐标
        auto goal = controller_.getLastGoal();
        double goal_x = goal.first;
        double goal_y = goal.second;
        
        // 更新可视化
        visualizer_->update(
            state.front_x, state.front_y, state.yaw,
            state.rear_x, state.rear_y, state.yaw,
            person_x_, person_y_,  // 使用更新后的行人位置
            cmd.steering, cmd.throttle,
            trajectory_.get_history(),
            bike_history_front,
            bike_history_rear,
            state.speed,
            all_trajectories,    // 所有模拟轨迹
            best_trajectory,
            goal_x,goal_y);    // 最佳轨迹

            
        // 记录控制周期结束时间
        auto cycle_end = std::chrono::high_resolution_clock::now();
        
        // 计算执行时间（毫秒）
        auto cycle_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
        
        // 记录执行时间
        std::string time_msg = "控制周期执行时间: " + std::to_string(cycle_duration.count()) + " ms";
        logToFile("PERF", time_msg);
        // RCLCPP_INFO(this->get_logger(), "程序运行时间(ms): %ld", cycle_duration.count());

        // // 添加循环结束分隔符
        // logToFile("INFO", "==================================================");
    }

    // 成员变量
    BicycleModel model_;
    TrajectoryGenerator trajectory_;
    BikeController controller_;
    std::deque<std::pair<double, double>> bike_history_front;
    std::deque<std::pair<double, double>> bike_history_rear;
    std::unique_ptr<BikeVisualizer> visualizer_;

    serial::Serial serial_port_;
    Motor motor_;
    
    std::deque<std::pair<double, double>> bike_history_;
    std::deque<rclcpp::Time> bike_time_history_;
    rclcpp::Subscription<integrated_navigation::msg::IntegratedData>::SharedPtr integrated_data_sub_;
    
    // 日志相关
    std::ofstream log_file_;
    std::string log_filename_;
    
    integrated_navigation::msg::IntegratedData latest_integrated_data_;
    bool has_integrated_data_ = false;

    double bike_x_ = 0.0;
    double bike_y_ = 0.0;
    double bike_yaw_ = 0.0;
    double person_x_ = 0.0;
    double person_y_ = 0.0;
    double bike_speed = 0;

    rclcpp::Time bike_pos_stamp_;
    rclcpp::Time person_pos_stamp_;
    rclcpp::Time last_bike_time_;
    rclcpp::Time last_person_time_;

    double last_bike_pos_x_ = 0.0;
    double last_bike_pos_y_ = 0.0;
    double last_bike_yaw_ = 0.0;

    // 地图信息
    std::vector<std::vector<int>> grid_map_;
    double grid_resolution_ = 0.0;
    double grid_min_x_ = 0.0;
    double grid_min_y_ = 0.0;

    bool bike_bit_map = false;
    bool first_up = false;

    int hub_throttle_value_;
    int speed_limit;

    double bike_step = 0;
    double bike_f_yaw_ = 0;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BikeNode>();
    rclcpp::spin(node);
    node->closeSerialPort();
    rclcpp::shutdown();
    return 0;
}
