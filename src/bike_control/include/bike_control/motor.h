#ifndef BIKE_CONTROL_MOTOR_H
#define BIKE_CONTROL_MOTOR_H

#include <vector>
#include <cstdint>
#include <string>
#include <set>
#include <deque>
#include <serial/serial.h>


// 定义电机状态结构体
struct RotorMotorState {
    uint16_t voltage = 0;
    uint8_t motor_status = 0;
    int16_t speed = 0;
    float torque = 0.0f;
    int32_t position = 0;
    uint16_t fault_code = 0;
    uint8_t error_code = 0;
};

struct AuxWheelMotorState {
    float position = 0.0f;
    float speed = 0.0f;
    float load = 0.0f;
    uint8_t voltage = 0;
    uint8_t temperature = 0;
    uint8_t status = 0;
    uint8_t moving_flag = 0;
    uint16_t current = 0;
};

struct HubMotorState {
    uint16_t speed = 0;
    uint8_t error_code = 0;
};

class Motor {
public:
    explicit Motor(serial::Serial& serial);
    
    // 控制函数
    void controlRotorMotor(uint16_t speed, int16_t position);
    void controlLeftAuxiliaryWheelMotor(uint16_t speed, uint16_t position);
    void controlRightAuxiliaryWheelMotor(uint16_t speed, uint16_t position);
    void controlFrontBrakeMotor(uint16_t speed, uint16_t position);
    void controlRearBrakeMotor(uint16_t speed, uint16_t position);
    void controlHubMotor(uint8_t gear, uint16_t speed, uint8_t brake);
    
    // 处理接收到的帧
    bool processReceivedFrame();
    
    // 状态获取函数
    const RotorMotorState& getRotorState() const { return rotor_state_; }
    const AuxWheelMotorState& getLeftAuxState() const { return left_aux_state_; }
    const AuxWheelMotorState& getRightAuxState() const { return right_aux_state_; }
    const HubMotorState& getHubState() const { return hub_state_; }
    
private:
    serial::Serial& m_serial;
    std::vector<uint8_t> m_receive_buffer;
    std::set<uint8_t> controlled_motors;
    
    // 状态存储
    RotorMotorState rotor_state_;
    AuxWheelMotorState left_aux_state_;
    AuxWheelMotorState right_aux_state_;
    HubMotorState hub_state_;
    
    // 帧处理函数
    std::vector<uint8_t> build_packet(uint8_t object_id, uint8_t command, const std::vector<uint8_t>& data);
    bool tryExtractFrame(std::vector<uint8_t>& frame);
    bool processFrame(const std::vector<uint8_t>& frame);
    
    // 解析函数
    void parseRotorRealtime(const std::vector<uint8_t>& data);
    void parseAuxWheelRealtime(const std::vector<uint8_t>& data, bool is_left);
    void parseHubMotorRealtime(const std::vector<uint8_t>& data);
};

#endif // BIKE_CONTROL_MOTOR_H